#pragma once

#include <sys/types.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "ipc_zero_copy_tests/alloc_counter.hpp"
#include "ipc_zero_copy_tests/cases.hpp"
#include "ipc_zero_copy_tests/payload_traits.hpp"

namespace ipc_zero_copy_tests
{

// Payload buffers smaller than this are not checked for payload-sized
// allocations: at that size a deep copy is indistinguishable from the small
// bookkeeping allocations rclcpp makes on every publish.
inline constexpr std::size_t kMinBufferBytesForAllocCheck = 4096;

struct RunConfig
{
    std::string message_type;  // "pkg/Type", e.g. "sensor_msgs/PointCloud2"
    std::size_t payload_bytes = 0;
    int iterations = 100;
    int warmup_iterations = 10;
    bool use_intra_process_comms = true;
    int qos_depth = 10;
    double discovery_timeout_s = 15.0;
    double delivery_timeout_s = 5.0;
    // After every subscription has the frame, keep spinning until nothing new
    // arrives for this long, to catch duplicate deliveries.
    double settle_ms = 2.0;
    std::string topic_prefix = "/ipc_zero_copy";
    std::string external_subscriber_exe;
};

struct SubResult
{
    SubKind kind = SubKind::UniquePtr;
    uint64_t received = 0;    // measured frames delivered at least once
    uint64_t same = 0;        // ... of which one delivery was the published buffer
    uint64_t copy = 0;        // ... of which every delivery was a deep copy
    uint64_t duplicates = 0;  // deliveries beyond the first, summed over frames
};

struct CaseResult
{
    CaseSpec spec;
    std::size_t buffer_bytes = 0;       // size of the variable-length field (0 = fixed-size type)
    std::size_t serialized_bytes = 0;   // CDR size of the message
    std::size_t large_threshold = 0;    // payload-sized allocation threshold (0 = disabled)
    // Publisher's view after discovery: all matched subscriptions, and how
    // many of those rclcpp delivers to intra-process. rclcpp also publishes
    // inter-process whenever matched > intra.
    std::size_t matched_subscriptions = 0;
    std::size_t intra_process_subscriptions = 0;
    std::vector<SubResult> subs;
    std::vector<alloc::Snapshot> frames;  // one per measured iteration
    std::string error;
};

// "sensor_msgs/PointCloud2" -> "sensor_msgs/msg/PointCloud2"
std::string ros_type_name(const std::string & message_type);

// Spawns `count` copies of the ipc_external_subscriber executable (a generic,
// serialized subscription in its own process) on construction and stops them
// on destruction.
class ExternalSubscribers
{
public:
    ExternalSubscribers(
        const std::string & exe, const std::string & topic, const std::string & ros_type,
        int qos_depth, int count, const std::string & name_prefix);
    ~ExternalSubscribers();

    ExternalSubscribers(const ExternalSubscribers &) = delete;
    ExternalSubscribers & operator=(const ExternalSubscribers &) = delete;

private:
    std::vector<pid_t> pids_;
};

namespace detail
{

template <typename MsgT>
std::size_t serialized_size(const MsgT & msg)
{
    rclcpp::Serialization<MsgT> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&msg, &serialized);
    return serialized.size();
}

// Records what one subscription received. Written only from the executor
// thread, which is also the thread that publishes.
struct Probe
{
    const void * const * published = nullptr;
    uint32_t frame_hits = 0;
    bool frame_same = false;
    SubResult result;

    void hit(const void * received)
    {
        ++frame_hits;
        // Once the published buffer has been freed its address can come back
        // from the allocator for a copy, so equality alone is not enough.
        if (received == *published && !alloc::reused()) {
            frame_same = true;
        }
    }

    void end_frame(bool measuring)
    {
        if (measuring && frame_hits > 0) {
            ++result.received;
            if (frame_same) {
                ++result.same;
            } else {
                ++result.copy;
            }
            result.duplicates += frame_hits - 1;
        }
        frame_hits = 0;
        frame_same = false;
    }
};

template <typename MsgT>
rclcpp::SubscriptionBase::SharedPtr make_subscription(
    rclcpp::Node & node, const std::string & topic, const rclcpp::QoS & qos,
    SubKind kind, Probe * probe)
{
    switch (kind) {
        case SubKind::UniquePtr:
            return node.create_subscription<MsgT>(topic, qos,
                [probe](std::unique_ptr<MsgT> msg) { probe->hit(payload_buffer(*msg)); });
        case SubKind::SharedPtr:
            // Deprecated signature; rclcpp treats it as take-ownership.
            return node.create_subscription<MsgT>(topic, qos,
                [probe](std::shared_ptr<MsgT> msg) { probe->hit(payload_buffer(*msg)); });
        case SubKind::ConstSharedPtr:
            return node.create_subscription<MsgT>(topic, qos,
                [probe](std::shared_ptr<const MsgT> msg) { probe->hit(payload_buffer(*msg)); });
        case SubKind::ConstSharedPtrRef:
            return node.create_subscription<MsgT>(topic, qos,
                [probe](const std::shared_ptr<const MsgT> & msg) { probe->hit(payload_buffer(*msg)); });
        case SubKind::ConstRef:
            return node.create_subscription<MsgT>(topic, qos,
                [probe](const MsgT & msg) { probe->hit(payload_buffer(msg)); });
    }
    return nullptr;
}

// Spin until every probe has seen the current frame, then keep spinning until
// no further deliveries arrive within `settle`. The executor runs ready
// subscriptions before ready waitables (which is how intra-process deliveries
// arrive), so stopping at the first delivery could leave the intra-process
// copy unprocessed behind a duplicate that came through the RMW.
inline bool spin_until_delivered(
    rclcpp::Executor & executor, std::vector<Probe> & probes,
    std::chrono::steady_clock::time_point deadline, std::chrono::nanoseconds settle)
{
    auto all_delivered = [&probes]() {
        for (const auto & p : probes) {
            if (p.frame_hits == 0) return false;
        }
        return true;
    };
    auto total_hits = [&probes]() {
        uint64_t n = 0;
        for (const auto & p : probes) n += p.frame_hits;
        return n;
    };
    while (!all_delivered()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        executor.spin_once(std::chrono::milliseconds(10));
    }
    for (;;) {
        const uint64_t before = total_hits();
        executor.spin_some();
        if (total_hits() == before) {
            executor.spin_once(settle);
            if (total_hits() == before) break;
        }
    }
    return true;
}

}  // namespace detail

// Run one case for message type MsgT. Each case gets a fresh node, topic and
// executor so cases cannot interfere with each other.
template <typename MsgT>
CaseResult run_case(const CaseSpec & spec, const RunConfig & cfg)
{
    using Clock = std::chrono::steady_clock;

    CaseResult result;
    result.spec = spec;

    MsgT prototype;
    fill_payload(prototype, cfg.payload_bytes);
    result.buffer_bytes = payload_buffer_bytes(prototype);
    result.serialized_bytes = detail::serialized_size(prototype);
    // Any allocation of at least half the payload buffer is counted as a copy
    // of it (a copy allocates exactly the buffer size).
    result.large_threshold =
        result.buffer_bytes >= kMinBufferBytesForAllocCheck ? result.buffer_bytes / 2 : 0;

    const std::string node_name = "ipc_case_" + std::to_string(spec.id);
    const std::string topic = cfg.topic_prefix + "/case_" + std::to_string(spec.id);

    ExternalSubscribers external(
        cfg.external_subscriber_exe, topic, ros_type_name(cfg.message_type),
        cfg.qos_depth, spec.external_subscribers, node_name);

    auto node = std::make_shared<rclcpp::Node>(
        node_name,
        rclcpp::NodeOptions()
            .use_intra_process_comms(cfg.use_intra_process_comms)
            .start_parameter_services(false)
            .start_parameter_event_publisher(false));

    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(cfg.qos_depth))};
    qos.reliable().durability_volatile();

    const void * published = nullptr;

    std::vector<detail::Probe> probes(spec.subscriptions.size());
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    for (std::size_t i = 0; i < spec.subscriptions.size(); ++i) {
        probes[i].published = &published;
        probes[i].result.kind = spec.subscriptions[i];
        subscriptions.push_back(
            detail::make_subscription<MsgT>(*node, topic, qos, spec.subscriptions[i], &probes[i]));
    }
    auto publisher = node->create_publisher<MsgT>(topic, qos);

    // Wait until every subscription, in this process and out of it, is
    // matched. Otherwise the publisher may skip the inter-process path.
    const std::size_t expected_matches =
        subscriptions.size() + static_cast<std::size_t>(spec.external_subscribers);
    const auto discovery_deadline =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(cfg.discovery_timeout_s));
    while (publisher->get_subscription_count() < expected_matches) {
        if (Clock::now() > discovery_deadline) {
            result.error = "only " + std::to_string(publisher->get_subscription_count()) + "/" +
                std::to_string(expected_matches) + " subscriptions matched within " +
                std::to_string(cfg.discovery_timeout_s) + " s";
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    result.matched_subscriptions = publisher->get_subscription_count();
    result.intra_process_subscriptions = publisher->get_intra_process_subscription_count();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto delivery_timeout = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(cfg.delivery_timeout_s));
    const auto settle = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(cfg.settle_ms));
    const int total = cfg.warmup_iterations + cfg.iterations;
    result.frames.reserve(static_cast<std::size_t>(cfg.iterations));

    for (int it = 0; it < total; ++it) {
        const bool measuring = it >= cfg.warmup_iterations;

        // The message is constructed outside the window: only what rclcpp
        // allocates during publish() and dispatch is counted.
        bool delivered = false;
        alloc::Snapshot snap;
        if (spec.publish == PublishMode::UniquePtr) {
            auto msg = std::make_unique<MsgT>(prototype);
            published = payload_buffer(*msg);
            alloc::watch(published);
            alloc::begin(result.large_threshold);
            publisher->publish(std::move(msg));
            delivered = detail::spin_until_delivered(
                executor, probes, Clock::now() + delivery_timeout, settle);
            snap = alloc::end();
        } else {
            MsgT msg(prototype);
            published = payload_buffer(msg);
            alloc::watch(published);
            alloc::begin(result.large_threshold);
            publisher->publish(msg);
            delivered = detail::spin_until_delivered(
                executor, probes, Clock::now() + delivery_timeout, settle);
            snap = alloc::end();
        }

        for (auto & p : probes) {
            p.end_frame(measuring);
        }
        if (!delivered) {
            result.error = "iteration " + std::to_string(it) +
                " not delivered to every subscription within " +
                std::to_string(cfg.delivery_timeout_s) + " s";
            break;
        }
        if (measuring) {
            result.frames.push_back(snap);
        }
    }

    executor.remove_node(node);
    for (const auto & p : probes) {
        result.subs.push_back(p.result);
    }
    return result;
}

// ---------------------------------------------------------------------------
// Message type registry: "pkg/Type" -> run_case<MsgT>. Populated by the
// generated per-type translation units (see CMakeLists.txt).
// ---------------------------------------------------------------------------
using CaseRunnerFn = CaseResult (*)(const CaseSpec &, const RunConfig &);

std::map<std::string, CaseRunnerFn> & type_registry();

struct TypeRegistrar
{
    TypeRegistrar(const char * name, CaseRunnerFn fn) { type_registry()[name] = fn; }
};

}  // namespace ipc_zero_copy_tests

#define IPC_ZERO_COPY_REGISTER_TYPE(MSG_T, NAME)                                 \
    static const ::ipc_zero_copy_tests::TypeRegistrar ipc_zero_copy_type_registrar{ \
        NAME, &::ipc_zero_copy_tests::run_case<MSG_T>};
