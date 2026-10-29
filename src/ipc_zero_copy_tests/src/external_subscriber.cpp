// Out-of-process subscriber for the cases that need one (11, 12, 18).
//
// Uses a generic (serialized) subscription so one executable serves every
// message type. Its only job is to exist and be matched, which makes the
// publisher in the case runner take the inter-process path as well.

#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ipc_external_subscriber");

    const auto topic = node->declare_parameter<std::string>("topic", "/ipc_zero_copy/case_0");
    const auto type = node->declare_parameter<std::string>("message_type", "sensor_msgs/msg/PointCloud2");
    const auto depth = node->declare_parameter<int>("qos_depth", 10);

    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(depth))};
    qos.reliable().durability_volatile();

    std::atomic<uint64_t> received{0};
    auto sub = node->create_generic_subscription(
        topic, type, qos,
        [&received](std::shared_ptr<const rclcpp::SerializedMessage>) { ++received; });

    rclcpp::spin(node);

    // The context is already shut down here, so print rather than log.
    std::printf("[ipc_external_subscriber] received %" PRIu64 " messages on %s\n",
        received.load(), topic.c_str());
    std::fflush(stdout);
    rclcpp::shutdown();
    return 0;
}
