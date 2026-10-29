// Runs the zero-copy intra-process cases from the blog post for a chosen
// message type and payload size, prints a summary table and writes a CSV.

#include <sys/utsname.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "rmw/rmw.h"

#include "ipc_zero_copy_tests/case_runner.hpp"
#include "ipc_zero_copy_tests/cases.hpp"

namespace fs = std::filesystem;
using namespace ipc_zero_copy_tests;

namespace
{

// ---------------------------------------------------------------------------
// Host metadata (same fields as ros_latency_tests)
// ---------------------------------------------------------------------------

std::string env_or(const char * name, const std::string & fallback)
{
    const char * v = std::getenv(name);
    return v ? std::string(v) : fallback;
}

std::string read_first(const std::string & path, const std::string & key)
{
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind(key, 0) == 0) {
            const auto colon = line.find(':');
            if (colon != std::string::npos) {
                auto v = line.substr(colon + 1);
                v.erase(0, v.find_first_not_of(" \t"));
                return v;
            }
        }
    }
    return "?";
}

std::string os_pretty_name()
{
    std::ifstream f("/etc/os-release");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("PRETTY_NAME=", 0) == 0) {
            auto v = line.substr(12);
            v.erase(std::remove(v.begin(), v.end(), '"'), v.end());
            return v;
        }
    }
    return "?";
}

std::string timestamp(const char * fmt)
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), fmt, &tm);
    return buf;
}

std::string sibling_executable(const std::string & name)
{
    std::error_code ec;
    const auto self = fs::read_symlink("/proc/self/exe", ec);
    return ec ? name : (self.parent_path() / name).string();
}

// ---------------------------------------------------------------------------
// Case selection
// ---------------------------------------------------------------------------

std::vector<CaseSpec> select_cases(rclcpp::Node & cfg)
{
    // Dynamically typed so `-p cases:=11` (parsed as an integer) works as well
    // as `cases:=11,12,18`.
    rcl_interfaces::msg::ParameterDescriptor any_type;
    any_type.dynamic_typing = true;
    const auto cases_param = cfg.declare_parameter("cases", rclcpp::ParameterValue("all"), any_type);
    const std::string selection = cases_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
        std::to_string(cases_param.get<int64_t>()) : cases_param.get<std::string>();
    const auto custom_publish = cfg.declare_parameter<std::string>("custom_publish", "unique_ptr");
    const auto custom_subs = cfg.declare_parameter<std::string>("custom_subscriptions", "unique_ptr");
    const auto custom_external = cfg.declare_parameter<int>("custom_external_subscribers", 0);

    if (selection == "all") {
        return blog_cases();
    }
    if (selection == "custom") {
        CaseSpec spec;
        spec.id = 0;
        spec.description = "custom";
        spec.publish = parse_publish_mode(custom_publish);
        spec.subscriptions = parse_sub_kinds(custom_subs);
        spec.external_subscribers = static_cast<int>(custom_external);
        // No reference outcome for a custom case.
        spec.expected_copies = -1;
        spec.expected_payload_allocs = -1;
        if (spec.subscriptions.empty()) {
            throw std::invalid_argument("custom_subscriptions must name at least one subscription");
        }
        return {spec};
    }

    std::set<int> wanted;
    std::stringstream ss(selection);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) wanted.insert(std::stoi(tok));
    }
    std::vector<CaseSpec> out;
    for (const auto & c : blog_cases()) {
        if (wanted.erase(c.id)) out.push_back(c);
    }
    if (!wanted.empty()) {
        throw std::invalid_argument("unknown case id " + std::to_string(*wanted.begin()) +
            " (valid: 1-" + std::to_string(blog_cases().size()) + ", all, custom)");
    }
    return out;
}

// ---------------------------------------------------------------------------
// Result reduction
// ---------------------------------------------------------------------------

struct Summary
{
    int observed_copies = 0;
    bool mixed = false;
    uint64_t duplicates = 0;
    std::string per_sub;  // "unique_ptr=same;const_shared_ptr=copy"
    uint64_t bytes_median = 0, bytes_min = 0, bytes_max = 0;
    uint64_t count_median = 0;
    uint64_t large_median = 0;
    uint64_t bytes_all_median = 0, count_all_median = 0;
    bool alloc_check = false;  // payload-sized allocation count is meaningful
    std::string verdict;
};

template <typename F>
uint64_t median_of(const std::vector<alloc::Snapshot> & frames, F field)
{
    if (frames.empty()) return 0;
    std::vector<uint64_t> v;
    v.reserve(frames.size());
    for (const auto & f : frames) v.push_back(field(f));
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

Summary summarise(const CaseResult & r, bool ipc_enabled)
{
    Summary s;
    for (std::size_t i = 0; i < r.subs.size(); ++i) {
        const auto & sub = r.subs[i];
        std::string state;
        if (sub.received == 0) {
            state = "none";
        } else if (sub.copy == 0) {
            state = "same";
        } else if (sub.same == 0) {
            state = "copy";
            ++s.observed_copies;
        } else {
            state = "mixed(" + std::to_string(sub.copy) + "/" + std::to_string(sub.received) + ")";
            s.mixed = true;
        }
        if (sub.duplicates > 0) {
            // Delivered more than once per frame, e.g. intra-process plus an
            // RMW copy that rclcpp failed to recognise as its own.
            state += "+dup(" + std::to_string(sub.duplicates) + ")";
            s.duplicates += sub.duplicates;
        }
        if (i) s.per_sub += ";";
        s.per_sub += to_string(sub.kind) + "=" + state;
    }

    if (!r.frames.empty()) {
        s.bytes_median = median_of(r.frames, [](const auto & f) { return f.bytes; });
        s.count_median = median_of(r.frames, [](const auto & f) { return f.count; });
        s.large_median = median_of(r.frames, [](const auto & f) { return f.large; });
        s.bytes_all_median = median_of(r.frames, [](const auto & f) { return f.bytes_all_threads; });
        s.count_all_median = median_of(r.frames, [](const auto & f) { return f.count_all_threads; });
        const auto [mn, mx] = std::minmax_element(r.frames.begin(), r.frames.end(),
            [](const auto & a, const auto & b) { return a.bytes < b.bytes; });
        s.bytes_min = mn->bytes;
        s.bytes_max = mx->bytes;
    }
    s.alloc_check = r.large_threshold > 0;

    if (!r.error.empty()) {
        s.verdict = "ERROR";
    } else if (s.duplicates > 0) {
        s.verdict = "DUP";
    } else if (!ipc_enabled || r.spec.expected_copies < 0) {
        s.verdict = "n/a";
    } else if (s.mixed) {
        s.verdict = "MIXED";
    } else if (s.observed_copies != r.spec.expected_copies) {
        s.verdict = "DIFF";
    } else if (s.alloc_check && r.spec.expected_payload_allocs >= 0 &&
        s.large_median != static_cast<uint64_t>(r.spec.expected_payload_allocs))
    {
        s.verdict = "DIFF";
    } else {
        s.verdict = "PASS";
    }
    return s;
}

struct Unit
{
    const char * name;
    double scale;
};

// One unit per table, chosen from the payload size, so columns compare directly.
Unit unit_for(std::size_t payload_bytes)
{
    if (payload_bytes >= 1024 * 1024) return {"MiB", 1024.0 * 1024.0};
    if (payload_bytes >= 1024) return {"KiB", 1024.0};
    return {"B", 1.0};
}

std::string fmt_bytes(uint64_t bytes, const Unit & unit)
{
    std::ostringstream o;
    o << std::fixed << std::setprecision(unit.scale > 1.0 ? 2 : 0)
      << static_cast<double>(bytes) / unit.scale;
    return o.str();
}

std::string expected_str(int v, const char * unknown)
{
    return v < 0 ? unknown : std::to_string(v);
}

std::string csv_quote(const std::string & s)
{
    if (s.find_first_of(",\"\n") == std::string::npos) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += '"';
        out += c;
    }
    return out + "\"";
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto cfg_node = std::make_shared<rclcpp::Node>("ipc_case_runner");

    RunConfig cfg;
    cfg.message_type = cfg_node->declare_parameter<std::string>("message_type", "sensor_msgs/PointCloud2");
    cfg.payload_bytes = static_cast<std::size_t>(
        cfg_node->declare_parameter<int64_t>("payload_bytes", 1840000));
    cfg.iterations = static_cast<int>(cfg_node->declare_parameter<int>("iterations", 100));
    cfg.warmup_iterations = static_cast<int>(cfg_node->declare_parameter<int>("warmup_iterations", 10));
    cfg.use_intra_process_comms = cfg_node->declare_parameter<bool>("use_intra_process_comms", true);
    cfg.qos_depth = static_cast<int>(cfg_node->declare_parameter<int>("qos_depth", 10));
    cfg.discovery_timeout_s = cfg_node->declare_parameter<double>("discovery_timeout_s", 15.0);
    cfg.delivery_timeout_s = cfg_node->declare_parameter<double>("delivery_timeout_s", 5.0);
    cfg.settle_ms = cfg_node->declare_parameter<double>("settle_ms", 2.0);
    const auto output_dir = cfg_node->declare_parameter<std::string>("output_dir", "data/results");
    cfg.external_subscriber_exe = sibling_executable("ipc_external_subscriber");

    const auto & registry = type_registry();
    if (cfg.message_type == "list" || registry.find(cfg.message_type) == registry.end()) {
        if (cfg.message_type != "list") {
            std::fprintf(stderr, "Unsupported message_type '%s'. ", cfg.message_type.c_str());
        }
        std::fprintf(stderr, "Supported message types:\n");
        for (const auto & [name, fn] : registry) {
            std::fprintf(stderr, "  %s\n", name.c_str());
        }
        rclcpp::shutdown();
        return cfg.message_type == "list" ? 0 : 1;
    }
    const CaseRunnerFn run = registry.at(cfg.message_type);

    std::vector<CaseSpec> cases;
    try {
        cases = select_cases(*cfg_node);
    } catch (const std::exception & e) {
        std::fprintf(stderr, "%s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    // The config node only carries parameters; drop it so it does not take
    // part in discovery during the cases.
    cfg_node.reset();

    const std::string rmw = rmw_get_implementation_identifier();
    std::printf("ipc_case_runner: rclcpp %s, %s, %s, payload_bytes=%zu, %d+%d iterations, "
        "intra-process %s\n",
        RCLCPP_VERSION_STR, rmw.c_str(), cfg.message_type.c_str(), cfg.payload_bytes,
        cfg.warmup_iterations, cfg.iterations, cfg.use_intra_process_comms ? "on" : "off");
    std::fflush(stdout);

    std::vector<std::pair<CaseResult, Summary>> results;
    for (const auto & spec : cases) {
        if (!rclcpp::ok()) break;
        std::printf("[case %2d] %s ...\n", spec.id, spec.description.c_str());
        std::fflush(stdout);
        CaseResult r;
        try {
            r = run(spec, cfg);
        } catch (const std::exception & e) {
            r.spec = spec;
            r.error = e.what();
        }
        Summary s = summarise(r, cfg.use_intra_process_comms);
        if (!r.error.empty()) {
            std::printf("[case %2d] ERROR: %s\n", spec.id, r.error.c_str());
        } else {
            std::printf("[case %2d] matched subscriptions: %zu (%zu intra-process)\n",
                spec.id, r.matched_subscriptions, r.intra_process_subscriptions);
        }
        results.emplace_back(std::move(r), std::move(s));
    }

    // -----------------------------------------------------------------------
    // Summary table
    // -----------------------------------------------------------------------
    const std::size_t buffer_bytes = results.empty() ? 0 : results.front().first.buffer_bytes;
    const std::size_t serialized_bytes = results.empty() ? 0 : results.front().first.serialized_bytes;
    const Unit unit = unit_for(std::max(buffer_bytes, serialized_bytes));
    const std::string per_frame = std::string(unit.name) + "/frame";
    std::printf("\n%s, payload buffer %zu B, serialized %zu B, rmw %s\n",
        cfg.message_type.c_str(), buffer_bytes, serialized_bytes, rmw.c_str());
    std::printf("copies = in-process subscriptions handed a buffer other than the published one\n");
    std::printf("+dup(n) = n extra deliveries of a frame to one subscription (verdict DUP)\n");
    std::printf("payload allocs = allocations >= half the payload buffer on the publishing thread, per frame%s\n\n",
        (buffer_bytes >= kMinBufferBytesForAllocCheck) ? "" : " (n/a: buffer too small)");
    int sub_w = 16;
    for (const auto & [r, s] : results) {
        sub_w = std::max(sub_w, static_cast<int>(s.per_sub.size()));
    }
    std::printf("%-4s %-10s %-*s %-3s %-9s %-14s %-12s %-12s %s\n",
        "case", "publish", sub_w, "subscriptions", "ext", "copies", "payload allocs",
        per_frame.c_str(), per_frame.c_str(), "verdict");
    std::printf("%-4s %-10s %-*s %-3s %-9s %-14s %-12s %-12s %s\n",
        "", "", sub_w, "(per-sub result)", "", "exp/obs", "exp/obs", "(thread)", "(all thr.)", "");
    for (const auto & [r, s] : results) {
        const std::string copies = expected_str(r.spec.expected_copies, "-") + "/" +
            std::to_string(s.observed_copies);
        const std::string allocs = s.alloc_check ?
            expected_str(r.spec.expected_payload_allocs, "rmw") + "/" + std::to_string(s.large_median) :
            std::string("n/a");
        std::printf("%-4d %-10s %-*s %-3d %-9s %-14s %-12s %-12s %s\n",
            r.spec.id, to_string(r.spec.publish).c_str(), sub_w, s.per_sub.c_str(),
            r.spec.external_subscribers, copies.c_str(), allocs.c_str(),
            fmt_bytes(s.bytes_median, unit).c_str(), fmt_bytes(s.bytes_all_median, unit).c_str(),
            s.verdict.c_str());
    }
    std::printf("\n");

    // -----------------------------------------------------------------------
    // CSV
    // -----------------------------------------------------------------------
    std::string msg_tag = cfg.message_type;
    std::replace(msg_tag.begin(), msg_tag.end(), '/', '_');
    const std::string ts = timestamp("%Y%m%d_%H%M%S");
    const fs::path out_path = fs::path(output_dir) /
        ("ipc_" + rmw + "_" + msg_tag + "_" + std::to_string(serialized_bytes) + "B_" +
        (cfg.use_intra_process_comms ? "ipc" : "noipc") + "_" + ts + ".csv");

    std::error_code ec;
    fs::create_directories(output_dir, ec);
    std::ofstream f(out_path);
    if (!f) {
        std::fprintf(stderr, "failed to open %s for writing\n", out_path.c_str());
        rclcpp::shutdown();
        return 1;
    }

    utsname uts{};
    (void)uname(&uts);
    char hostname[256] = {0};
    (void)gethostname(hostname, sizeof(hostname) - 1);

    f << "# rmw=" << rmw << "\n"
      << "# rclcpp=" << RCLCPP_VERSION_STR << "\n"
      << "# ros_distro=" << env_or("ROS_DISTRO", "?") << "\n"
      << "# message_type=" << cfg.message_type << "\n"
      << "# payload_bytes_requested=" << cfg.payload_bytes << "\n"
      << "# payload_buffer_bytes=" << buffer_bytes << "\n"
      << "# serialized_bytes=" << serialized_bytes << "\n"
      << "# iterations=" << cfg.iterations << "\n"
      << "# warmup_iterations=" << cfg.warmup_iterations << "\n"
      << "# use_intra_process_comms=" << (cfg.use_intra_process_comms ? "true" : "false") << "\n"
      << "# qos_depth=" << cfg.qos_depth << "\n"
      << "# start_time=" << timestamp("%Y-%m-%dT%H:%M:%S%z") << "\n"
      << "# host=" << hostname << "\n"
      << "# cpu_model=" << read_first("/proc/cpuinfo", "model name") << "\n"
      << "# cpu_cores=" << std::thread::hardware_concurrency() << "\n"
      << "# os=" << os_pretty_name() << "\n"
      << "# kernel=" << uts.sysname << " " << uts.release << " " << uts.machine << "\n";

    f << "case,description,publish,subscriptions,external_subscribers,per_subscription,"
         "expected_copies,observed_copies,duplicate_deliveries,expected_payload_allocs,payload_allocs_per_frame,"
         "alloc_bytes_per_frame,alloc_bytes_min,alloc_bytes_max,alloc_count_per_frame,"
         "alloc_bytes_all_threads_per_frame,alloc_count_all_threads_per_frame,frames,verdict,error\n";
    for (const auto & [r, s] : results) {
        f << r.spec.id << ","
          << csv_quote(r.spec.description) << ","
          << to_string(r.spec.publish) << ","
          << csv_quote(to_string(r.spec.subscriptions)) << ","
          << r.spec.external_subscribers << ","
          << csv_quote(s.per_sub) << ","
          << (r.spec.expected_copies < 0 ? "" : std::to_string(r.spec.expected_copies)) << ","
          << s.observed_copies << ","
          << s.duplicates << ","
          << (r.spec.expected_payload_allocs < 0 ? "" : std::to_string(r.spec.expected_payload_allocs)) << ","
          << (s.alloc_check ? std::to_string(s.large_median) : "") << ","
          << s.bytes_median << ","
          << s.bytes_min << ","
          << s.bytes_max << ","
          << s.count_median << ","
          << s.bytes_all_median << ","
          << s.count_all_median << ","
          << r.frames.size() << ","
          << s.verdict << ","
          << csv_quote(r.error) << "\n";
    }
    std::printf("wrote %s\n", out_path.c_str());

    rclcpp::shutdown();

    const bool any_error = std::any_of(results.begin(), results.end(),
        [](const auto & p) { return !p.first.error.empty(); });
    return any_error ? 1 : 0;
}
