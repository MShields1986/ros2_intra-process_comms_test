#include "ipc_zero_copy_tests/case_runner.hpp"

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

extern char ** environ;

namespace ipc_zero_copy_tests
{

std::map<std::string, CaseRunnerFn> & type_registry()
{
    static std::map<std::string, CaseRunnerFn> registry;
    return registry;
}

std::string ros_type_name(const std::string & message_type)
{
    if (message_type.find("/msg/") != std::string::npos) {
        return message_type;
    }
    const auto slash = message_type.find('/');
    if (slash == std::string::npos) {
        return message_type;
    }
    return message_type.substr(0, slash) + "/msg/" + message_type.substr(slash + 1);
}

ExternalSubscribers::ExternalSubscribers(
    const std::string & exe, const std::string & topic, const std::string & ros_type,
    int qos_depth, int count, const std::string & name_prefix)
{
    for (int k = 0; k < count; ++k) {
        std::vector<std::string> args = {
            exe,
            "--ros-args",
            "--log-level", "warn",
            "-r", "__node:=" + name_prefix + "_external_" + std::to_string(k),
            "-p", "topic:=" + topic,
            "-p", "message_type:=" + ros_type,
            "-p", "qos_depth:=" + std::to_string(qos_depth),
        };
        std::vector<char *> argv;
        for (auto & a : args) {
            argv.push_back(a.data());
        }
        argv.push_back(nullptr);

        pid_t pid = 0;
        const int rc = posix_spawn(&pid, exe.c_str(), nullptr, nullptr, argv.data(), environ);
        if (rc != 0) {
            throw std::runtime_error("failed to spawn '" + exe + "': " + std::strerror(rc));
        }
        pids_.push_back(pid);
    }
}

ExternalSubscribers::~ExternalSubscribers()
{
    for (pid_t pid : pids_) {
        kill(pid, SIGINT);
    }
    for (pid_t pid : pids_) {
        // Allow a clean shutdown (it logs its receive count), then force it.
        bool exited = false;
        for (int i = 0; i < 500 && !exited; ++i) {
            int status = 0;
            const pid_t r = waitpid(pid, &status, WNOHANG);
            exited = (r == pid) || (r < 0 && errno == ECHILD);
            if (!exited) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (!exited) {
            kill(pid, SIGKILL);
            waitpid(pid, nullptr, 0);
        }
    }
}

}  // namespace ipc_zero_copy_tests
