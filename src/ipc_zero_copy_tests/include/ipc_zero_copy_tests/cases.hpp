#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace ipc_zero_copy_tests
{

// How the publisher hands the message to rclcpp.
enum class PublishMode
{
    UniquePtr,  // pub->publish(std::move(unique_ptr))
    ConstRef,   // pub->publish(msg)  — "by value" in the blog; binds publish(const T &)
};

// Subscription callback signatures.
enum class SubKind
{
    UniquePtr,          // void(std::unique_ptr<T>)                  — take-ownership
    SharedPtr,          // void(std::shared_ptr<T>)                  — deprecated, take-ownership
    ConstSharedPtr,     // void(std::shared_ptr<const T>)            — take-shared
    ConstSharedPtrRef,  // void(const std::shared_ptr<const T> &)    — take-shared
    ConstRef,           // void(const T &)                           — take-shared
};

struct CaseSpec
{
    int id = 0;
    std::string description;
    PublishMode publish = PublishMode::UniquePtr;
    std::vector<SubKind> subscriptions;  // all in the publisher's process
    int external_subscribers = 0;        // subscriber processes spawned per case
    // Expected outcome from the blog post (rclcpp 32.0.0):
    //   expected_copies          in-process subscriptions that receive a buffer
    //                            other than the one published
    //   expected_payload_allocs  payload-sized allocations on the publishing
    //                            thread per frame; -1 when the RMW contributes
    //                            (out-of-process cases), so no fixed value
    int expected_copies = 0;
    int expected_payload_allocs = 0;
};

// The 20 cases from the blog post, numbered identically.
const std::vector<CaseSpec> & blog_cases();

std::string to_string(PublishMode mode);
std::string to_string(SubKind kind);
// Comma-joined subscription kinds, e.g. "unique_ptr,const_ref".
std::string to_string(const std::vector<SubKind> & kinds);

// Parse the tokens above (throws std::invalid_argument on unknown input).
PublishMode parse_publish_mode(const std::string & s);
SubKind parse_sub_kind(const std::string & s);
std::vector<SubKind> parse_sub_kinds(const std::string & csv);

}  // namespace ipc_zero_copy_tests
