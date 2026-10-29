#include "ipc_zero_copy_tests/cases.hpp"

#include <sstream>
#include <stdexcept>

namespace ipc_zero_copy_tests
{

namespace
{
constexpr auto U = PublishMode::UniquePtr;
constexpr auto V = PublishMode::ConstRef;

constexpr auto UP = SubKind::UniquePtr;
constexpr auto SP = SubKind::SharedPtr;
constexpr auto CSP = SubKind::ConstSharedPtr;
constexpr auto CSPR = SubKind::ConstSharedPtrRef;
constexpr auto CR = SubKind::ConstRef;
}  // namespace

const std::vector<CaseSpec> & blog_cases()
{
    // id, description, publish, subscriptions, external, expected copies, expected payload allocs
    static const std::vector<CaseSpec> cases = {
        {1,  "unique_ptr -> 1x UniquePtr",                                  U, {UP},           0, 0,  0},
        {2,  "unique_ptr -> 1x ConstSharedPtr",                             U, {CSP},          0, 0,  0},
        {3,  "unique_ptr -> 1x mutable SharedPtr (deprecated)",             U, {SP},           0, 0,  0},
        {4,  "by value -> 1x UniquePtr",                                    V, {UP},           0, 1,  1},
        {5,  "by value -> 1x ConstSharedPtr",                               V, {CSP},          0, 1,  1},
        {6,  "unique_ptr -> 2x UniquePtr",                                  U, {UP, UP},       0, 1,  1},
        {7,  "unique_ptr -> 3x ConstSharedPtr",                             U, {CSP, CSP, CSP}, 0, 0, 0},
        {8,  "unique_ptr -> 1x UniquePtr + 1x ConstSharedPtr",              U, {UP, CSP},      0, 1,  1},
        {9,  "unique_ptr -> 1x UniquePtr + 2x ConstSharedPtr",              U, {UP, CSP, CSP}, 0, 2,  1},
        {10, "unique_ptr -> 2x mutable SharedPtr",                          U, {SP, SP},       0, 1,  1},
        {11, "unique_ptr -> 1x UniquePtr + out-of-process sub",             U, {UP},           1, 0, -1},
        {12, "unique_ptr -> 1x ConstSharedPtr + out-of-process sub",        U, {CSP},          1, 0, -1},
        {13, "unique_ptr -> 1x const T&",                                   U, {CR},           0, 0,  0},
        {14, "unique_ptr -> 3x const T&",                                   U, {CR, CR, CR},   0, 0,  0},
        {15, "unique_ptr -> 1x const T& + 1x ConstSharedPtr",               U, {CR, CSP},      0, 0,  0},
        {16, "by value -> 1x const T&",                                     V, {CR},           0, 1,  1},
        {17, "unique_ptr -> 1x const T& + 1x UniquePtr",                    U, {CR, UP},       0, 1,  1},
        {18, "unique_ptr -> 1x const T& + out-of-process sub",              U, {CR},           1, 0, -1},
        {19, "unique_ptr -> 1x const ConstSharedPtr&",                      U, {CSPR},         0, 0,  0},
        {20, "unique_ptr -> 1x const T& + 1x ConstSharedPtr + 1x const ConstSharedPtr&",
                                                                            U, {CR, CSP, CSPR}, 0, 0, 0},
    };
    return cases;
}

std::string to_string(PublishMode mode)
{
    switch (mode) {
        case PublishMode::UniquePtr: return "unique_ptr";
        case PublishMode::ConstRef:  return "const_ref";
    }
    return "?";
}

std::string to_string(SubKind kind)
{
    switch (kind) {
        case SubKind::UniquePtr:         return "unique_ptr";
        case SubKind::SharedPtr:         return "shared_ptr";
        case SubKind::ConstSharedPtr:    return "const_shared_ptr";
        case SubKind::ConstSharedPtrRef: return "const_shared_ptr_ref";
        case SubKind::ConstRef:          return "const_ref";
    }
    return "?";
}

std::string to_string(const std::vector<SubKind> & kinds)
{
    std::string out;
    for (std::size_t i = 0; i < kinds.size(); ++i) {
        if (i) out += ",";
        out += to_string(kinds[i]);
    }
    return out;
}

PublishMode parse_publish_mode(const std::string & s)
{
    if (s == "unique_ptr") return PublishMode::UniquePtr;
    if (s == "const_ref" || s == "value") return PublishMode::ConstRef;
    throw std::invalid_argument("unknown publish mode '" + s + "' (expected unique_ptr|const_ref)");
}

SubKind parse_sub_kind(const std::string & s)
{
    if (s == "unique_ptr")           return SubKind::UniquePtr;
    if (s == "shared_ptr")           return SubKind::SharedPtr;
    if (s == "const_shared_ptr")     return SubKind::ConstSharedPtr;
    if (s == "const_shared_ptr_ref") return SubKind::ConstSharedPtrRef;
    if (s == "const_ref")            return SubKind::ConstRef;
    throw std::invalid_argument(
        "unknown subscription kind '" + s +
        "' (expected unique_ptr|shared_ptr|const_shared_ptr|const_shared_ptr_ref|const_ref)");
}

std::vector<SubKind> parse_sub_kinds(const std::string & csv)
{
    std::vector<SubKind> out;
    std::stringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(parse_sub_kind(tok));
    }
    return out;
}

}  // namespace ipc_zero_copy_tests
