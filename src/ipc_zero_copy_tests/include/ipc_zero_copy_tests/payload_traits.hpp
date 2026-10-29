#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

// Per-type hooks used by the case runner. Three free functions:
//
//   fill_payload(msg, bytes)   size the variable-length field to ~bytes
//   payload_buffer(msg)        address used for the zero-copy check
//   payload_buffer_bytes(msg)  size of that buffer (0 for fixed-size types)
//
// The generic templates cover fixed-size types: nothing to fill, and the
// message object's own address is the identity checked on receipt. Types with
// a variable-length field get non-template overloads below, which overload
// resolution prefers. The address checked is then the field's heap buffer
// (e.g. PointCloud2::data.data()), exactly as in the blog post.

namespace ipc_zero_copy_tests
{

template <typename T>
inline void fill_payload(T & /*msg*/, std::size_t /*target_bytes*/) {}

template <typename T>
inline const void * payload_buffer(const T & msg) { return &msg; }

template <typename T>
inline std::size_t payload_buffer_bytes(const T & /*msg*/) { return 0; }

namespace detail
{

template <typename Msg, typename Container>
inline const void * buffer_of(const Msg & msg, const Container & c)
{
    // An empty container has no heap buffer; fall back to the message itself.
    return c.empty() ? static_cast<const void *>(&msg) : static_cast<const void *>(c.data());
}

template <typename Container>
inline std::size_t bytes_of(const Container & c)
{
    return c.size() * sizeof(typename Container::value_type);
}

}  // namespace detail

#define IPC_ZERO_COPY_PAYLOAD_FIELD(MSG_T, FIELD)                              \
    inline const void * payload_buffer(const MSG_T & msg)                      \
    {                                                                          \
        return detail::buffer_of(msg, msg.FIELD);                              \
    }                                                                          \
    inline std::size_t payload_buffer_bytes(const MSG_T & msg)                 \
    {                                                                          \
        return detail::bytes_of(msg.FIELD);                                    \
    }

// ---------------------------------------------------------------------------
// sensor_msgs
// ---------------------------------------------------------------------------

// XYZ + intensity float32, 16-byte point step — the layout used in the blog
// (115k points = 1.75 MiB at the default payload_bytes).
inline void fill_payload(sensor_msgs::msg::PointCloud2 & msg, std::size_t target_bytes)
{
    using sensor_msgs::msg::PointField;
    const char * names[] = {"x", "y", "z", "intensity"};
    msg.fields.resize(4);
    for (std::size_t i = 0; i < 4; ++i) {
        msg.fields[i].name = names[i];
        msg.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
        msg.fields[i].datatype = PointField::FLOAT32;
        msg.fields[i].count = 1;
    }
    msg.height = 1;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.point_step = 16;
    msg.width = static_cast<uint32_t>(target_bytes / msg.point_step);
    msg.row_step = msg.point_step * msg.width;
    msg.data.assign(msg.row_step, 0u);
}
IPC_ZERO_COPY_PAYLOAD_FIELD(sensor_msgs::msg::PointCloud2, data)

inline void fill_payload(sensor_msgs::msg::Image & msg, std::size_t target_bytes)
{
    // Single-row mono8 with a width chosen to hit the target byte count.
    msg.encoding = "mono8";
    msg.is_bigendian = 0;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(target_bytes);
    msg.step = msg.width;
    msg.data.assign(target_bytes, 0u);
}
IPC_ZERO_COPY_PAYLOAD_FIELD(sensor_msgs::msg::Image, data)

inline void fill_payload(sensor_msgs::msg::LaserScan & msg, std::size_t target_bytes)
{
    const std::size_t n = target_bytes / sizeof(float);
    msg.angle_min = -3.14159f;
    msg.angle_max = 3.14159f;
    msg.angle_increment = (n > 0) ? (6.28318f / static_cast<float>(n)) : 0.0f;
    msg.range_min = 0.0f;
    msg.range_max = 100.0f;
    msg.ranges.assign(n, 1.0f);
    msg.intensities.clear();
}
IPC_ZERO_COPY_PAYLOAD_FIELD(sensor_msgs::msg::LaserScan, ranges)

inline void fill_payload(sensor_msgs::msg::PointCloud & msg, std::size_t target_bytes)
{
    msg.points.assign(target_bytes / (3 * sizeof(float)), geometry_msgs::msg::Point32());
    msg.channels.clear();
}
IPC_ZERO_COPY_PAYLOAD_FIELD(sensor_msgs::msg::PointCloud, points)

inline void fill_payload(sensor_msgs::msg::JointState & msg, std::size_t target_bytes)
{
    msg.name.clear();
    msg.position.assign(target_bytes / sizeof(double), 0.0);
    msg.velocity.clear();
    msg.effort.clear();
}
IPC_ZERO_COPY_PAYLOAD_FIELD(sensor_msgs::msg::JointState, position)

// ---------------------------------------------------------------------------
// nav_msgs
// ---------------------------------------------------------------------------

inline void fill_payload(nav_msgs::msg::Path & msg, std::size_t target_bytes)
{
    // 7 doubles of pose data per PoseStamped.
    msg.poses.assign(target_bytes / (7 * sizeof(double)), geometry_msgs::msg::PoseStamped());
}
IPC_ZERO_COPY_PAYLOAD_FIELD(nav_msgs::msg::Path, poses)

// ---------------------------------------------------------------------------
// std_msgs
// ---------------------------------------------------------------------------

inline void fill_payload(std_msgs::msg::String & msg, std::size_t target_bytes)
{
    msg.data.assign(target_bytes, 'x');
}
IPC_ZERO_COPY_PAYLOAD_FIELD(std_msgs::msg::String, data)

#define IPC_ZERO_COPY_MULTI_ARRAY(MSG_T)                                         \
    inline void fill_payload(MSG_T & msg, std::size_t target_bytes)              \
    {                                                                            \
        using elem_t = typename decltype(msg.data)::value_type;                  \
        msg.data.assign(target_bytes / sizeof(elem_t), elem_t{});                \
        msg.layout.dim.clear();                                                  \
        msg.layout.data_offset = 0;                                              \
    }                                                                            \
    IPC_ZERO_COPY_PAYLOAD_FIELD(MSG_T, data)

IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::ByteMultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::Float32MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::Float64MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::Int8MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::Int16MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::Int32MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::Int64MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::UInt8MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::UInt16MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::UInt32MultiArray)
IPC_ZERO_COPY_MULTI_ARRAY(std_msgs::msg::UInt64MultiArray)

#undef IPC_ZERO_COPY_MULTI_ARRAY
#undef IPC_ZERO_COPY_PAYLOAD_FIELD

}  // namespace ipc_zero_copy_tests
