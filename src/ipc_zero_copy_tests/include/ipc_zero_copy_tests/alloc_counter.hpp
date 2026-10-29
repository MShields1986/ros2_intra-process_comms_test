#pragma once

#include <cstddef>
#include <cstdint>

namespace ipc_zero_copy_tests
{
namespace alloc
{

// Byte/count totals for one measurement window.
//
// The global operator new family is overridden in alloc_counter.cpp. While a
// window is open every allocation in the process is added to the *_all_threads
// totals; allocations made on the thread that opened the window (the one that
// calls publish() and spins the executor) are also added to the thread totals.
//
// Only operator new is hooked — direct malloc() calls from C code (rcutils,
// DDS internals) are not counted. This matches the method used in the blog.
struct Snapshot
{
    uint64_t bytes = 0;
    uint64_t count = 0;
    // Thread-local allocations of at least `large_threshold` bytes; used to
    // count payload-sized allocations (i.e. deep copies of the message buffer).
    uint64_t large = 0;
    uint64_t bytes_all_threads = 0;
    uint64_t count_all_threads = 0;
};

// Open a window on the calling thread. `large_threshold` of 0 disables the
// large-allocation counter.
void begin(std::size_t large_threshold);

// Close the window and return its totals.
Snapshot end();

// Watch an address for reuse. Once the published buffer is freed, the
// allocator may return the same address for a copy, which would then look
// like zero-copy delivery. reused() reports whether any allocation has
// returned `p` since watch(p) was called.
void watch(const void * p);
bool reused();

}  // namespace alloc
}  // namespace ipc_zero_copy_tests
