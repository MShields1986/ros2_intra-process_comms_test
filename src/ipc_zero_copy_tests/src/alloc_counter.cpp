#include "ipc_zero_copy_tests/alloc_counter.hpp"

#include <atomic>
#include <cstdlib>
#include <limits>
#include <new>

namespace
{

std::atomic<bool> g_active{false};
std::atomic<uint64_t> g_bytes_all{0};
std::atomic<uint64_t> g_count_all{0};
std::size_t g_large_threshold = std::numeric_limits<std::size_t>::max();

std::atomic<const void *> g_watched{nullptr};
std::atomic<bool> g_watched_reused{false};

thread_local bool t_measured = false;
thread_local uint64_t t_bytes = 0;
thread_local uint64_t t_count = 0;
thread_local uint64_t t_large = 0;

inline void record(std::size_t n) noexcept
{
    if (!g_active.load(std::memory_order_relaxed)) {
        return;
    }
    g_bytes_all.fetch_add(n, std::memory_order_relaxed);
    g_count_all.fetch_add(1, std::memory_order_relaxed);
    if (t_measured) {
        t_bytes += n;
        ++t_count;
        if (n >= g_large_threshold) {
            ++t_large;
        }
    }
}

inline void check_watched(const void * p) noexcept
{
    if (p && p == g_watched.load(std::memory_order_relaxed)) {
        g_watched_reused.store(true, std::memory_order_relaxed);
    }
}

inline void * raw_alloc(std::size_t n) noexcept
{
    record(n);
    void * p = std::malloc(n ? n : 1);
    check_watched(p);
    return p;
}

inline void * raw_aligned_alloc(std::size_t n, std::align_val_t al) noexcept
{
    record(n);
    std::size_t alignment = static_cast<std::size_t>(al);
    if (alignment < sizeof(void *)) {
        alignment = sizeof(void *);
    }
    void * p = nullptr;
    if (posix_memalign(&p, alignment, n ? n : 1) != 0) {
        return nullptr;
    }
    check_watched(p);
    return p;
}

inline void * alloc_or_throw(std::size_t n)
{
    void * p = raw_alloc(n);
    if (!p) {
        throw std::bad_alloc();
    }
    return p;
}

inline void * aligned_alloc_or_throw(std::size_t n, std::align_val_t al)
{
    void * p = raw_aligned_alloc(n, al);
    if (!p) {
        throw std::bad_alloc();
    }
    return p;
}

}  // namespace

namespace ipc_zero_copy_tests
{
namespace alloc
{

void begin(std::size_t large_threshold)
{
    g_large_threshold = large_threshold ? large_threshold : std::numeric_limits<std::size_t>::max();
    g_bytes_all.store(0, std::memory_order_relaxed);
    g_count_all.store(0, std::memory_order_relaxed);
    t_bytes = 0;
    t_count = 0;
    t_large = 0;
    t_measured = true;
    g_active.store(true, std::memory_order_seq_cst);
}

Snapshot end()
{
    g_active.store(false, std::memory_order_seq_cst);
    t_measured = false;
    Snapshot s;
    s.bytes = t_bytes;
    s.count = t_count;
    s.large = t_large;
    s.bytes_all_threads = g_bytes_all.load(std::memory_order_relaxed);
    s.count_all_threads = g_count_all.load(std::memory_order_relaxed);
    return s;
}

void watch(const void * p)
{
    g_watched.store(p, std::memory_order_relaxed);
    g_watched_reused.store(false, std::memory_order_relaxed);
}

bool reused()
{
    return g_watched_reused.load(std::memory_order_relaxed);
}

}  // namespace alloc
}  // namespace ipc_zero_copy_tests

// ----------------------------------------------------------------------------
// Global replacements. Defined in the executable so they interpose on every
// shared library in the process (rclcpp, rmw, DDS, ...).
// ----------------------------------------------------------------------------
void * operator new(std::size_t n) { return alloc_or_throw(n); }
void * operator new[](std::size_t n) { return alloc_or_throw(n); }
void * operator new(std::size_t n, const std::nothrow_t &) noexcept { return raw_alloc(n); }
void * operator new[](std::size_t n, const std::nothrow_t &) noexcept { return raw_alloc(n); }
void * operator new(std::size_t n, std::align_val_t al) { return aligned_alloc_or_throw(n, al); }
void * operator new[](std::size_t n, std::align_val_t al) { return aligned_alloc_or_throw(n, al); }
void * operator new(std::size_t n, std::align_val_t al, const std::nothrow_t &) noexcept
{
    return raw_aligned_alloc(n, al);
}
void * operator new[](std::size_t n, std::align_val_t al, const std::nothrow_t &) noexcept
{
    return raw_aligned_alloc(n, al);
}

void operator delete(void * p) noexcept { std::free(p); }
void operator delete[](void * p) noexcept { std::free(p); }
void operator delete(void * p, std::size_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t) noexcept { std::free(p); }
void operator delete(void * p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void * p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete(void * p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void * p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void * p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete(void * p, std::align_val_t, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void * p, std::align_val_t, const std::nothrow_t &) noexcept { std::free(p); }
