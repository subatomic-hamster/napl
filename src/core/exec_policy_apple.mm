#include "core/exec_policy.h"
#import <Foundation/Foundation.h>
#include <dispatch/dispatch.h>
#include <mach/mach.h>
#include <sys/sysctl.h>
#include <atomic>

namespace execution {
namespace {
std::atomic<int> pressure{-1};
std::uint64_t available_memory() {
  vm_statistics64_data_t vm{};
  mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;
  const mach_port_t host = mach_host_self();
  vm_size_t page = 0;
  const bool ok = host_page_size(host, &page) == KERN_SUCCESS &&
      host_statistics64(host, HOST_VM_INFO64, reinterpret_cast<host_info64_t>(&vm), &count) == KERN_SUCCESS;
  mach_port_deallocate(mach_task_self(), host);
  return ok ? (static_cast<std::uint64_t>(vm.free_count) + vm.inactive_count) * page : 0;
}
void sample() {
  @autoreleasepool {
    NSProcessInfo* info = NSProcessInfo.processInfo;
    signals(static_cast<int>(info.thermalState), info.lowPowerModeEnabled,
            pressure.load(std::memory_order_relaxed), available_memory());
  }
}
}  // namespace
void start_apple_telemetry() {
  // A utility queue, not a Cocoa run loop; sources live for process lifetime.
  dispatch_queue_attr_t attr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_UTILITY, 0);
  dispatch_queue_t queue = dispatch_queue_create("napl.execution.telemetry", attr);
  dispatch_source_t memory = dispatch_source_create(DISPATCH_SOURCE_TYPE_MEMORYPRESSURE, 0,
      DISPATCH_MEMORYPRESSURE_NORMAL | DISPATCH_MEMORYPRESSURE_WARN | DISPATCH_MEMORYPRESSURE_CRITICAL, queue);
  if (memory) {
    dispatch_source_set_event_handler(memory, ^{
      const unsigned long flags = dispatch_source_get_data(memory);
      pressure.store(flags & DISPATCH_MEMORYPRESSURE_CRITICAL ? 2 : flags & DISPATCH_MEMORYPRESSURE_WARN ? 1 : 0,
                     std::memory_order_relaxed);
      sample();
    });
    dispatch_resume(memory);
  }
  // Query the kernel's current pressure level before the first transition event.
  int level = -1;
  std::size_t size = sizeof level;
  if (sysctlbyname("kern.memorystatus_vm_pressure_level", &level, &size, nullptr, 0) == 0)
    pressure.store(level >= 4 ? 2 : level >= 2 ? 1 : 0, std::memory_order_relaxed);
  sample();
  dispatch_source_t timer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, queue);
  dispatch_source_set_timer(timer, dispatch_time(DISPATCH_TIME_NOW, NSEC_PER_SEC), NSEC_PER_SEC, NSEC_PER_SEC / 10);
  dispatch_source_set_event_handler(timer, ^{ sample(); });
  dispatch_resume(timer);
}
}  // namespace execution
