/* Diagnostics must not turn an unreadable guest stack into a host crash.
 * Read through the host kernel, which also handles a concurrent unmapping.
 * This is deliberately outside the guest-memory execution fast path. */
#pragma once
#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_vm.h>
#elif !defined(_WIN32)
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>
#endif
static bool racit_stack_read(uint32_t ea, uint32_t* word)
{
    if ((uint64_t)ea + sizeof(*word) > 0x100000000ULL) return false;
#if defined(__APPLE__)
    mach_vm_size_t copied = 0;
    return mach_vm_read_overwrite(mach_task_self(),
        (mach_vm_address_t)(vm_base + ea), sizeof(*word),
        (mach_vm_address_t)word, &copied) == KERN_SUCCESS && copied == sizeof(*word);
#elif defined(_WIN32)
    SIZE_T copied = 0;
    return ReadProcessMemory(GetCurrentProcess(), vm_base + ea, word,
        sizeof(*word), &copied) && copied == sizeof(*word);
#else
    struct iovec local = { word, sizeof(*word) }, remote = { vm_base + ea, sizeof(*word) };
    return syscall(SYS_process_vm_readv, getpid(), &local, 1, &remote, 1, 0) == sizeof(*word);
#endif
}
