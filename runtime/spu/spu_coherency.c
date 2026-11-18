/*
 * ps3recomp - PPU <-> SPU lock-line coherence
 *
 * See spu_coherency.h for what this is for. The state here is deliberately
 * plain: a bit per 128-byte line, a small table of the SPU contexts that have
 * ever reserved one, and the lock-line spinlock. Everything that WRITES it runs
 * with the lock-line lock held (the SPU's GETLLAR, the PPU's coherent store),
 * so the read-modify-write on a shared bitmap byte is serialized. Only
 * spu_coh_is_reserved reads without the lock, which is the entire point: it is
 * on every PPU store. The one other unlocked path is a GETLLAR that re-reserves
 * a marked line (spu_coh_reserve_fast); its ordering argument is there.
 */

#include <stdlib.h>
#include "spu_coherency.h"
#include "spu_coh_bitmap.h"

#include <stdint.h>
#include <stdio.h>
#include "ps3emu/log_first.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * The lock-line lock
 *
 * Moved here from spu_channels.c, where it was a file-static and so could
 * serialize SPU against SPU but not SPU against PPU -- which left the PPU free
 * to write a line in the middle of a PUTLLC's compare-and-commit. Same
 * exclusion semantics, now shared. MSVC uses a bounded polite spin loop;
 * POSIX spins briefly and then parks on a mutex (see below).
 * ===========================================================================*/
#if defined(_WIN32)
#include <windows.h>   /* SwitchToThread, WaitOnAddress */
static volatile long g_lockline = 0;
static volatile long g_lockline_waiters = 0;
typedef BOOL (WINAPI* woa_fn)(volatile VOID*, PVOID, SIZE_T, DWORD);
typedef VOID (WINAPI* wba_fn)(PVOID);
static woa_fn s_woa; static wba_fn s_wba; static volatile int s_ll_init;
static int s_ll_spin = 4096;
/* Test-and-test-and-set: spin on a plain read (no cache-line ping-pong) with a
 * pause, yield after a while, and then really wait. The bare exchange loop let
 * GH3's four idle job-policy pollers saturate the line and delay every other SPU
 * atomic -- FMOD's mixer task among them, 25-40 ms at a time.
 *
 * This was under _MSC_VER until v99, and the Windows build is zig's clang: it got
 * the bare exchange loop after all. With more threads than CPUs a holder that loses
 * its CPU kept all the others hammering the line until it got one back (Windows'
 * time slice: 15+ ms): 48 fps on 6 threads and 1-5 on 4, game audio at 6-14 blocks a
 * second of its 187.5, with every CPU flat out. Waiting frees the CPU the holder needs. */
void spu_lockline_lock(void)
{
    if (!_InterlockedExchange(&g_lockline, 1)) return;
    if (!s_ll_init) {
        extern int racacit_few_cpus(void);
        HMODULE m = LoadLibraryA("api-ms-win-core-synch-l1-2-0.dll");
        if (m) { s_woa = (woa_fn)(void*)GetProcAddress(m, "WaitOnAddress"); s_wba = (wba_fn)(void*)GetProcAddress(m, "WakeByAddressSingle"); }
        if (racacit_few_cpus()) s_ll_spin = 128;
        s_ll_init = 1;
    }
    unsigned spins = 0, rounds = 0;
    do {
        while (g_lockline) {
            YieldProcessor();
            if (++spins < (unsigned)s_ll_spin) continue;
            spins = 0;
            if (++rounds < 3 || !s_woa) { SwitchToThread(); continue; }
            long held = 1;
            _InterlockedIncrement(&g_lockline_waiters);
            if (g_lockline) s_woa(&g_lockline, &held, sizeof held, 1);
            _InterlockedDecrement(&g_lockline_waiters);
        }
    } while (_InterlockedExchange(&g_lockline, 1));
}
void spu_lockline_unlock(void)
{
    _InterlockedExchange(&g_lockline, 0);
    if (g_lockline_waiters && s_wba) s_wba((PVOID)&g_lockline);
}
/* The short locks' wait: on a plain read, with a pause; a holder off its CPU gets a yield. */
void spu_spin_wait(volatile long* lock)
{
    unsigned spins = 0;
    while (*lock) {
        YieldProcessor();
        if (++spins >= 1024u) { spins = 0; SwitchToThread(); }
    }
}
#else
#include <stdatomic.h>
#include <sched.h>
#include <pthread.h>
void racacit_yield(void);   /* win32_compat.c: sched_yield without the macOS priority depression */
static atomic_int g_lockline;
#if defined(__aarch64__) || defined(__arm64__)
#define SPU_LL_PAUSE() __asm__ __volatile__("yield")
#elif defined(__x86_64__) || defined(__i386__)
#define SPU_LL_PAUSE() __builtin_ia32_pause()
#else
#define SPU_LL_PAUSE() ((void)0)
#endif
/* Test-and-test-and-set, as the Windows path above: spin on a plain read
 * with a pause and only then exchange, and give the CPU up after a while.
 * The bare exchange loop this replaces kept every waiting lane writing the
 * line, and with more busy threads than cores (main, RSX, backend, six SPU
 * lanes) a holder that lost its CPU left the others hammering until it got
 * one back. SPU_LL_SPIN=<n>: reads before yielding (default 2048). */
static int s_ll_spin = -1;
/* SPU_LL_STATS: when the lock was taken (hold times are measured at unlock). */
static int s_ll_st = -1;
static unsigned long spu_ll_tid(void)
{
#ifdef __APPLE__
    uint64_t t = 0; pthread_threadid_np(NULL, &t); return (unsigned long)t;   /* = GetCurrentThreadId, as in the [hosttid] lines */
#else
    return 0;
#endif
}
static unsigned long long s_ll_acq;
static inline void spu_ll_note_acq(void)
{
    if (s_ll_st < 0) s_ll_st = getenv("SPU_LL_STATS") ? 1 : 0;
    if (s_ll_st) { extern unsigned long long ps3_qpc_us(void); s_ll_acq = ps3_qpc_us(); }
}
void spu_lockline_lock(void)
{
    if (!atomic_exchange_explicit(&g_lockline, 1, memory_order_acquire)) { spu_ll_note_acq(); return; }
    if (s_ll_spin < 0) { const char* e = getenv("SPU_LL_SPIN"); s_ll_spin = e ? atoi(e) : 2048; if (s_ll_spin < 1) s_ll_spin = 1; }
    /* SPU_LL_STATS=1: contended acquisitions every 5 s -- how many, how many
     * waited >= 1 ms, and the longest (a holder off its CPU stalls everyone). */
    static int s_st = -1; if (s_st < 0) s_st = getenv("SPU_LL_STATS") ? 1 : 0;
    extern unsigned long long ps3_qpc_us(void);
    const unsigned long long t0 = s_st ? ps3_qpc_us() : 0;
    unsigned spins = 0;
    do {
        while (atomic_load_explicit(&g_lockline, memory_order_relaxed)) {
            SPU_LL_PAUSE();
            if (++spins >= (unsigned)s_ll_spin) { spins = 0; racacit_yield(); }
        }
    } while (atomic_exchange_explicit(&g_lockline, 1, memory_order_acquire));
    if (s_st) {
        static unsigned long long n, slow, mx, last; const unsigned long long t = ps3_qpc_us(), d = t - t0;
        n++; if (d >= 1000) { slow++; extern void ps3_tl_at(unsigned long long, char, uint32_t, uint32_t); ps3_tl_at(t0, 'K', 0x11u, (uint32_t)d); } if (d > mx) mx = d;
        if (!last) last = t;
        if (t - last > 5000000ull) { fprintf(stderr, "[ll-wait] 5 s: %llu contended, %llu >= 1 ms, longest %llu us%c", n, slow, mx, 10); n = slow = mx = 0; last = t; }
    }
    spu_ll_note_acq();
}
void spu_lockline_unlock(void)
{
    if (s_ll_st > 0) {
        extern unsigned long long ps3_qpc_us(void);
        const unsigned long long d = ps3_qpc_us() - s_ll_acq;
        if (d >= 1000) {
            static int told; char nm[64] = "?";
            pthread_getname_np(pthread_self(), nm, sizeof nm);
            if (told++ < 200) fprintf(stderr, "[ll-hold] %llu us by thread \"%s\" tid %lu%c", d, nm, spu_ll_tid(), 10);
        }
    }
    atomic_store_explicit(&g_lockline, 0, memory_order_release);
}
void spu_spin_wait(volatile long* lock)
{
    unsigned spins = 0;
    while (*lock) {
        SPU_LL_PAUSE();
        if (++spins >= 1024u) { spins = 0; racacit_yield(); }
    }
}
#endif

/* ===========================================================================
 * The reserved-line bitmap
 *
 * One bit per 128-byte line over the whole 32-bit guest address space: 4 MiB of
 * zero-initialized BSS, of which only the pages around addresses an SPU has
 * actually reserved are ever touched. The reference implementation scopes its
 * bitmap to the one window a single title's SPURS structures live in; this
 * runtime hands out guest memory from several disjoint regions (the ELF image,
 * the sys_memory window at 0x40000000, the raw-SPU local-store windows), and a
 * line outside a hardcoded window would be silently non-coherent, which is the
 * failure this whole file exists to remove.
 * ===========================================================================*/
#define SPU_COH_LINES       (1u << (32 - SPU_COH_LINE_SHIFT))   /* 2^25 lines */
#define SPU_COH_BITMAP_SZ   (SPU_COH_LINES / 8)                 /* 4 MiB */

unsigned char spu_coh_bitmap_g[SPU_COH_BITMAP_SZ];   /* read inline by spu_coh_is_reserved (header) */
#define s_coh_bitmap spu_coh_bitmap_g

/* g_spu_coh_bits (spu_coh_bitmap.h) stays NULL until the first GETLLAR
 * anywhere. That keeps the PPU fast path off the bitmap entirely for a title
 * that runs no SPU code, and keeps every existing test that stores through
 * vm_write* paying one compare against a hot global. Written under the lock;
 * read unlocked, where a stale NULL costs at most one missed event on the very
 * first reservation of the run. racacit: spu_coh_armed_g mirrors it for the
 * RACiT inline readers (spu_coh_is_reserved_fast, ppu_loader.cpp). */
volatile int spu_coh_armed_g;
#define s_coh_armed spu_coh_armed_g

/* Every live virtual context may retain a reservation between scheduling
 * quanta; physical SPU count is not a bound on this set. All access is under
 * the lock-line lock. Entries are removed before their context's lifetime ends.
 *
 * A contiguous array, not a linked list: every notifier walks all of it with
 * the line held, and a list's scattered nodes were a cache miss per holder on
 * top of the one on each holder's resv_valid, which the walk now prefetches a
 * few entries ahead. */
static spu_context** s_coh_ctxs;
static unsigned s_coh_n, s_coh_cap;
#define SPU_COH_PREFETCH 4
static inline void spu_coh_prefetch_holder(unsigned i)
{
    if (i < s_coh_n) __builtin_prefetch(&s_coh_ctxs[i]->resv_valid);
}
static spu_coh_watch* s_coh_watches;

unsigned long g_spu_lr_raise = 0;

/* For the [mem] log line (game/main.cpp): lines marked in the bitmap now, and
 * writes spu_coh_notify_write was told of. A line stays marked once any SPU
 * reserved it, and every PPU store into a marked line takes the locked path:
 * if this count keeps climbing through a long session, so does that cost. */
unsigned long g_spu_coh_lines = 0;
unsigned long g_spu_coh_writes = 0;
unsigned long g_ppu_coh_stores = 0;   /* of those, PPU stores (ppu_loader.cpp; the rest are SPU atomics) */

void spu_coh_reserve(spu_context* ctx, uint32_t ea)
{
    /* Read before writing. The kernel reserves the same few lines over and
     * over, and a store of the same value still takes the cache line away
     * from every other core: each PPU store then missed on the pointer and on
     * the bitmap byte. */
    uint32_t line = ea >> SPU_COH_LINE_SHIFT;
    const unsigned char bit = (unsigned char)(1u << (line & 7));
    if (!(s_coh_bitmap[line >> 3] & bit)) {
        s_coh_bitmap[line >> 3] |= bit;
        g_spu_coh_lines++;
    }
    if (!g_spu_coh_bits)
        g_spu_coh_bits = s_coh_bitmap;
    if (!s_coh_armed)
        s_coh_armed = 1;

    if (!ctx) return;
    for (unsigned i = 0; i < s_coh_n; i++)
        if (s_coh_ctxs[i] == ctx) { __atomic_store_n(&ctx->coh_listed, 1, __ATOMIC_RELAXED); return; }
    if (s_coh_n == s_coh_cap) {
        unsigned cap = s_coh_cap ? s_coh_cap * 2u : 64u;
        spu_context** grown = (spu_context**)realloc(s_coh_ctxs, cap * sizeof(*grown));
        if (!grown) {
            fprintf(stderr, "[spu-coh] cannot register reservation holder %p\n", (void*)ctx);
            fflush(stderr);
            abort(); /* A missed holder loses invalidation events, including ABA. */
        }
        s_coh_ctxs = grown;
        s_coh_cap = cap;
    }
    s_coh_ctxs[s_coh_n++] = ctx;
    __atomic_store_n(&ctx->coh_listed, 1, __ATOMIC_RELAXED);
}

void spu_coh_watch_add(spu_coh_watch* watch, uint32_t ea,
                       void (*wake)(void*), void* opaque)
{
    /* Arming the existing bitmap makes PPU stores/CAS and MFC writes use
     * the same serialized commit/notify path as SPU reservations. */
    spu_coh_reserve(NULL, ea);
    watch->line = ea & ~127u;
    watch->wake = wake;
    watch->opaque = opaque;
    watch->next = s_coh_watches;
    s_coh_watches = watch;
}

void spu_coh_watch_remove(spu_coh_watch* watch)
{
    for (spu_coh_watch** link = &s_coh_watches; *link; link = &(*link)->next) {
        if (*link != watch) continue;
        *link = watch->next;
        watch->next = NULL;
        return;
    }
}

static void spu_coh_wake_watches(uint32_t line)
{
    for (spu_coh_watch* w = s_coh_watches; w; w = w->next)
        if (w->line == line) w->wake(w->opaque);
}

/* Drop a context from the reserving set.
 *
 * There was no way to do this, and the contexts that most need it are
 * TRANSIENT: spu_run_lifted_job_abi runs each job on a STACK-LOCAL
 * spu_context, which registered itself here on its first reservation and then
 * returned. The entry outlived the object, so spu_coh_notify_write walked a
 * dangling pointer -- reading resv_valid, writing event_status and calling
 * spu_ch_wake on reclaimed stack. Undefined behaviour, and observable: Guitar
 * Hero III showed THREE live-looking contexts for one dispatch, stealing each
 * other`s reservations, with every PUTLLC failing "no reservation". */
void spu_coh_unregister(spu_context* ctx)
{
    if (!ctx) return;
    /* Under the lock-line lock, which every walker of s_coh_ctxs holds: a
     * notifier that loaded this pointer just before removal would
     * otherwise go on dereferencing it after the job returned and its stack
     * (or its whole async thread) was gone. GH3 at -O2 crashed that way in
     * spu_coh_notify_write, reading a freed thread stack. */
    spu_lockline_lock();
    for (unsigned i = 0; i < s_coh_n; i++) {
        if (s_coh_ctxs[i] != ctx) continue;
        s_coh_ctxs[i] = s_coh_ctxs[--s_coh_n];   /* order does not matter */
        break;
    }
    __atomic_store_n(&ctx->coh_listed, 0, __ATOMIC_RELAXED);
    spu_lockline_unlock();
}

/* Drop every registered context that lies in [lo, hi) -- a host thread's
 * stack, called as that thread exits. A stack-local context still registered
 * then has leaked past its unregister; the next walker would read an unmapped
 * stack (GH3 at the end of a Havok task: spu_coh_notify_write faulting).
 * ponytail: safety net that also names the leak; find the path that skips
 * spu_coh_unregister if it keeps firing. */
void spu_coh_forget_range(uintptr_t lo, uintptr_t hi)
{
    spu_lockline_lock();
    for (unsigned i = 0; i < s_coh_n;) {
        uintptr_t c = (uintptr_t)s_coh_ctxs[i];
        if (c >= lo && c < hi) {
            static int _n = 0;
            if (PS3_LOG_FIRST(_n, 16))
                fprintf(stderr, "[spu-coh] leaked ctx %p (img=%d) dropped at thread exit\n",
                        (void*)c, s_coh_ctxs[i]->image_id);
            s_coh_ctxs[i] = s_coh_ctxs[--s_coh_n];   /* re-check slot i */
        } else {
            i++;
        }
    }
    spu_lockline_unlock();
}

/* racacit: hand a line a title marked with spu_coh_reserve(NULL, ...) back to
 * the PPU's lock-free store path (game/stubs.cpp, the collision records) --
 * unless an SPU holds a reservation on it right now; its next GETLLAR marks it
 * again anyway. Lock-line lock held. */
void spu_coh_unreserve_line(uint32_t ea)
{
    uint32_t line = ea & ~127u;
    for (unsigned i = 0; i < s_coh_n; i++) {
        spu_context* c = s_coh_ctxs[i];
        if (c && c->resv_valid && (c->resv_ea & ~127u) == line) return;
    }
    line >>= SPU_COH_LINE_SHIFT;
    if (s_coh_bitmap[line >> 3] & (1u << (line & 7))) g_spu_coh_lines--;
    s_coh_bitmap[line >> 3] &= (unsigned char)~(1u << (line & 7));
}

/* Hot paths use the inline spu_coh_is_reserved_fast (spu_coherency.h). */
int spu_coh_is_reserved(uint32_t addr) { return spu_coh_is_reserved_fast(addr); }

/* A DMA PUT is issued BY an SPU, and hardware does not take that SPU's own
 * reservation away for its own MFC write -- the MFC is part of the same SPE.
 * Our notify had no way to say "everyone but me", so an SPU that reserved a
 * line and then DMA'd a buffer overlapping it killed its own reservation and
 * its next PUTLLC could never succeed. Guitar Hero III's job policy module
 * livelocks exactly there: 16 million atomic ops on one line, every PUTLLC
 * failing for "no reservation", 600k of them self-inflicted.
 *
 * The PUTLLC path already guards this case by dropping its own reservation
 * before notifying; this gives the DMA path the same ability. */
/* ppu_loader.cpp, when linked. The loader is compiled per game, so the host,
 * the fixtures and any build without it have no PPU reservations to break.
 * POSIX: a weak no-op definition the loader's overrides (a weak undefined
 * reference is not enough: ld64 only lets those stay unresolved for dylib
 * symbols). link.exe/lld-link: the linker's own default (see ppu_gcm_pump in
 * ppu_loader.cpp). */
#if defined(_MSC_VER)
void ppu_resv_break_line(uint32_t ea);
void ppu_resv_break_line_absent(uint32_t ea) { (void)ea; }
#pragma comment(linker, "/alternatename:ppu_resv_break_line=ppu_resv_break_line_absent")
#else
__attribute__((weak)) void ppu_resv_break_line(uint32_t ea) { (void)ea; }
#endif

/* ===========================================================================
 * GETLLAR without the lock-line lock
 *
 * Every SPURS policy module polls its workload lines with GETLLAR, so taking
 * the one global lock for each poll serialized every SPU on a 128-byte copy.
 * A poll is almost always a context that is already in s_coh_ctxs
 * re-reading a line that is already marked, and for that case the lock buys
 * nothing the reservation itself does not:
 *
 *   reader (spu_coh_reserve_fast + caller)    writer (any notifier caller)
 *     resv_valid = 0, resv_ea = ea,             store the new line data
 *     resv_valid = 1                            spu_coh_writer_fence()
 *     seq_cst fence                             load resv_valid / resv_ea
 *     copy the line                             if they cover the line:
 *                                                 LR, resv_valid = 0
 *
 * This is the store-buffering (Dekker) shape: with a seq_cst fence between the
 * store and the load on both sides, the two fences are ordered, so at least
 * one side sees the other. Either the reader's copy already holds the write
 * (an ordinary GETLLAR after the store), or the writer sees the reservation
 * and ends it -- the reader may then hold a stale or torn copy, but its
 * PUTLLC fails and it has SPU_EVENT_LR, exactly as for a write landing just
 * after a locked GETLLAR. A writer that loads stale resv_valid/resv_ea is one
 * whose fence came first, so the copy already holds its data; at worst it
 * ends the new reservation spuriously. What the reader gives up is a copy
 * atomic against a writer's multi-store critical section, but such a copy is
 * always followed by that writer's notify, so the reservation is ended.
 *
 * Two preconditions keep every writer of the line on that right-hand side:
 *  - the line is already marked. Marks are never cleared, so every store path
 *    that checks spu_coh_is_reserved takes the notifier for this line.
 *  - ctx is already in s_coh_ctxs (coh_listed). The entry was added under the
 *    lock by an earlier GETLLAR of this same thread, so any writer whose
 *    critical section could overlap this read walks past it.
 * Anything else takes the lock and spu_coh_reserve as before.
 *
 * Writers still hold the lock-line lock, so they stay serialized with each
 * other and with PUTLLC's compare-and-commit; only the reader stopped
 * waiting. resv_valid and resv_ea are accessed atomically on both sides of
 * the race; everything else about a reservation is still only touched by the
 * owning thread or under the lock.
 * ===========================================================================*/
static inline void spu_coh_writer_fence(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

static inline int spu_coh_holds(const spu_context* c, uint32_t line)
{
    return __atomic_load_n(&c->resv_valid, __ATOMIC_RELAXED) &&
           (__atomic_load_n(&c->resv_ea, __ATOMIC_RELAXED) & ~127u) == line;
}

int spu_coh_reserve_fast(spu_context* ctx, uint32_t ea)
{
    if (!ctx || !__atomic_load_n(&ctx->coh_listed, __ATOMIC_RELAXED) ||
        !spu_coh_is_reserved(ea))
        return 0;
    /* Retire any older reservation before retargeting it. The fence is what
     * the argument above rests on. */
    __atomic_store_n(&ctx->resv_valid, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&ctx->resv_ea, ea, __ATOMIC_RELAXED);
    __atomic_store_n(&ctx->resv_valid, 1, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 1;
}

void spu_coh_notify_write_except(uint32_t ea, const void* self)
{
    uint32_t line = ea & ~127u;
    ppu_resv_break_line(ea);
    spu_coh_writer_fence();
    spu_coh_wake_watches(line);
    for (unsigned i = 0; i < SPU_COH_PREFETCH; i++) spu_coh_prefetch_holder(i);
    for (unsigned i = 0; i < s_coh_n; i++) {
        spu_coh_prefetch_holder(i + SPU_COH_PREFETCH);
        spu_context* c = s_coh_ctxs[i];
        if (!c || (const void*)c == self) continue;
        if (spu_coh_holds(c, line)) {
            SPU_EVENT_SET(c, SPU_EVENT_LR);
            __atomic_store_n(&c->resv_valid, 0, __ATOMIC_RELAXED);
            g_spu_lr_raise++;
            spu_ch_wake(c);
        }
    }
}

/* racacit: title hooks on SPU DMA of one guest structure (game/stubs.cpp, the
 * A Crack in Time collision command record). NULL for every other title.
 *   g_coh_write_hook     -- every write spu_coh_notify_write is told of: a PPU
 *                           store / stwcx. / vector store, or an SPU atomic, into
 *                           a reserved line (DMA PUTs use the _except variant
 *                           and are not reported). Lock-line lock held.
 *   g_spu_get_copy_hook  -- a DMA GET; returns 1 if it made the copy itself.
 *   g_spu_put_skip_hook  -- a DMA PUT into a reserved line, lock held; returns 1
 *                           to drop the copy (the transfer still completes). */
void (*g_coh_write_hook)(uint32_t ea) = 0;
int  (*g_spu_get_copy_hook)(void* spu, const uint8_t* ls, uint32_t lsa, uint32_t ea, uint32_t size,
                            uint8_t* dst, const uint8_t* src) = 0;
int  (*g_spu_put_skip_hook)(void* spu, const uint8_t* ls, uint32_t lsa, uint32_t ea, uint32_t size) = 0;

void spu_coh_notify_write(uint32_t ea)
{
    g_spu_coh_writes++;
    if (g_coh_write_hook) g_coh_write_hook(ea);
    uint32_t line = ea & ~127u;
    ppu_resv_break_line(ea);
    spu_coh_writer_fence();
    spu_coh_wake_watches(line);
    for (unsigned i = 0; i < SPU_COH_PREFETCH; i++) spu_coh_prefetch_holder(i);
    for (unsigned i = 0; i < s_coh_n; i++) {
        spu_coh_prefetch_holder(i + SPU_COH_PREFETCH);
        spu_context* c = s_coh_ctxs[i];
        if (!c) continue;
        if (spu_coh_holds(c, line)) {
            SPU_EVENT_SET(c, SPU_EVENT_LR);
            __atomic_store_n(&c->resv_valid, 0, __ATOMIC_RELAXED);   /* reservation lost, PUTLLC must fail */
            /* SPU_PUTLLC_WHY=1: name the agent that killed it. A PUTLLC that
             * always fails for "no reservation" is useless without knowing who
             * took it away -- a peer SPU, or the PPU committing to the line. */
            { static int s_w = -1;
              if (s_w < 0) s_w = getenv("SPU_PUTLLC_WHY") ? 1 : 0;
              if (s_w) { static unsigned long long n;
                  if ((++n % 200000) == 1)
                      fprintf(stderr, "[resv-lost] %llu: line 0x%08X taken from img=%d ctx=%p\n",
                              n, line, c->image_id, (void*)c); } }
            g_spu_lr_raise++;
            /* The event_status store has to be visible before the wake. The
             * waiter re-polls its predicate on a timeout as well, so a
             * straggling store costs latency and not the wakeup itself. */
            spu_ch_wake(c);
        }
    }
}

/* racacit: wake every parked SPU now. A Crack in Time posts its SPU jobs
 * with plain stores (no lock-line notify reaches the idle SPURS kernels),
 * then the main thread waits on them -- about nine submit/wait round trips
 * per view per frame (func_004DFDF0). An idle lane only noticed the new work
 * when its spu_lr_wait bound ran out (SPU_LR_WAIT_US), so every round trip
 * paid that bound. Called at the entry of the game's job/batch waits
 * (patch_lift step "spu kick"): each parked lane gets a reservation-lost
 * event, re-reads its workload state at once and finds the job. A spurious
 * LR event is harmless -- the kernel re-polls and parks again.
 * MEASURED 2026-10-18 (Galacton, standing still, 2x2 runs, order swapped):
 * no effect -- frame-waits stayed ~135 ms/s and frame times moved only with
 * run order. The waits are the jobs' own run time (igMobyPreRender ~1.4 ms a
 * frame), not lane pickup. So it is OFF by default; SPU_KICK_ON_WAIT=1 turns
 * it on. */
unsigned long g_spu_kicks = 0;
void racacit_spu_kick(void)
{
    static int s_on = -1;
    if (s_on < 0) { const char* e = getenv("SPU_KICK_ON_WAIT"); s_on = e ? atoi(e) != 0 : 0; }
    if (!s_on) return;
    spu_lockline_lock();
    for (unsigned i = 0; i < s_coh_n; i++) {
        spu_context* c = s_coh_ctxs[i];
        if (!c || !c->resv_valid) continue;
        SPU_EVENT_SET(c, SPU_EVENT_LR);
        __atomic_store_n(&c->resv_valid, 0, __ATOMIC_RELAXED);
        g_spu_lr_raise++;
        spu_ch_wake(c);
    }
    spu_lockline_unlock();
    g_spu_kicks++;
}

#ifdef __cplusplus
}
#endif
