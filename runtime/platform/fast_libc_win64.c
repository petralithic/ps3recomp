/* fast_libc_win64.c -- memcpy, memmove, memset, memcmp and sqrt for the Windows build
 * (and fma for the older-CPU build: see the end).
 *
 * zig's toolchain supplies these from its own compiler runtime, built for a baseline
 * x86-64: memcmp is a byte-at-a-time loop, memset a scalar one, sqrt a software
 * square root. The renderer compares and clears constant blocks and vertex data on
 * every draw through them. The linker takes these definitions over zig's (weak) ones.
 *
 * AVX2 (FL_AVX2) is safe here: racacit_entry refuses CPUs without it before anything
 * runs, and the lifted code and runtime need it anyway. The older-CPU build defines no
 * FL_AVX2 and gets the SSE2 form. Built with -fno-builtin, so the compiler
 * cannot turn a loop below back into a call to itself. */
#if defined(_WIN32) && defined(__x86_64__) && !defined(_MSC_VER)
#include <immintrin.h>
#include <stddef.h>
#include <stdint.h>

/* fl_v: 32 bytes. FL_AVX2 (CMake, with PS3RECOMP_AVX2): one AVX2 register. Without it
 * (RaCACiT_compat.exe, CPUs without AVX2) two SSE2 registers: the same paths, the same
 * overlap rules, half as wide per instruction. */
#if defined(FL_AVX2)
#define FL_TARGET __attribute__((target("avx2,bmi")))
/* Not _mm256_zeroupper(): that intrinsic counts as clobbering every vector register, and
 * on Windows xmm6-15 are callee-saved, so each call saved ten of them on entry, even a
 * 16-byte memset. As plain assembly it costs what it should. */
#define FL_VZU() __asm__ volatile("vzeroupper")
typedef __m256i fl_v;
#define FL_LD(p)    _mm256_loadu_si256((const __m256i*)(p))
#define FL_ST(p, v) _mm256_storeu_si256((__m256i*)(p), (v))
FL_TARGET static inline fl_v fl_set1(char c) { return _mm256_set1_epi8(c); }
FL_TARGET static inline unsigned fl_eqmask(const void* a, const void* b)
{
    return (unsigned)_mm256_movemask_epi8(_mm256_cmpeq_epi8(FL_LD(a), FL_LD(b)));
}
#else
#define FL_TARGET
#define FL_VZU() ((void)0)
typedef struct { __m128i lo, hi; } fl_v;
static inline fl_v fl_ld(const void* p)
{
    fl_v v; v.lo = _mm_loadu_si128((const __m128i*)p); v.hi = _mm_loadu_si128((const __m128i*)((const char*)p + 16)); return v;
}
static inline void fl_st(void* p, fl_v v)
{
    _mm_storeu_si128((__m128i*)p, v.lo); _mm_storeu_si128((__m128i*)((char*)p + 16), v.hi);
}
#define FL_LD(p)    fl_ld(p)
#define FL_ST(p, v) fl_st((p), (v))
static inline fl_v fl_set1(char c) { fl_v v; v.lo = v.hi = _mm_set1_epi8(c); return v; }
static inline unsigned fl_eqmask(const void* a, const void* b)
{
    const fl_v x = fl_ld(a), y = fl_ld(b);
    return (unsigned)_mm_movemask_epi8(_mm_cmpeq_epi8(x.lo, y.lo)) |
           ((unsigned)_mm_movemask_epi8(_mm_cmpeq_epi8(x.hi, y.hi)) << 16);
}
#endif
typedef uint16_t __attribute__((aligned(1), may_alias)) fl_u16;
typedef uint32_t __attribute__((aligned(1), may_alias)) fl_u32;
typedef uint64_t __attribute__((aligned(1), may_alias)) fl_u64;

/* Overlap-safe: every path up to 128 bytes loads everything before it stores. */
FL_TARGET static inline void* fl_move(void* dv, const void* sv, size_t n)
{
    char* d = (char*)dv; const char* s = (const char*)sv;
    if (n <= 16) {
        if (n >= 8) { const uint64_t a = *(const fl_u64*)s, b = *(const fl_u64*)(s + n - 8); *(fl_u64*)d = a; *(fl_u64*)(d + n - 8) = b; return dv; }
        if (n >= 4) { const uint32_t a = *(const fl_u32*)s, b = *(const fl_u32*)(s + n - 4); *(fl_u32*)d = a; *(fl_u32*)(d + n - 4) = b; return dv; }
        if (n >= 2) { const uint16_t a = *(const fl_u16*)s, b = *(const fl_u16*)(s + n - 2); *(fl_u16*)d = a; *(fl_u16*)(d + n - 2) = b; return dv; }
        if (n) *d = *s;
        return dv;
    }
    if (n <= 32) {
        const __m128i a = _mm_loadu_si128((const __m128i*)s), b = _mm_loadu_si128((const __m128i*)(s + n - 16));
        _mm_storeu_si128((__m128i*)d, a); _mm_storeu_si128((__m128i*)(d + n - 16), b);
        return dv;
    }
    if (n <= 64) {
        const fl_v a = FL_LD(s), b = FL_LD((s + n - 32));
        FL_ST(d, a); FL_ST((d + n - 32), b);
        FL_VZU();
        return dv;
    }
    if (n <= 128) {
        const fl_v a = FL_LD(s), b = FL_LD((s + 32));
        const fl_v c = FL_LD((s + n - 64)), e = FL_LD((s + n - 32));
        FL_ST(d, a); FL_ST((d + 32), b);
        FL_ST((d + n - 64), c); FL_ST((d + n - 32), e);
        FL_VZU();
        return dv;
    }
    if ((uintptr_t)d - (uintptr_t)s >= n) {
        /* Forward: the destination is before the source, or clear of it. */
        const fl_v t1 = FL_LD((s + n - 64)), t0 = FL_LD((s + n - 32));
        size_t i = 0;
        const size_t end = n - 64;
        for (; i + 64 <= end; i += 64) {
            const fl_v a = FL_LD((s + i)), b = FL_LD((s + i + 32));
            FL_ST((d + i), a); FL_ST((d + i + 32), b);
        }
        if (i < end) {
            const fl_v a = FL_LD((s + i)), b = FL_LD((s + i + 32));
            FL_ST((d + i), a); FL_ST((d + i + 32), b);
        }
        FL_ST((d + n - 64), t1); FL_ST((d + n - 32), t0);
    } else {
        /* Backward: the destination overlaps the source from above. */
        const fl_v h0 = FL_LD(s), h1 = FL_LD((s + 32));
        size_t i = n;
        for (; i >= 64 + 64; i -= 64) {
            const fl_v a = FL_LD((s + i - 32)), b = FL_LD((s + i - 64));
            FL_ST((d + i - 32), a); FL_ST((d + i - 64), b);
        }
        if (i > 64) {
            const fl_v a = FL_LD((s + i - 32)), b = FL_LD((s + i - 64));
            FL_ST((d + i - 32), a); FL_ST((d + i - 64), b);
        }
        FL_ST(d, h0); FL_ST((d + 32), h1);
    }
    FL_VZU();
    return dv;
}

FL_TARGET void* memmove(void* d, const void* s, size_t n) { return fl_move(d, s, n); }
FL_TARGET void* memcpy(void* d, const void* s, size_t n)  { return fl_move(d, s, n); }

FL_TARGET void* memset(void* dv, int c, size_t n)
{
    char* d = (char*)dv;
    if (n <= 16) {
        const uint64_t v = 0x0101010101010101ULL * (uint8_t)c;
        if (n >= 8) { *(fl_u64*)d = v; *(fl_u64*)(d + n - 8) = v; return dv; }
        if (n >= 4) { *(fl_u32*)d = (uint32_t)v; *(fl_u32*)(d + n - 4) = (uint32_t)v; return dv; }
        if (n >= 2) { *(fl_u16*)d = (uint16_t)v; *(fl_u16*)(d + n - 2) = (uint16_t)v; return dv; }
        if (n) *d = (char)c;
        return dv;
    }
    if (n <= 32) {
        const __m128i v = _mm_set1_epi8((char)c);
        _mm_storeu_si128((__m128i*)d, v); _mm_storeu_si128((__m128i*)(d + n - 16), v);
        return dv;
    }
    const fl_v v = fl_set1((char)c);
    for (size_t i = 0; i + 32 < n; i += 32) FL_ST((d + i), v);
    FL_ST((d + n - 32), v);
    FL_VZU();
    return dv;
}

FL_TARGET int memcmp(const void* av, const void* bv, size_t n)
{
    const unsigned char* a = (const unsigned char*)av; const unsigned char* b = (const unsigned char*)bv;
    if (n >= 32) {
        do {
            const unsigned m = fl_eqmask(a, b);
            if (m != 0xFFFFFFFFu) {
                const unsigned k = (unsigned)__builtin_ctz(~m);
                const int r = (int)a[k] - (int)b[k];
                FL_VZU();
                return r;
            }
            a += 32; b += 32; n -= 32;
        } while (n >= 32);
        FL_VZU();
    }
    if (n >= 16) {
        const unsigned m = (unsigned)_mm_movemask_epi8(_mm_cmpeq_epi8(_mm_loadu_si128((const __m128i*)a), _mm_loadu_si128((const __m128i*)b)));
        if (m != 0xFFFFu) { const unsigned k = (unsigned)__builtin_ctz(~m); return (int)a[k] - (int)b[k]; }
        a += 16; b += 16; n -= 16;
    }
    if (n >= 8) {
        const uint64_t x = *(const fl_u64*)a ^ *(const fl_u64*)b;
        if (x) { const unsigned k = (unsigned)(__builtin_ctzll(x) >> 3); return (int)a[k] - (int)b[k]; }
        a += 8; b += 8; n -= 8;
    }
    for (size_t i = 0; i < n; i++) if (a[i] != b[i]) return (int)a[i] - (int)b[i];
    return 0;
}

FL_TARGET double sqrt(double x) { const __m128d v = _mm_set_sd(x); return _mm_cvtsd_f64(_mm_sqrt_sd(v, v)); }
FL_TARGET float sqrtf(float x)  { const __m128 v = _mm_set_ss(x); return _mm_cvtss_f32(_mm_sqrt_ss(v)); }

#if !defined(FL_AVX2)
/* fma for the older-CPU build (no FL_AVX2: RaCACiT_compat.exe). Without FMA hardware the
 * lifted code's fma() -- every PPC fmadd/fmsub/fnmadd/fnmsub -- is a call, and zig's
 * mingw fma() is wrong: NaN when z dwarfs x*y, garbage when x*y + z nearly cancels (8% of
 * random single-precision inputs, tested against vfmadd). This one is exact, checked
 * bit for bit against the hardware on 10^8+ cases.
 *   Fast path: x and y single-precision values (the game's fmadds, almost everything):
 *   their product has at most 48 significant bits and an exponent well inside double's
 *   range, so x*y is exact and one rounded add is the fused result.
 *   Otherwise: the product exactly in 128-bit integers, z aligned with a sticky bit,
 *   one round to nearest even (subnormals and overflow included). */
typedef unsigned __int128 fl_u128;

static inline uint64_t fl_dbits(double d) { union { double d; uint64_t u; } v; v.d = d; return v.u; }
static inline double fl_bitsd(uint64_t u) { union { double d; uint64_t u; } v; v.u = u; return v.d; }
static inline int fl_clz128(fl_u128 v)
{
    const uint64_t hi = (uint64_t)(v >> 64), lo = (uint64_t)v;
    return hi ? __builtin_clzll(hi) : 64 + __builtin_clzll(lo);
}

static double fl_fma_slow(double x, double y, double z)
{
    const uint64_t bx = fl_dbits(x), by = fl_dbits(y), bz = fl_dbits(z);
    const int ex = (int)(bx >> 52 & 0x7FF), ey = (int)(by >> 52 & 0x7FF), ez = (int)(bz >> 52 & 0x7FF);
    if (ex == 0x7FF || ey == 0x7FF) return x * y + z;            /* inf/NaN in the product */
    if (ez == 0x7FF) return z + z;                                 /* finite x*y + inf/NaN: z (NaN quieted) */
    if (x == 0.0 || y == 0.0) return x * y + z;                    /* exact signed zero + z */
    if (z == 0.0) return x * y;                                    /* one rounding; keeps an underflowed product's sign */

    const uint64_t fm = (1ull << 52) - 1;
    const uint64_t mx = ex ? (bx & fm) | (1ull << 52) : (bx & fm);
    const uint64_t my = ey ? (by & fm) | (1ull << 52) : (by & fm);
    const uint64_t mz = ez ? (bz & fm) | (1ull << 52) : (bz & fm);
    const int px = (ex ? ex : 1) - 1075, py = (ey ? ey : 1) - 1075, pz = (ez ? ez : 1) - 1075;
    const int sp = (int)((bx ^ by) >> 63), sz = (int)(bz >> 63);

    /* Both with their top bit at 124: value = M * 2^e. */
    fl_u128 A = (fl_u128)mx * my; int ea = px + py;
    int sh = fl_clz128(A) - 3; A <<= sh; ea -= sh;
    fl_u128 C = (fl_u128)mz;      int ec = pz;
    sh = fl_clz128(C) - 3; C <<= sh; ec -= sh;

    int sa = sp, sc = sz;
    if (ec > ea || (ec == ea && C > A)) {                          /* A: the larger magnitude */
        fl_u128 t = A; A = C; C = t; int te = ea; ea = ec; ec = te; int ts = sa; sa = sc; sc = ts;
    }
    const int d = ea - ec;
    if (d >= 128) C = 1;                                           /* only a sticky bit left */
    else if (d > 0) C = (C >> d) | (fl_u128)((C & (((fl_u128)1 << d) - 1)) != 0);
    fl_u128 S;
    if (sa == sc) S = A + C;
    else {
        S = A - C;
        if (S == 0) return 0.0;                                    /* exact cancellation: +0 */
    }
    const int m = 127 - fl_clz128(S);                              /* top bit of S */
    int L = ea + m - 52;                                           /* the result's last-bit exponent */
    if (L < -1074) L = -1074;
    const int s = L - ea;                                          /* bits to drop */
    uint64_t mant;
    if (s <= 0) mant = (uint64_t)(S << -s);
    else if (s > 126) mant = 0;                                    /* below half the smallest subnormal */
    else {
        mant = (uint64_t)(S >> s);
        const fl_u128 rem = S & (((fl_u128)1 << s) - 1), half = (fl_u128)1 << (s - 1);
        if (rem > half || (rem == half && (mant & 1))) mant++;
        if (mant == (1ull << 53)) { mant >>= 1; L++; }
    }
    uint64_t r;
    if (mant >= (1ull << 52)) {
        const int be = L + 52 + 1075 - 52;                         /* biased exponent = L + 1075 */
        if (be >= 0x7FF) r = 0x7FFull << 52;                       /* overflow: inf */
        else r = ((uint64_t)be << 52) | (mant & fm);
    } else r = mant;                                               /* subnormal (L == -1074) or zero */
    return fl_bitsd(r | ((uint64_t)sa << 63));
}

double fma(double x, double y, double z)
{
    if ((double)(float)x == x && (double)(float)y == y) return x * y + z;
    return fl_fma_slow(x, y, z);
}
#endif
#endif
