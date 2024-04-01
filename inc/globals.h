#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define LOW 0
#define HIGH 1
// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif // min

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif // max

#ifndef bool
#define bool _Bool
#endif // boolo

#define abs(x) ((x) > 0 ? (x) : -(x))

#define mod(a, b) ((a % b + b) % b)

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x)-0.5))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

#define SIZEOF(arr) sizeof(arr) / sizeof(*arr)
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT8 (1 << 8)
#define BIT9 (1 << 9)

#define enable_interrupts() asm(" cpsie i ")
#define disable_interrupts() asm(" cpsid i ")
#endif