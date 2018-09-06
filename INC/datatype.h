/**
 * @FileName：datatype.h
 * @Description：本头文件定义了C51常用的数据类型，使用时include "stdint.h"
 * @Author：王非
 * @Date：2018.08.12
 */

#ifndef __DATATYPE_H__
#define __DATATYPE_H__


/**
 * booll类型，逻辑类型 C51 常用定义 
 */
typedef unsigned char bool;
typedef unsigned char Bool;
typedef unsigned char _BOOL;

#define true            1
#define false           0

/**
 * 整型部分 C51 常用定义
 */
/* exact-width signed integer types */
typedef signed char     int8_t;
typedef signed int      int16_t;
typedef signed long     int32_t;

/* exact-width unsigned integer types */
typedef unsigned char   uint8_t;
typedef unsigned int    uint16_t;
typedef unsigned long    uint32_t;

/* common data type */
typedef int8_t          s8;
typedef int16_t         s16;
typedef int32_t         s32;

typedef uint8_t         u8;
typedef uint16_t        u16;
typedef uint32_t        u32;

typedef unsigned char   uchar;
typedef unsigned int    uint;

/* smallest type of at least n bits */
/* minimum-width signed integer types */
typedef signed char     int_least8_t;
typedef signed int      int_least16_t;
typedef signed long     int_least32_t;

/* minimum-width unsigned integer types */
typedef unsigned char   uint_least8_t;
typedef unsigned int    uint_least16_t;
typedef unsigned long   uint_least32_t;


/* fastest minimum-width signed integer types */
typedef signed long     int_fast8_t;
typedef signed long     int_fast16_t;
typedef signed long     int_fast32_t;

/* fastest minimum-width unsigned integer types */
typedef unsigned long   uint_fast8_t;
typedef unsigned long   uint_fast16_t;
typedef unsigned long   uint_fast32_t;

/*integer types capable of holding object pointers */
typedef signed long     intptr_t;
typedef unsigned long   uintptr_t;

/* minimum values of exact-width signed integer types */
#define INT8_MIN        -128
#define INT16_MIN       -32768
#define INT32_MIN       (~0x7fffffff) /* -2147483648 is unsigned */

/* maximum values of exact-width signed integer types */
#define INT8_MAX        127
#define INT16_MAX       32767
#define INT32_MAX       2147483647

/* maximum values of exact-width unsigned integer types */
#define UINT8_MAX       255
#define UINT16_MAX      65535
#define UINT32_MAX      4294967295u

/* 7.18.2.2 */

/* minimum values of minimum-width signed integer types */
#define INT_LEAST8_MIN  -128
#define INT_LEAST16_MIN -32768
#define INT_LEAST32_MIN (~0x7fffffff)

/* maximum values of minimum-width signed integer types */
#define INT_LEAST8_MAX  127
#define INT_LEAST16_MAX 32767
#define INT_LEAST32_MAX 2147483647

/* maximum values of minimum-width unsigned integer types */
#define UINT_LEAST8_MAX  255
#define UINT_LEAST16_MAX 65535
#define UINT_LEAST32_MAX 4294967295u

/* minimum values of fastest minimum-width signed integer types */
#define INT_FAST8_MIN   (~0x7fffffff)
#define INT_FAST16_MIN  (~0x7fffffff)
#define INT_FAST32_MIN  (~0x7fffffff)

/* maximum values of fastest minimum-width signed integer types */
#define INT_FAST8_MAX   2147483647
#define INT_FAST16_MAX  2147483647
#define INT_FAST32_MAX  2147483647

/* maximum values of fastest minimum-width unsigned integer types */
#define UINT_FAST8_MAX  4294967295u
#define UINT_FAST16_MAX 4294967295u
#define UINT_FAST32_MAX 4294967295u

/* minimum value of pointer-holding signed integer type */
#define INTPTR_MIN (~0x7fffffff)

/* maximum value of pointer-holding signed integer type */
#define INTPTR_MAX 2147483647

/* maximum value of pointer-holding unsigned integer type */
#define UINTPTR_MAX 4294967295u


/* limits of ptrdiff_t */
#define PTRDIFF_MIN (~0x7fffffff)
#define PTRDIFF_MAX 2147483647

/* limits of sig_atomic_t */
#define SIG_ATOMIC_MIN (~0x7fffffff)
#define SIG_ATOMIC_MAX 2147483647

/* limit of size_t */
#define SIZE_MAX 4294967295u



#endif
