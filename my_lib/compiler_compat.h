#ifndef __MY_COMPILER_COMPAT_H
#define __MY_COMPILER_COMPAT_H

/**
  ******************************************************************************
  * @file    compiler_compat.h
  * @brief   编译器兼容层
  *
  * 本工程最初用 Keil MDK（ARMCC / ARMCLANG）开发，部分语法是 ARM 编译器专有的。
  * 这个头文件把它们映射到 GCC 的等价写法，使同一份源码可以用
  * arm-none-eabi-gcc 编译（CI 用它做编译校验）。
  *
  * 目前只处理 __weak：
  *   - ARMCC/ARMCLANG：__weak 是关键字
  *   - GCC：没有这个关键字，用 __attribute__((weak)) 代替
  ******************************************************************************
  */

#if defined(__GNUC__)
  #ifndef __weak
    #define __weak __attribute__((weak))
  #endif
#endif

#endif /* __MY_COMPILER_COMPAT_H */
