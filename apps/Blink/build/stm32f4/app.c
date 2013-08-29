#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 147 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/include/stddef.h" 3
typedef int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef unsigned int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 41 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/stdint.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;




typedef signed char int_least8_t;
typedef unsigned char uint_least8_t;




typedef signed short int16_t;
typedef unsigned short uint16_t;
#line 67
typedef int16_t int_least16_t;
typedef uint16_t uint_least16_t;










typedef signed long int32_t;
typedef unsigned long uint32_t;
#line 97
typedef int32_t int_least32_t;
typedef uint32_t uint_least32_t;
#line 119
typedef signed long long int64_t;
typedef unsigned long long uint64_t;








typedef int64_t int_least64_t;
typedef uint64_t uint_least64_t;
#line 159
typedef signed int int_fast8_t;
typedef unsigned int uint_fast8_t;




typedef signed int int_fast16_t;
typedef unsigned int uint_fast16_t;




typedef signed int int_fast32_t;
typedef unsigned int uint_fast32_t;
#line 213
typedef int_least64_t int_fast64_t;
typedef uint_least64_t uint_fast64_t;







typedef long long int intmax_t;








typedef long long unsigned int uintmax_t;
#line 243
typedef signed int intptr_t;
typedef unsigned int uintptr_t;
# 283 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/inttypes.h" 3
#line 280
typedef struct __nesc_unnamed4242 {
  intmax_t quot;
  intmax_t rem;
} imaxdiv_t;
# 431 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 25 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/machine/_default_types.h" 3
typedef signed char __int8_t;
typedef unsigned char __uint8_t;








typedef signed short __int16_t;
typedef unsigned short __uint16_t;








typedef __int16_t __int_least16_t;
typedef __uint16_t __uint_least16_t;










typedef signed int __int32_t;
typedef unsigned int __uint32_t;
#line 75
typedef __int32_t __int_least32_t;
typedef __uint32_t __uint_least32_t;
#line 98
typedef signed long long __int64_t;
typedef unsigned long long __uint64_t;
# 6 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/sys/lock.h" 3
typedef int _LOCK_T;
typedef int _LOCK_RECURSIVE_T;
# 16 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/sys/_types.h" 3
typedef long _off_t;







typedef short __dev_t;




typedef unsigned short __uid_t;


typedef unsigned short __gid_t;



__extension__ 
#line 36
typedef long long _off64_t;







typedef long _fpos_t;
#line 56
typedef int _ssize_t;
# 353 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/include/stddef.h" 3
typedef unsigned int wint_t;
# 75 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/sys/_types.h" 3
#line 67
typedef struct __nesc_unnamed4243 {

  int __count;
  union __nesc_unnamed4244 {

    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;



typedef _LOCK_RECURSIVE_T _flock_t;




typedef void *_iconv_t;
# 21 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 37
struct _reent;






struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _on_exit_args {
  void *_fnargs[32];
  void *_dso_handle[32];

  __ULong _fntypes;


  __ULong _is_cxa;
};









struct _atexit {
  struct _atexit *_next;
  int _ind;

  void (*_fns[32])(void );
  struct _on_exit_args _on_exit_args;
};









struct __sbuf {
  unsigned char *_base;
  int _size;
};
#line 169
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  void *_cookie;

  int (*_read)(struct _reent *arg_0x2b430c8a0540, void *arg_0x2b430c8a07e0, char *arg_0x2b430c8a0a80, int arg_0x2b430c8a0ce8);

  int (*_write)(struct _reent *arg_0x2b430c8a7418, void *arg_0x2b430c8a76b8, const char *arg_0x2b430c8a7990, int arg_0x2b430c8a7bf8);

  _fpos_t (*_seek)(struct _reent *arg_0x2b430c8a6378, void *arg_0x2b430c8a6618, _fpos_t arg_0x2b430c8a68c8, int arg_0x2b430c8a6b30);
  int (*_close)(struct _reent *arg_0x2b430c8a4290, void *arg_0x2b430c8a4530);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  _off_t _offset;


  struct _reent *_data;



  _flock_t _lock;

  _mbstate_t _mbstate;
  int _flags2;
};
#line 273
typedef struct __sFILE __FILE;



struct _glue {

  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
#line 305
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};
#line 580
struct _reent {

  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2b430c8ac5a8);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4245 {

    struct __nesc_unnamed4246 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
      _mbstate_t _mblen_state;
      _mbstate_t _mbtowc_state;
      _mbstate_t _wctomb_state;
      char _l64a_buf[8];
      char _signal_buf[24];
      int _getdate_err;
      _mbstate_t _mbrlen_state;
      _mbstate_t _mbrtowc_state;
      _mbstate_t _mbsrtowcs_state;
      _mbstate_t _wcrtomb_state;
      _mbstate_t _wcsrtombs_state;
      int _h_errno;
    } _reent;



    struct __nesc_unnamed4247 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2b430c8b4b30);




  struct _glue __sglue;
  __FILE __sf[3];
};
#line 818
struct _reent;
struct _reent;
# 25 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/string.h" 3
void *memset(void *arg_0x2b430c8cc868, int arg_0x2b430c8ccad0, size_t arg_0x2b430c8ccd78);
# 34 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/stdlib.h" 3
#line 30
typedef struct __nesc_unnamed4248 {

  int quot;
  int rem;
} div_t;





#line 36
typedef struct __nesc_unnamed4249 {

  long quot;
  long rem;
} ldiv_t;






#line 43
typedef struct __nesc_unnamed4250 {

  long long int quot;
  long long int rem;
} lldiv_t;
# 14 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/math.h" 3
union __dmath {

  double d;
  __ULong i[2];
};

union __fmath {

  float f;
  __ULong i[1];
};


union __ldmath {

  long double ld;
  __ULong i[4];
};
#line 154
typedef float float_t;
typedef double double_t;
#line 519
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 574
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4251 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/types/TinyError.h"
enum __nesc_unnamed4252 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 40 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
# 19 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/machine/types.h" 3
typedef long int __off_t;
typedef int __pid_t;

__extension__ 
#line 22
typedef long long int __loff_t;
# 92 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;



typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;



typedef unsigned long clock_t;




typedef long time_t;







struct timespec {
  time_t tv_sec;
  long tv_nsec;
};


struct itimerspec {
  struct timespec it_interval;
  struct timespec it_value;
};

typedef long daddr_t;
typedef char *caddr_t;






typedef unsigned short ino_t;
#line 166
typedef _off_t off_t;
typedef __dev_t dev_t;
typedef __uid_t uid_t;
typedef __gid_t gid_t;





typedef int pid_t;







typedef long key_t;

typedef _ssize_t ssize_t;
#line 199
typedef unsigned int mode_t __attribute((__mode__(__SI__))) ;




typedef unsigned short nlink_t;
#line 226
typedef long fd_mask;









#line 234
typedef struct _types_fd_set {
  fd_mask fds_bits[(64 + (sizeof(fd_mask ) * 8 - 1)) / (sizeof(fd_mask ) * 8)];
} _types_fd_set;
#line 257
typedef unsigned long clockid_t;




typedef unsigned long timer_t;



typedef unsigned long useconds_t;
typedef long suseconds_t;
# 51 "/home/taoli/sat/lib/gcc/arm-none-eabi/4.8.2/../../../../arm-none-eabi/include/stdio.h" 3
typedef __FILE FILE;




typedef _fpos_t fpos_t;
#line 552
typedef ssize_t cookie_read_function_t(void *__cookie, char *__buf, size_t __n);
typedef ssize_t cookie_write_function_t(void *__cookie, const char *__buf, 
size_t __n);




typedef int cookie_seek_function_t(void *__cookie, off_t *__off, int __whence);

typedef int cookie_close_function_t(void *__cookie);








#line 562
typedef struct __nesc_unnamed4253 {



  cookie_read_function_t *read;
  cookie_write_function_t *write;
  cookie_seek_function_t *seek;
  cookie_close_function_t *close;
} cookie_io_functions_t;
# 239 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/stm32f4xx.h"
#line 145
typedef enum IRQn {


  NonMaskableInt_IRQn = -14, 
  MemoryManagement_IRQn = -12, 
  BusFault_IRQn = -11, 
  UsageFault_IRQn = -10, 
  SVCall_IRQn = -5, 
  DebugMonitor_IRQn = -4, 
  PendSV_IRQn = -2, 
  SysTick_IRQn = -1, 

  WWDG_IRQn = 0, 
  PVD_IRQn = 1, 
  TAMP_STAMP_IRQn = 2, 
  RTC_WKUP_IRQn = 3, 
  FLASH_IRQn = 4, 
  RCC_IRQn = 5, 
  EXTI0_IRQn = 6, 
  EXTI1_IRQn = 7, 
  EXTI2_IRQn = 8, 
  EXTI3_IRQn = 9, 
  EXTI4_IRQn = 10, 
  DMA1_Stream0_IRQn = 11, 
  DMA1_Stream1_IRQn = 12, 
  DMA1_Stream2_IRQn = 13, 
  DMA1_Stream3_IRQn = 14, 
  DMA1_Stream4_IRQn = 15, 
  DMA1_Stream5_IRQn = 16, 
  DMA1_Stream6_IRQn = 17, 
  ADC_IRQn = 18, 
  CAN1_TX_IRQn = 19, 
  CAN1_RX0_IRQn = 20, 
  CAN1_RX1_IRQn = 21, 
  CAN1_SCE_IRQn = 22, 
  EXTI9_5_IRQn = 23, 
  TIM1_BRK_TIM9_IRQn = 24, 
  TIM1_UP_TIM10_IRQn = 25, 
  TIM1_TRG_COM_TIM11_IRQn = 26, 
  TIM1_CC_IRQn = 27, 
  TIM2_IRQn = 28, 
  TIM3_IRQn = 29, 
  TIM4_IRQn = 30, 
  I2C1_EV_IRQn = 31, 
  I2C1_ER_IRQn = 32, 
  I2C2_EV_IRQn = 33, 
  I2C2_ER_IRQn = 34, 
  SPI1_IRQn = 35, 
  SPI2_IRQn = 36, 
  USART1_IRQn = 37, 
  USART2_IRQn = 38, 
  USART3_IRQn = 39, 
  EXTI15_10_IRQn = 40, 
  RTC_Alarm_IRQn = 41, 
  OTG_FS_WKUP_IRQn = 42, 
  TIM8_BRK_TIM12_IRQn = 43, 
  TIM8_UP_TIM13_IRQn = 44, 
  TIM8_TRG_COM_TIM14_IRQn = 45, 
  TIM8_CC_IRQn = 46, 
  DMA1_Stream7_IRQn = 47, 
  FSMC_IRQn = 48, 
  SDIO_IRQn = 49, 
  TIM5_IRQn = 50, 
  SPI3_IRQn = 51, 
  UART4_IRQn = 52, 
  UART5_IRQn = 53, 
  TIM6_DAC_IRQn = 54, 
  TIM7_IRQn = 55, 
  DMA2_Stream0_IRQn = 56, 
  DMA2_Stream1_IRQn = 57, 
  DMA2_Stream2_IRQn = 58, 
  DMA2_Stream3_IRQn = 59, 
  DMA2_Stream4_IRQn = 60, 
  ETH_IRQn = 61, 
  ETH_WKUP_IRQn = 62, 
  CAN2_TX_IRQn = 63, 
  CAN2_RX0_IRQn = 64, 
  CAN2_RX1_IRQn = 65, 
  CAN2_SCE_IRQn = 66, 
  OTG_FS_IRQn = 67, 
  DMA2_Stream5_IRQn = 68, 
  DMA2_Stream6_IRQn = 69, 
  DMA2_Stream7_IRQn = 70, 
  USART6_IRQn = 71, 
  I2C3_EV_IRQn = 72, 
  I2C3_ER_IRQn = 73, 
  OTG_HS_EP1_OUT_IRQn = 74, 
  OTG_HS_EP1_IN_IRQn = 75, 
  OTG_HS_WKUP_IRQn = 76, 
  OTG_HS_IRQn = 77, 
  DCMI_IRQn = 78, 
  CRYP_IRQn = 79, 
  HASH_RNG_IRQn = 80, 
  FPU_IRQn = 81
} IRQn_Type;
# 236 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/core/core_cm4.h"
#line 218
typedef union __nesc_unnamed4254 {

  struct __nesc_unnamed4255 {




    uint32_t _reserved0 : 16;
    uint32_t GE : 4;
    uint32_t _reserved1 : 7;

    uint32_t Q : 1;
    uint32_t V : 1;
    uint32_t C : 1;
    uint32_t Z : 1;
    uint32_t N : 1;
  } b;
  uint32_t w;
} APSR_Type;
#line 249
#line 241
typedef union __nesc_unnamed4256 {

  struct __nesc_unnamed4257 {

    uint32_t ISR : 9;
    uint32_t _reserved0 : 23;
  } b;
  uint32_t w;
} IPSR_Type;
#line 275
#line 254
typedef union __nesc_unnamed4258 {

  struct __nesc_unnamed4259 {

    uint32_t ISR : 9;



    uint32_t _reserved0 : 7;
    uint32_t GE : 4;
    uint32_t _reserved1 : 4;

    uint32_t T : 1;
    uint32_t IT : 2;
    uint32_t Q : 1;
    uint32_t V : 1;
    uint32_t C : 1;
    uint32_t Z : 1;
    uint32_t N : 1;
  } b;
  uint32_t w;
} xPSR_Type;
#line 290
#line 280
typedef union __nesc_unnamed4260 {

  struct __nesc_unnamed4261 {

    uint32_t nPRIV : 1;
    uint32_t SPSEL : 1;
    uint32_t FPCA : 1;
    uint32_t _reserved0 : 29;
  } b;
  uint32_t w;
} CONTROL_Type;
#line 318
#line 303
typedef struct __nesc_unnamed4262 {

  volatile uint32_t ISER[8];
  uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];
  uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];
  uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];
  uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];
  uint32_t RESERVED4[56];
  volatile uint8_t IP[240];
  uint32_t RESERVED5[644];
  volatile uint32_t STIR;
} NVIC_Type;
#line 358
#line 335
typedef struct __nesc_unnamed4263 {

  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile const uint32_t PFR[2];
  volatile const uint32_t DFR;
  volatile const uint32_t ADR;
  volatile const uint32_t MMFR[4];
  volatile const uint32_t ISAR[5];
  uint32_t RESERVED0[5];
  volatile uint32_t CPACR;
} SCB_Type;
#line 557
#line 552
typedef struct __nesc_unnamed4264 {

  uint32_t RESERVED0[1];
  volatile const uint32_t ICTR;
  volatile uint32_t ACTLR;
} SCnSCB_Type;
#line 596
#line 590
typedef struct __nesc_unnamed4265 {

  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
#line 654
#line 640
typedef struct __nesc_unnamed4266 {

  volatile union __nesc_unnamed4267 {

    volatile uint8_t u8;
    volatile uint16_t u16;
    volatile uint32_t u32;
  } PORT[32];
  uint32_t RESERVED0[864];
  volatile uint32_t TER;
  uint32_t RESERVED1[15];
  volatile uint32_t TPR;
  uint32_t RESERVED2[15];
  volatile uint32_t TCR;
} ITM_Type;
#line 713
#line 700
typedef struct __nesc_unnamed4268 {

  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RASR;
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
} MPU_Type;
#line 783
#line 775
typedef struct __nesc_unnamed4269 {

  uint32_t RESERVED0[1];
  volatile uint32_t FPCCR;
  volatile uint32_t FPCAR;
  volatile uint32_t FPDSCR;
  volatile const uint32_t MVFR0;
  volatile const uint32_t MVFR1;
} FPU_Type;
#line 886
#line 880
typedef struct __nesc_unnamed4270 {

  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
} CoreDebug_Type;
# 80 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/system_stm32f4xx.h"
extern void SystemInit(void );
# 253 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/stm32f4xx.h"
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;

typedef volatile int32_t vs32;
typedef volatile int16_t vs16;
typedef volatile int8_t vs8;

typedef volatile const int32_t vsc32;
typedef volatile const int16_t vsc16;
typedef volatile const int8_t vsc8;

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;

typedef volatile uint32_t vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t vu8;

typedef volatile const uint32_t vuc32;
typedef volatile const uint16_t vuc16;
typedef volatile const uint8_t vuc8;

typedef enum __nesc_unnamed4271 {
#line 285
  RESET = 0, SET = !RESET
} 
#line 285
FlagStatus;
#line 285
typedef enum __nesc_unnamed4271 ITStatus;

typedef enum __nesc_unnamed4272 {
#line 287
  DISABLE = 0, ENABLE = !DISABLE
} 
#line 287
FunctionalState;


typedef enum __nesc_unnamed4273 {
#line 290
  ERROR = 0, SUCCESS_lib = !ERROR
} 
#line 290
ErrorStatus;
#line 326
#line 304
typedef struct __nesc_unnamed4274 {

  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;







#line 328
typedef struct __nesc_unnamed4275 {

  volatile uint32_t CSR;
  volatile uint32_t CCR;
  volatile uint32_t CDR;
} 
ADC_Common_TypeDef;
#line 347
#line 341
typedef struct __nesc_unnamed4276 {

  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;
#line 359
#line 353
typedef struct __nesc_unnamed4277 {

  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;









#line 365
typedef struct __nesc_unnamed4278 {

  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;
#line 399
#line 375
typedef struct __nesc_unnamed4279 {

  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t RESERVED2;
  volatile uint32_t FS1R;
  uint32_t RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t RESERVED4;
  volatile uint32_t FA1R;
  uint32_t RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[28];
} CAN_TypeDef;
#line 412
#line 405
typedef struct __nesc_unnamed4280 {

  volatile uint32_t DR;
  volatile uint8_t IDR;
  uint8_t RESERVED0;
  uint16_t RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;
#line 434
#line 418
typedef struct __nesc_unnamed4281 {

  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;
  volatile uint32_t SR;
} DAC_TypeDef;
#line 446
#line 440
typedef struct __nesc_unnamed4282 {

  volatile uint32_t IDCODE;
  volatile uint32_t CR;
  volatile uint32_t APB1FZ;
  volatile uint32_t APB2FZ;
} DBGMCU_TypeDef;
#line 465
#line 452
typedef struct __nesc_unnamed4283 {

  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t RISR;
  volatile uint32_t IER;
  volatile uint32_t MISR;
  volatile uint32_t ICR;
  volatile uint32_t ESCR;
  volatile uint32_t ESUR;
  volatile uint32_t CWSTRTR;
  volatile uint32_t CWSIZER;
  volatile uint32_t DR;
} DCMI_TypeDef;
#line 479
#line 471
typedef struct __nesc_unnamed4284 {

  volatile uint32_t CR;
  volatile uint32_t NDTR;
  volatile uint32_t PAR;
  volatile uint32_t M0AR;
  volatile uint32_t M1AR;
  volatile uint32_t FCR;
} DMA_Stream_TypeDef;







#line 481
typedef struct __nesc_unnamed4285 {

  volatile uint32_t LISR;
  volatile uint32_t HISR;
  volatile uint32_t LIFCR;
  volatile uint32_t HIFCR;
} DMA_TypeDef;
#line 561
#line 493
typedef struct __nesc_unnamed4286 {

  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;
  uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;
  volatile uint32_t MACPMTCSR;
  uint32_t RESERVED1[2];
  volatile uint32_t MACSR;
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;
  uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;
  uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;
  volatile uint32_t MMCTGFMSCCR;
  uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;
#line 575
#line 567
typedef struct __nesc_unnamed4287 {

  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;
#line 589
#line 581
typedef struct __nesc_unnamed4288 {

  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t OPTCR;
} FLASH_TypeDef;








#line 595
typedef struct __nesc_unnamed4289 {

  volatile uint32_t BTCR[8];
} FSMC_Bank1_TypeDef;








#line 604
typedef struct __nesc_unnamed4290 {

  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;
#line 621
#line 613
typedef struct __nesc_unnamed4291 {

  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t RESERVED0;
  volatile uint32_t ECCR2;
} FSMC_Bank2_TypeDef;
#line 635
#line 627
typedef struct __nesc_unnamed4292 {

  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t RESERVED0;
  volatile uint32_t ECCR3;
} FSMC_Bank3_TypeDef;
#line 648
#line 641
typedef struct __nesc_unnamed4293 {

  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4;
} FSMC_Bank4_TypeDef;
#line 666
#line 654
typedef struct __nesc_unnamed4294 {

  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint16_t BSRRL;
  volatile uint16_t BSRRH;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} GPIO_TypeDef;
#line 679
#line 672
typedef struct __nesc_unnamed4295 {

  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED[2];
  volatile uint32_t CMPCR;
} SYSCFG_TypeDef;
#line 705
#line 685
typedef struct __nesc_unnamed4296 {

  volatile uint16_t CR1;
  uint16_t RESERVED0;
  volatile uint16_t CR2;
  uint16_t RESERVED1;
  volatile uint16_t OAR1;
  uint16_t RESERVED2;
  volatile uint16_t OAR2;
  uint16_t RESERVED3;
  volatile uint16_t DR;
  uint16_t RESERVED4;
  volatile uint16_t SR1;
  uint16_t RESERVED5;
  volatile uint16_t SR2;
  uint16_t RESERVED6;
  volatile uint16_t CCR;
  uint16_t RESERVED7;
  volatile uint16_t TRISE;
  uint16_t RESERVED8;
} I2C_TypeDef;
#line 717
#line 711
typedef struct __nesc_unnamed4297 {

  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;









#line 723
typedef struct __nesc_unnamed4298 {

  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;
#line 765
#line 733
typedef struct __nesc_unnamed4299 {

  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  uint32_t RESERVED0;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t RESERVED2;
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
  uint32_t RESERVED3[2];
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t AHB3LPENR;
  uint32_t RESERVED4;
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  uint32_t RESERVED5[2];
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t RESERVED6[2];
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
} RCC_TypeDef;
#line 813
#line 771
typedef struct __nesc_unnamed4300 {

  volatile uint32_t TR;
  volatile uint32_t DR;
  volatile uint32_t CR;
  volatile uint32_t ISR;
  volatile uint32_t PRER;
  volatile uint32_t WUTR;
  volatile uint32_t CALIBR;
  volatile uint32_t ALRMAR;
  volatile uint32_t ALRMBR;
  volatile uint32_t WPR;
  volatile uint32_t SSR;
  volatile uint32_t SHIFTR;
  volatile uint32_t TSTR;
  volatile uint32_t TSDR;
  volatile uint32_t TSSSR;
  volatile uint32_t CALR;
  volatile uint32_t TAFCR;
  volatile uint32_t ALRMASSR;
  volatile uint32_t ALRMBSSR;
  uint32_t RESERVED7;
  volatile uint32_t BKP0R;
  volatile uint32_t BKP1R;
  volatile uint32_t BKP2R;
  volatile uint32_t BKP3R;
  volatile uint32_t BKP4R;
  volatile uint32_t BKP5R;
  volatile uint32_t BKP6R;
  volatile uint32_t BKP7R;
  volatile uint32_t BKP8R;
  volatile uint32_t BKP9R;
  volatile uint32_t BKP10R;
  volatile uint32_t BKP11R;
  volatile uint32_t BKP12R;
  volatile uint32_t BKP13R;
  volatile uint32_t BKP14R;
  volatile uint32_t BKP15R;
  volatile uint32_t BKP16R;
  volatile uint32_t BKP17R;
  volatile uint32_t BKP18R;
  volatile uint32_t BKP19R;
} RTC_TypeDef;
#line 841
#line 819
typedef struct __nesc_unnamed4301 {

  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;
#line 867
#line 847
typedef struct __nesc_unnamed4302 {

  volatile uint16_t CR1;
  uint16_t RESERVED0;
  volatile uint16_t CR2;
  uint16_t RESERVED1;
  volatile uint16_t SR;
  uint16_t RESERVED2;
  volatile uint16_t DR;
  uint16_t RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t RESERVED8;
} SPI_TypeDef;
#line 911
#line 873
typedef struct __nesc_unnamed4303 {

  volatile uint16_t CR1;
  uint16_t RESERVED0;
  volatile uint16_t CR2;
  uint16_t RESERVED1;
  volatile uint16_t SMCR;
  uint16_t RESERVED2;
  volatile uint16_t DIER;
  uint16_t RESERVED3;
  volatile uint16_t SR;
  uint16_t RESERVED4;
  volatile uint16_t EGR;
  uint16_t RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t RESERVED7;
  volatile uint16_t CCER;
  uint16_t RESERVED8;
  volatile uint32_t CNT;
  volatile uint16_t PSC;
  uint16_t RESERVED9;
  volatile uint32_t ARR;
  volatile uint16_t RCR;
  uint16_t RESERVED10;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint16_t BDTR;
  uint16_t RESERVED11;
  volatile uint16_t DCR;
  uint16_t RESERVED12;
  volatile uint16_t DMAR;
  uint16_t RESERVED13;
  volatile uint16_t OR;
  uint16_t RESERVED14;
} TIM_TypeDef;
#line 933
#line 917
typedef struct __nesc_unnamed4304 {

  volatile uint16_t SR;
  uint16_t RESERVED0;
  volatile uint16_t DR;
  uint16_t RESERVED1;
  volatile uint16_t BRR;
  uint16_t RESERVED2;
  volatile uint16_t CR1;
  uint16_t RESERVED3;
  volatile uint16_t CR2;
  uint16_t RESERVED4;
  volatile uint16_t CR3;
  uint16_t RESERVED5;
  volatile uint16_t GTPR;
  uint16_t RESERVED6;
} USART_TypeDef;










#line 939
typedef struct __nesc_unnamed4305 {

  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;
#line 972
#line 950
typedef struct __nesc_unnamed4306 {

  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t DOUT;
  volatile uint32_t DMACR;
  volatile uint32_t IMSCR;
  volatile uint32_t RISR;
  volatile uint32_t MISR;
  volatile uint32_t K0LR;
  volatile uint32_t K0RR;
  volatile uint32_t K1LR;
  volatile uint32_t K1RR;
  volatile uint32_t K2LR;
  volatile uint32_t K2RR;
  volatile uint32_t K3LR;
  volatile uint32_t K3RR;
  volatile uint32_t IV0LR;
  volatile uint32_t IV0RR;
  volatile uint32_t IV1LR;
  volatile uint32_t IV1RR;
} CRYP_TypeDef;
#line 988
#line 978
typedef struct __nesc_unnamed4307 {

  volatile uint32_t CR;
  volatile uint32_t DIN;
  volatile uint32_t STR;
  volatile uint32_t HR[5];
  volatile uint32_t IMR;
  volatile uint32_t SR;
  uint32_t RESERVED[52];
  volatile uint32_t CSR[51];
} HASH_TypeDef;










#line 994
typedef struct __nesc_unnamed4308 {

  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t DR;
} RNG_TypeDef;
# 79 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_adc.h"
#line 53
typedef struct __nesc_unnamed4309 {

  uint32_t ADC_Resolution;

  FunctionalState ADC_ScanConvMode;



  FunctionalState ADC_ContinuousConvMode;


  uint32_t ADC_ExternalTrigConvEdge;



  uint32_t ADC_ExternalTrigConv;



  uint32_t ADC_DataAlign;


  uint8_t ADC_NbrOfConversion;
} 


ADC_InitTypeDef;
#line 100
#line 84
typedef struct __nesc_unnamed4310 {

  uint32_t ADC_Mode;


  uint32_t ADC_Prescaler;


  uint32_t ADC_DMAAccessMode;



  uint32_t ADC_TwoSamplingDelay;
} 


ADC_CommonInitTypeDef;
# 93 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_can.h"
#line 56
typedef struct __nesc_unnamed4311 {

  uint16_t CAN_Prescaler;


  uint8_t CAN_Mode;


  uint8_t CAN_SJW;




  uint8_t CAN_BS1;



  uint8_t CAN_BS2;


  FunctionalState CAN_TTCM;


  FunctionalState CAN_ABOM;


  FunctionalState CAN_AWUM;


  FunctionalState CAN_NART;


  FunctionalState CAN_RFLM;


  FunctionalState CAN_TXFP;
} 
CAN_InitTypeDef;
#line 131
#line 98
typedef struct __nesc_unnamed4312 {

  uint16_t CAN_FilterIdHigh;



  uint16_t CAN_FilterIdLow;



  uint16_t CAN_FilterMaskIdHigh;




  uint16_t CAN_FilterMaskIdLow;




  uint16_t CAN_FilterFIFOAssignment;


  uint8_t CAN_FilterNumber;

  uint8_t CAN_FilterMode;


  uint8_t CAN_FilterScale;


  FunctionalState CAN_FilterActivation;
} 
CAN_FilterInitTypeDef;
#line 158
#line 136
typedef struct __nesc_unnamed4313 {

  uint32_t StdId;


  uint32_t ExtId;


  uint8_t IDE;



  uint8_t RTR;



  uint8_t DLC;



  uint8_t Data[8];
} 
CanTxMsg;
#line 188
#line 163
typedef struct __nesc_unnamed4314 {

  uint32_t StdId;


  uint32_t ExtId;


  uint8_t IDE;



  uint8_t RTR;



  uint8_t DLC;


  uint8_t Data[8];


  uint8_t FMI;
} 

CanRxMsg;
# 65 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_cryp.h"
#line 53
typedef struct __nesc_unnamed4315 {

  uint16_t CRYP_AlgoDir;

  uint16_t CRYP_AlgoMode;


  uint16_t CRYP_DataType;

  uint16_t CRYP_KeySize;
} 

CRYP_InitTypeDef;
#line 80
#line 70
typedef struct __nesc_unnamed4316 {

  uint32_t CRYP_Key0Left;
  uint32_t CRYP_Key0Right;
  uint32_t CRYP_Key1Left;
  uint32_t CRYP_Key1Right;
  uint32_t CRYP_Key2Left;
  uint32_t CRYP_Key2Right;
  uint32_t CRYP_Key3Left;
  uint32_t CRYP_Key3Right;
} CRYP_KeyInitTypeDef;









#line 84
typedef struct __nesc_unnamed4317 {

  uint32_t CRYP_IV0Left;
  uint32_t CRYP_IV0Right;
  uint32_t CRYP_IV1Left;
  uint32_t CRYP_IV1Right;
} CRYP_IVInitTypeDef;
#line 113
#line 95
typedef struct __nesc_unnamed4318 {


  uint32_t CR_bits9to2;

  uint32_t CRYP_IV0LR;
  uint32_t CRYP_IV0RR;
  uint32_t CRYP_IV1LR;
  uint32_t CRYP_IV1RR;

  uint32_t CRYP_K0LR;
  uint32_t CRYP_K0RR;
  uint32_t CRYP_K1LR;
  uint32_t CRYP_K1RR;
  uint32_t CRYP_K2LR;
  uint32_t CRYP_K2RR;
  uint32_t CRYP_K3LR;
  uint32_t CRYP_K3RR;
} CRYP_Context;
# 69 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_dac.h"
#line 54
typedef struct __nesc_unnamed4319 {

  uint32_t DAC_Trigger;


  uint32_t DAC_WaveGeneration;



  uint32_t DAC_LFSRUnmask_TriangleAmplitude;



  uint32_t DAC_OutputBuffer;
} 
DAC_InitTypeDef;
# 73 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_dcmi.h"
#line 51
typedef struct __nesc_unnamed4320 {

  uint16_t DCMI_CaptureMode;


  uint16_t DCMI_SynchroMode;


  uint16_t DCMI_PCKPolarity;


  uint16_t DCMI_VSPolarity;


  uint16_t DCMI_HSPolarity;


  uint16_t DCMI_CaptureRate;


  uint16_t DCMI_ExtendedDataMode;
} 
DCMI_InitTypeDef;
#line 92
#line 78
typedef struct __nesc_unnamed4321 {

  uint16_t DCMI_VerticalStartLine;


  uint16_t DCMI_HorizontalOffsetCount;


  uint16_t DCMI_VerticalLineCount;


  uint16_t DCMI_CaptureCount;
} 

DCMI_CROPInitTypeDef;










#line 97
typedef struct __nesc_unnamed4322 {

  uint8_t DCMI_FrameStartCode;
  uint8_t DCMI_LineStartCode;
  uint8_t DCMI_LineEndCode;
  uint8_t DCMI_FrameEndCode;
} DCMI_CodesInitTypeDef;
# 110 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_dma.h"
#line 54
typedef struct __nesc_unnamed4323 {

  uint32_t DMA_Channel;


  uint32_t DMA_PeripheralBaseAddr;

  uint32_t DMA_Memory0BaseAddr;



  uint32_t DMA_DIR;



  uint32_t DMA_BufferSize;



  uint32_t DMA_PeripheralInc;


  uint32_t DMA_MemoryInc;


  uint32_t DMA_PeripheralDataSize;


  uint32_t DMA_MemoryDataSize;


  uint32_t DMA_Mode;




  uint32_t DMA_Priority;


  uint32_t DMA_FIFOMode;




  uint32_t DMA_FIFOThreshold;


  uint32_t DMA_MemoryBurst;




  uint32_t DMA_PeripheralBurst;
} 


DMA_InitTypeDef;
# 58 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_exti.h"
#line 54
typedef enum __nesc_unnamed4324 {

  EXTI_Mode_Interrupt = 0x00, 
  EXTI_Mode_Event = 0x04
} EXTIMode_TypeDef;
#line 71
#line 66
typedef enum __nesc_unnamed4325 {

  EXTI_Trigger_Rising = 0x08, 
  EXTI_Trigger_Falling = 0x0C, 
  EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;
#line 93
#line 80
typedef struct __nesc_unnamed4326 {

  uint32_t EXTI_Line;


  EXTIMode_TypeDef EXTI_Mode;


  EXTITrigger_TypeDef EXTI_Trigger;


  FunctionalState EXTI_LineCmd;
} 
EXTI_InitTypeDef;
# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_flash.h"
#line 52
typedef enum __nesc_unnamed4327 {

  FLASH_BUSY = 1, 
  FLASH_ERROR_PGS, 
  FLASH_ERROR_PGP, 
  FLASH_ERROR_PGA, 
  FLASH_ERROR_WRP, 
  FLASH_ERROR_PROGRAM, 
  FLASH_ERROR_OPERATION, 
  FLASH_COMPLETE
} FLASH_Status;
# 89 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_fsmc.h"
#line 53
typedef struct __nesc_unnamed4328 {

  uint32_t FSMC_AddressSetupTime;




  uint32_t FSMC_AddressHoldTime;




  uint32_t FSMC_DataSetupTime;




  uint32_t FSMC_BusTurnAroundDuration;




  uint32_t FSMC_CLKDivision;



  uint32_t FSMC_DataLatency;







  uint32_t FSMC_AccessMode;
} 
FSMC_NORSRAMTimingInitTypeDef;
#line 147
#line 94
typedef struct __nesc_unnamed4329 {

  uint32_t FSMC_Bank;


  uint32_t FSMC_DataAddressMux;



  uint32_t FSMC_MemoryType;



  uint32_t FSMC_MemoryDataWidth;


  uint32_t FSMC_BurstAccessMode;



  uint32_t FSMC_AsynchronousWait;



  uint32_t FSMC_WaitSignalPolarity;



  uint32_t FSMC_WrapMode;



  uint32_t FSMC_WaitSignalActive;




  uint32_t FSMC_WriteOperation;


  uint32_t FSMC_WaitSignal;



  uint32_t FSMC_ExtendedMode;


  uint32_t FSMC_WriteBurst;


  FSMC_NORSRAMTimingInitTypeDef *FSMC_ReadWriteTimingStruct;

  FSMC_NORSRAMTimingInitTypeDef *FSMC_WriteTimingStruct;
} FSMC_NORSRAMInitTypeDef;
#line 178
#line 152
typedef struct __nesc_unnamed4330 {

  uint32_t FSMC_SetupTime;





  uint32_t FSMC_WaitSetupTime;





  uint32_t FSMC_HoldSetupTime;






  uint32_t FSMC_HiZSetupTime;
} 



FSMC_NAND_PCCARDTimingInitTypeDef;
#line 211
#line 183
typedef struct __nesc_unnamed4331 {

  uint32_t FSMC_Bank;


  uint32_t FSMC_Waitfeature;


  uint32_t FSMC_MemoryDataWidth;


  uint32_t FSMC_ECC;


  uint32_t FSMC_ECCPageSize;


  uint32_t FSMC_TCLRSetupTime;



  uint32_t FSMC_TARSetupTime;



  FSMC_NAND_PCCARDTimingInitTypeDef *FSMC_CommonSpaceTimingStruct;

  FSMC_NAND_PCCARDTimingInitTypeDef *FSMC_AttributeSpaceTimingStruct;
} FSMC_NANDInitTypeDef;
#line 236
#line 217
typedef struct __nesc_unnamed4332 {

  uint32_t FSMC_Waitfeature;


  uint32_t FSMC_TCLRSetupTime;



  uint32_t FSMC_TARSetupTime;




  FSMC_NAND_PCCARDTimingInitTypeDef *FSMC_CommonSpaceTimingStruct;

  FSMC_NAND_PCCARDTimingInitTypeDef *FSMC_AttributeSpaceTimingStruct;

  FSMC_NAND_PCCARDTimingInitTypeDef *FSMC_IOSpaceTimingStruct;
} FSMC_PCCARDInitTypeDef;
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_hash.h"
#line 53
typedef struct __nesc_unnamed4333 {

  uint32_t HASH_AlgoSelection;

  uint32_t HASH_AlgoMode;

  uint32_t HASH_DataType;


  uint32_t HASH_HMACKeyType;
} 
HASH_InitTypeDef;








#line 69
typedef struct __nesc_unnamed4334 {

  uint32_t Data[5];
} 
HASH_MsgDigest;










#line 78
typedef struct __nesc_unnamed4335 {

  uint32_t HASH_IMR;
  uint32_t HASH_STR;
  uint32_t HASH_CR;
  uint32_t HASH_CSR[51];
} HASH_Context;
# 69 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_gpio.h"
#line 63
typedef enum __nesc_unnamed4336 {

  GPIO_Mode_IN = 0x00, 
  GPIO_Mode_OUT = 0x01, 
  GPIO_Mode_AF = 0x02, 
  GPIO_Mode_AN = 0x03
} GPIOMode_TypeDef;










#line 76
typedef enum __nesc_unnamed4337 {

  GPIO_OType_PP = 0x00, 
  GPIO_OType_OD = 0x01
} GPIOOType_TypeDef;
#line 93
#line 87
typedef enum __nesc_unnamed4338 {

  GPIO_Speed_2MHz = 0x00, 
  GPIO_Speed_25MHz = 0x01, 
  GPIO_Speed_50MHz = 0x02, 
  GPIO_Speed_100MHz = 0x03
} GPIOSpeed_TypeDef;
#line 105
#line 100
typedef enum __nesc_unnamed4339 {

  GPIO_PuPd_NOPULL = 0x00, 
  GPIO_PuPd_UP = 0x01, 
  GPIO_PuPd_DOWN = 0x02
} GPIOPuPd_TypeDef;










#line 112
typedef enum __nesc_unnamed4340 {

  Bit_RESET = 0, 
  Bit_SET
} BitAction;
#line 139
#line 123
typedef struct __nesc_unnamed4341 {

  uint32_t GPIO_Pin;


  GPIOMode_TypeDef GPIO_Mode;


  GPIOSpeed_TypeDef GPIO_Speed;


  GPIOOType_TypeDef GPIO_OType;


  GPIOPuPd_TypeDef GPIO_PuPd;
} 
GPIO_InitTypeDef;
#line 380
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);








void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);


void GPIO_ToggleBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
# 73 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_i2c.h"
#line 54
typedef struct __nesc_unnamed4342 {

  uint32_t I2C_ClockSpeed;


  uint16_t I2C_Mode;


  uint16_t I2C_DutyCycle;


  uint16_t I2C_OwnAddress1;


  uint16_t I2C_Ack;


  uint16_t I2C_AcknowledgedAddress;
} 
I2C_InitTypeDef;
# 54 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_rcc.h"
#line 48
typedef struct __nesc_unnamed4343 {

  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
} RCC_ClocksTypeDef;
#line 477
void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);


void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
# 63 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_rtc.h"
#line 53
typedef struct __nesc_unnamed4344 {

  uint32_t RTC_HourFormat;


  uint32_t RTC_AsynchPrediv;


  uint32_t RTC_SynchPrediv;
} 
RTC_InitTypeDef;
#line 83
#line 68
typedef struct __nesc_unnamed4345 {

  uint8_t RTC_Hours;




  uint8_t RTC_Minutes;


  uint8_t RTC_Seconds;


  uint8_t RTC_H12;
} 
RTC_TimeTypeDef;
#line 101
#line 88
typedef struct __nesc_unnamed4346 {

  uint8_t RTC_WeekDay;


  uint8_t RTC_Month;


  uint8_t RTC_Date;


  uint8_t RTC_Year;
} 
RTC_DateTypeDef;
#line 121
#line 106
typedef struct __nesc_unnamed4347 {

  RTC_TimeTypeDef RTC_AlarmTime;

  uint32_t RTC_AlarmMask;


  uint32_t RTC_AlarmDateWeekDaySel;


  uint8_t RTC_AlarmDateWeekDay;
} 



RTC_AlarmTypeDef;
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_sdio.h"
#line 50
typedef struct __nesc_unnamed4348 {

  uint32_t SDIO_ClockEdge;


  uint32_t SDIO_ClockBypass;



  uint32_t SDIO_ClockPowerSave;



  uint32_t SDIO_BusWide;


  uint32_t SDIO_HardwareFlowControl;


  uint8_t SDIO_ClockDiv;
} 

SDIO_InitTypeDef;
#line 92
#line 74
typedef struct __nesc_unnamed4349 {

  uint32_t SDIO_Argument;




  uint32_t SDIO_CmdIndex;

  uint32_t SDIO_Response;


  uint32_t SDIO_Wait;


  uint32_t SDIO_CPSM;
} 

SDIO_CmdInitTypeDef;
#line 113
#line 94
typedef struct __nesc_unnamed4350 {

  uint32_t SDIO_DataTimeOut;

  uint32_t SDIO_DataLength;

  uint32_t SDIO_DataBlockSize;


  uint32_t SDIO_TransferDir;



  uint32_t SDIO_TransferMode;


  uint32_t SDIO_DPSM;
} 

SDIO_DataInitTypeDef;
# 85 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_spi.h"
#line 54
typedef struct __nesc_unnamed4351 {

  uint16_t SPI_Direction;


  uint16_t SPI_Mode;


  uint16_t SPI_DataSize;


  uint16_t SPI_CPOL;


  uint16_t SPI_CPHA;


  uint16_t SPI_NSS;



  uint16_t SPI_BaudRatePrescaler;





  uint16_t SPI_FirstBit;


  uint16_t SPI_CRCPolynomial;
} SPI_InitTypeDef;
#line 111
#line 91
typedef struct __nesc_unnamed4352 {


  uint16_t I2S_Mode;


  uint16_t I2S_Standard;


  uint16_t I2S_DataFormat;


  uint16_t I2S_MCLKOutput;


  uint32_t I2S_AudioFreq;


  uint16_t I2S_CPOL;
} 
I2S_InitTypeDef;
# 78 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_tim.h"
#line 55
typedef struct __nesc_unnamed4353 {

  uint16_t TIM_Prescaler;


  uint16_t TIM_CounterMode;


  uint32_t TIM_Period;



  uint16_t TIM_ClockDivision;


  uint8_t TIM_RepetitionCounter;
} 






TIM_TimeBaseInitTypeDef;
#line 113
#line 84
typedef struct __nesc_unnamed4354 {

  uint16_t TIM_OCMode;


  uint16_t TIM_OutputState;


  uint16_t TIM_OutputNState;



  uint32_t TIM_Pulse;


  uint16_t TIM_OCPolarity;


  uint16_t TIM_OCNPolarity;



  uint16_t TIM_OCIdleState;



  uint16_t TIM_OCNIdleState;
} 

TIM_OCInitTypeDef;
#line 136
#line 119
typedef struct __nesc_unnamed4355 {


  uint16_t TIM_Channel;


  uint16_t TIM_ICPolarity;


  uint16_t TIM_ICSelection;


  uint16_t TIM_ICPrescaler;


  uint16_t TIM_ICFilter;
} 
TIM_ICInitTypeDef;
#line 167
#line 143
typedef struct __nesc_unnamed4356 {


  uint16_t TIM_OSSRState;


  uint16_t TIM_OSSIState;


  uint16_t TIM_LOCKLevel;


  uint16_t TIM_DeadTime;



  uint16_t TIM_Break;


  uint16_t TIM_BreakPolarity;


  uint16_t TIM_AutomaticOutput;
} 
TIM_BDTRInitTypeDef;
#line 1026
void TIM_DeInit(TIM_TypeDef *TIMx);
void TIM_TimeBaseInit(TIM_TypeDef *TIMx, TIM_TimeBaseInitTypeDef *TIM_TimeBaseInitStruct);
#line 1040
void TIM_Cmd(TIM_TypeDef *TIMx, FunctionalState NewState);
#line 1100
void TIM_ITConfig(TIM_TypeDef *TIMx, uint16_t TIM_IT, FunctionalState NewState);

FlagStatus TIM_GetFlagStatus(TIM_TypeDef *TIMx, uint16_t TIM_FLAG);

ITStatus TIM_GetITStatus(TIM_TypeDef *TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef *TIMx, uint16_t TIM_IT);
# 81 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/stm32f4xx_usart.h"
#line 54
typedef struct __nesc_unnamed4357 {

  uint32_t USART_BaudRate;





  uint16_t USART_WordLength;


  uint16_t USART_StopBits;


  uint16_t USART_Parity;






  uint16_t USART_Mode;


  uint16_t USART_HardwareFlowControl;
} 

USART_InitTypeDef;
#line 102
#line 87
typedef struct __nesc_unnamed4358 {


  uint16_t USART_Clock;


  uint16_t USART_CPOL;


  uint16_t USART_CPHA;


  uint16_t USART_LastBit;
} 

USART_ClockInitTypeDef;
# 74 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/fwlib/inc/peripherals/misc.h"
#line 54
typedef struct __nesc_unnamed4359 {

  uint8_t NVIC_IRQChannel;




  uint8_t NVIC_IRQChannelPreemptionPriority;




  uint8_t NVIC_IRQChannelSubPriority;




  FunctionalState NVIC_IRQChannelCmd;
} 

NVIC_InitTypeDef;
#line 159
void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct);
# 92 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/stm32f4xx_it.h"
void TIM2_IRQHandler(void );

void TIM4_IRQHandler(void );
# 42 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/stm32f4hardware.h"
typedef uint32_t __nesc_atomic_t;



static __inline void __nesc_enable_interrupt();
#line 72
static __inline void __nesc_disable_interrupt();
#line 91
__inline __nesc_atomic_t __nesc_atomic_start(void )  ;
#line 118
__inline void __nesc_atomic_end(__nesc_atomic_t oldState)  ;
#line 147
typedef uint8_t mcu_power_t  ;
# 29 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4360 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4361 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4362 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 32 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4363 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
typedef TMicro BlinkC$Timer0$precision_tag;
typedef TMilli BlinkC$Timer1$precision_tag;
typedef TMicro /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$precision_tag /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$precision_tag /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$precision_tag;
typedef TMicro /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$precision_tag /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$precision_tag /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMicro STM32Micro16TIMC$Alarm$precision_tag;
typedef uint16_t STM32Micro16TIMC$Alarm$size_type;
typedef TMicro STM32Micro16TIMC$Counter$precision_tag;
typedef uint16_t STM32Micro16TIMC$Counter$size_type;
typedef TMicro /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_precision_tag;
typedef uint32_t /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type;
typedef TMicro /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_precision_tag;
typedef uint16_t /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_size_type;
typedef uint16_t /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$upper_count_type;
typedef /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_precision_tag /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$precision_tag;
typedef /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$size_type;
typedef /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_precision_tag /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$precision_tag;
typedef /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$size_type;
typedef TMicro /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_precision_tag;
typedef uint32_t /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type;
typedef TMicro /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_precision_tag;
typedef uint16_t /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_size_type;
typedef /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_precision_tag /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$precision_tag;
typedef /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$size_type;
typedef /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_precision_tag /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$precision_tag;
typedef /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$size_type;
typedef /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_precision_tag /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$precision_tag;
typedef /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$size_type;
typedef TMicro /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$precision_tag;
typedef /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$precision_tag /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$precision_tag /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$precision_tag /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$precision_tag /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$precision_tag;
typedef TMilli STM32Milli32TIMC$LocalTime$precision_tag;
typedef TMilli STM32Milli32TIMC$Alarm$precision_tag;
typedef uint32_t STM32Milli32TIMC$Alarm$size_type;
typedef TMilli STM32Milli32TIMC$Counter$precision_tag;
typedef uint32_t STM32Milli32TIMC$Counter$size_type;
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
#line 51
static error_t MoteClockP$MoteClockInit$init(void );
# 59 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t McuSleepC$McuSleepInit$init(void );
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b430d185cb0);
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b430d185cb0);
# 46 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );
#line 61
static void SchedulerBasicP$Scheduler$taskLoop(void );
#line 54
static bool SchedulerBasicP$Scheduler$runNextTask(void );
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC$Timer0$fired(void );
# 49 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Boot.nc"
static void BlinkC$Boot$booted(void );
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC$Timer1$fired(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP$Leds$led0Toggle(void );
#line 72
static void LedsP$Leds$led1Toggle(void );
# 31 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$toggle(void );



static void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$makeOutput(void );
#line 29
static void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$set(void );

static void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$toggle(void );



static void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$makeOutput(void );
#line 29
static void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$set(void );
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$fired(void );
#line 72
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b430d39d020);
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$startPeriodic(
# 37 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b430d39d020, 
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$runTask(void );
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$fired(void );
# 125 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$getNow(void );
#line 118
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$stop(void );
# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static STM32Micro16TIMC$Alarm$size_type STM32Micro16TIMC$Alarm$getNow(void );
#line 92
static void STM32Micro16TIMC$Alarm$startAt(STM32Micro16TIMC$Alarm$size_type t0, STM32Micro16TIMC$Alarm$size_type dt);
#line 62
static void STM32Micro16TIMC$Alarm$stop(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t STM32Micro16TIMC$Init$init(void );
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static STM32Micro16TIMC$Counter$size_type STM32Micro16TIMC$Counter$get(void );






static bool STM32Micro16TIMC$Counter$isOverflowPending(void );










static void /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$overflow(void );
#line 53
static /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$get(void );
# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$size_type t0, /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$fired(void );
# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$overflow(void );
#line 71
static void /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$runTask(void );
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$fired(void );
#line 72
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$default$fired(
# 37 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b430d39d020);
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$startPeriodic(
# 37 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b430d39d020, 
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$runTask(void );
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$fired(void );
# 125 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$stop(void );
# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static STM32Milli32TIMC$Alarm$size_type STM32Milli32TIMC$Alarm$getNow(void );
#line 92
static void STM32Milli32TIMC$Alarm$startAt(STM32Milli32TIMC$Alarm$size_type t0, STM32Milli32TIMC$Alarm$size_type dt);
#line 105
static STM32Milli32TIMC$Alarm$size_type STM32Milli32TIMC$Alarm$getAlarm(void );
#line 62
static void STM32Milli32TIMC$Alarm$stop(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t STM32Milli32TIMC$Init$init(void );
# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static void STM32Milli32TIMC$Counter$default$overflow(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP$MoteInit$init(void );
#line 51
static error_t PlatformP$MoteClockInit$init(void );
#line 51
static error_t PlatformP$McuSleepInit$init(void );
# 46 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/platforms/stm32f4/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
# 39 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/platforms/stm32f4/MoteClockP.nc"
static inline error_t MoteClockP$MoteClockInit$init(void );
# 43 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/McuSleepC.nc"
static inline error_t McuSleepC$McuSleepInit$init(void );
#line 66
static inline void McuSleepC$enable_interrupts(void );






static inline void McuSleepC$McuSleep$sleep(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP$SoftwareInit$init(void );
# 49 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainP$Boot$booted(void );
# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP$PlatformInit$init(void );
# 46 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void RealMainP$Scheduler$init(void );
#line 61
static void RealMainP$Scheduler$taskLoop(void );
#line 54
static bool RealMainP$Scheduler$runNextTask(void );
# 52 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 45 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b430d185cb0);
# 59 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP$McuSleep$sleep(void );
# 50 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4364 {

  SchedulerBasicP$NUM_TASKS = 4U, 
  SchedulerBasicP$NO_TASK = 255
};

uint8_t SchedulerBasicP$m_head;
uint8_t SchedulerBasicP$m_tail;
uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(void );
#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void );
#line 160
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static inline void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC$Timer0$startPeriodic(uint32_t dt);
#line 53
static void BlinkC$Timer1$startPeriodic(uint32_t dt);
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Leds.nc"
static void BlinkC$Leds$led0Toggle(void );
#line 72
static void BlinkC$Leds$led1Toggle(void );
# 50 "BlinkC.nc"
static inline void BlinkC$Boot$booted(void );






static inline void BlinkC$Timer0$fired(void );







static inline void BlinkC$Timer1$fired(void );
# 31 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$toggle(void );



static void LedsP$Led0$makeOutput(void );
#line 29
static void LedsP$Led0$set(void );

static void LedsP$Led1$toggle(void );



static void LedsP$Led1$makeOutput(void );
#line 29
static void LedsP$Led1$set(void );
# 45 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 73
static inline void LedsP$Leds$led0Toggle(void );
#line 88
static inline void LedsP$Leds$led1Toggle(void );
# 49 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static __inline void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$set(void );
#line 62
static inline void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$toggle(void );
#line 90
static __inline void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$makeOutput(void );
#line 49
static __inline void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$set(void );
#line 62
static inline void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$toggle(void );
#line 90
static __inline void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$makeOutput(void );
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$postTask(void );
# 125 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 118
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$stop(void );




static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$fired(
# 37 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b430d39d020);
#line 60
enum /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$__nesc_unnamed4365 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 0U
};
#line 60
typedef int /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$__nesc_unnamed4366 {

  VirtualizeTimerC$0$NUM_TIMERS = 1U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$__nesc_unnamed4367 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer_t /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$m_timers[/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 89
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
#line 128
static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$fired(void );




static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);
#line 193
static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$postTask(void );
# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$stop(void );
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$fired(void );
# 63 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$__nesc_unnamed4368 {
#line 63
  AlarmToTimerC$0$fired = 1U
};
#line 63
typedef int /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$stop(void );


static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$runTask(void );






static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$fired(void );
#line 82
static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$getNow(void );
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void STM32Micro16TIMC$Alarm$fired(void );
# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static void STM32Micro16TIMC$Counter$overflow(void );
# 44 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
uint16_t STM32Micro16TIMC$alarm;


bool STM32Micro16TIMC$running;


static TIM_TimeBaseInitTypeDef STM32Micro16TIMC$TIM_TimeBaseStructure = { 

.TIM_Period = 1, 
.TIM_Prescaler = 40 - 1, 
.TIM_ClockDivision = (uint16_t )0x0000, 
.TIM_CounterMode = (uint16_t )0x0000 };



inline static void STM32Micro16TIMC$timer_clock_init(void );






inline static void STM32Micro16TIMC$init_free_timer(void );
#line 78
inline static void STM32Micro16TIMC$init_alarm_timer(void );






inline static void STM32Micro16TIMC$init_timer_irq(void );
#line 103
inline static void STM32Micro16TIMC$set_alarm_interval(uint16_t interval);





static inline void STM32Micro16TIMC$enableInterrupt(void );









static void STM32Micro16TIMC$disableInterrupt(void );









static inline error_t STM32Micro16TIMC$Init$init(void );
#line 150
static inline void STM32Micro16TIMC$Alarm$stop(void );









static inline void STM32Micro16TIMC$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 204
static inline uint16_t STM32Micro16TIMC$Alarm$getNow(void );
#line 216
static inline uint16_t STM32Micro16TIMC$Counter$get(void );




static inline bool STM32Micro16TIMC$Counter$isOverflowPending(void );
#line 241
void TIM4_IRQHandler(void )   ;
#line 258
void TIM3_IRQHandler(void )   ;
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$get(void );






static bool /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$isOverflowPending(void );










static void /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$overflow(void );
# 68 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$upper_count_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$m_upper = 0;

enum /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$__nesc_unnamed4369 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 0, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_size_type ) - /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_size_type ) + 0, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$upper_count_type )2 << (/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$get(void );
#line 134
static inline void /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$overflow(void );
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$fired(void );
#line 92
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$size_type dt);
#line 62
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$stop(void );
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$get(void );
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0;
/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt;

enum /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$__nesc_unnamed4370 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_size_type ) - 1 - 0, 
  TransformAlarmC$0$MAX_DELAY = (/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type )1 << /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getNow(void );




static inline /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getAlarm(void );










static inline void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$set_alarm(void );
#line 142
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type t0, /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type dt);
#line 157
static inline void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$fired(void );
#line 174
static inline void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$overflow(void );
# 47 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$postTask(void );
# 125 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$getNow(void );
#line 118
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$stop(void );




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$fired(
# 37 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b430d39d020);
#line 60
enum /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$__nesc_unnamed4371 {
#line 60
  VirtualizeTimerC$1$updateFromTimer = 2U
};
#line 60
typedef int /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$__nesc_unnamed4372 {

  VirtualizeTimerC$1$NUM_TIMERS = 1U, 
  VirtualizeTimerC$1$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$__nesc_unnamed4373 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer_t;

/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$m_timers[/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$NUM_TIMERS];




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$fireTimers(uint32_t now);
#line 89
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$fired(void );




static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$startPeriodic(uint8_t num, uint32_t dt);
#line 193
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$default$fired(uint8_t num);
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$postTask(void );
# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$stop(void );
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$fired(void );
# 63 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$__nesc_unnamed4374 {
#line 63
  AlarmToTimerC$1$fired = 3U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$m_dt;
bool /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$stop(void );


static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$getNow(void );
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void STM32Milli32TIMC$Alarm$fired(void );
# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
static void STM32Milli32TIMC$Counter$overflow(void );
# 43 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
uint32_t STM32Milli32TIMC$alarm;
bool STM32Milli32TIMC$running;

static TIM_TimeBaseInitTypeDef STM32Milli32TIMC$TIM_TimeBaseStructure = 
{ 
.TIM_Period = 1, 
.TIM_Prescaler = 41016 - 1, 
.TIM_ClockDivision = (uint16_t )0x0000, 
.TIM_CounterMode = (uint16_t )0x0000 };


inline static void STM32Milli32TIMC$timer_clock_init(void );






inline static void STM32Milli32TIMC$init_free_timer(void );
#line 74
inline static void STM32Milli32TIMC$init_alarm_timer(void );






inline static void STM32Milli32TIMC$init_timer_irq(void );
#line 99
inline static void STM32Milli32TIMC$set_alarm_interval(uint32_t interval);






static inline void STM32Milli32TIMC$enableInterrupt(void );









static void STM32Milli32TIMC$disableInterrupt(void );









static inline error_t STM32Milli32TIMC$Init$init(void );
#line 147
static inline void STM32Milli32TIMC$Alarm$stop(void );









static void STM32Milli32TIMC$Alarm$startAt(uint32_t t0, uint32_t dt);
#line 201
static inline uint32_t STM32Milli32TIMC$Alarm$getNow(void );






static inline uint32_t STM32Milli32TIMC$Alarm$getAlarm(void );
#line 233
static inline void STM32Milli32TIMC$Counter$default$overflow(void );




void TIM5_IRQHandler(void )   ;
#line 255
void TIM2_IRQHandler(void )   ;
# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/stm32f4hardware.h"
static __inline void __nesc_disable_interrupt()
#line 72
{
  uint32_t statusReg = 0;


  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001, DISABLE);
  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001, DISABLE);









  return;
}


__inline  __nesc_atomic_t __nesc_atomic_start(void )
{
  uint32_t result = 0;





  __nesc_disable_interrupt();
#line 115
  return result;
}

#line 46
static __inline void __nesc_enable_interrupt()
#line 46
{
  uint32_t statusReg = 0;


  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001, ENABLE);
  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001, ENABLE);
#line 68
  return;
}

#line 118
__inline  void __nesc_atomic_end(__nesc_atomic_t oldState)
{
  uint32_t statusReg = 0;









  __nesc_enable_interrupt();
#line 144
  return;
}

# 113 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$init(void ){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 49 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static __inline void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$set(void )
#line 49
{
  GPIO_TypeDef *port = (GPIO_TypeDef *)1073875968U;

  GPIO_SetBits(port, 8192U);
}

# 29 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$set(void ){
#line 29
  /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$set();
#line 29
}
#line 29
# 49 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static __inline void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$set(void )
#line 49
{
  GPIO_TypeDef *port = (GPIO_TypeDef *)1073875968U;

  GPIO_SetBits(port, 4096U);
}

# 29 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$set(void ){
#line 29
  /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$set();
#line 29
}
#line 29
# 90 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static __inline void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$makeOutput(void )
#line 90
{
  GPIO_TypeDef *port = (GPIO_TypeDef *)1073875968U;

  GPIO_InitTypeDef gpioi = { 
  .GPIO_Pin = (uint32_t )1 << 8192U, 
  .GPIO_Speed = GPIO_Speed_100MHz, 
  .GPIO_Mode = GPIO_Mode_OUT, 
  .GPIO_PuPd = GPIO_PuPd_NOPULL, 
  .GPIO_OType = GPIO_OType_PP };

  GPIO_Init(port, &gpioi);
}

# 35 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$makeOutput(void ){
#line 35
  /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$makeOutput();
#line 35
}
#line 35
# 90 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static __inline void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$makeOutput(void )
#line 90
{
  GPIO_TypeDef *port = (GPIO_TypeDef *)1073875968U;

  GPIO_InitTypeDef gpioi = { 
  .GPIO_Pin = (uint32_t )1 << 4096U, 
  .GPIO_Speed = GPIO_Speed_100MHz, 
  .GPIO_Mode = GPIO_Mode_OUT, 
  .GPIO_PuPd = GPIO_PuPd_NOPULL, 
  .GPIO_OType = GPIO_OType_PP };

  GPIO_Init(port, &gpioi);
}

# 35 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$makeOutput(void ){
#line 35
  /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$makeOutput();
#line 35
}
#line 35
# 45 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();

    LedsP$Led0$set();
    LedsP$Led1$set();
  }

  return SUCCESS;
}

# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LedsP$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 43 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/McuSleepC.nc"
static inline error_t McuSleepC$McuSleepInit$init(void )
#line 43
{
#line 62
  return SUCCESS;
}

# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$McuSleepInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = McuSleepC$McuSleepInit$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 39 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/platforms/stm32f4/MoteClockP.nc"
static inline error_t MoteClockP$MoteClockInit$init(void )
{


  SystemInit();
#line 97
  return SUCCESS;
}

# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteClockInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = MoteClockP$MoteClockInit$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 46 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/platforms/stm32f4/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 46
{

  GPIO_InitTypeDef GPIO_InitStructure;





  RCC_AHB1PeriphClockCmd((uint32_t )0x00000008, ENABLE);

  GPIO_InitStructure.GPIO_Pin = (uint16_t )0xFFFF;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;









  GPIO_Init((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0000), &GPIO_InitStructure);
  ((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0000))->ODR = 0;
  GPIO_Init((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0400), &GPIO_InitStructure);
  ((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0400))->ODR = 0;
  GPIO_Init((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0800), &GPIO_InitStructure);
  ((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0800))->ODR = 0;
  GPIO_Init((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0C00), &GPIO_InitStructure);
  ((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0C00))->ODR = 0;

  * (volatile unsigned long *)0xE000ED14 = * (volatile unsigned long *)0xE000ED14 | 0x200;


  PlatformP$MoteClockInit$init();
  PlatformP$McuSleepInit$init();






  PlatformP$MoteInit$init();


  return SUCCESS;
}

# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP$PlatformInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 54 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static bool RealMainP$Scheduler$runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP$Scheduler$runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 99 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
inline static void STM32Milli32TIMC$set_alarm_interval(uint32_t interval)
{
  (
  (TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000))->CNT = 0;
  ((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000))->ARR = (uint16_t )interval - 1;
}

static inline void STM32Milli32TIMC$enableInterrupt(void )
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001, ENABLE);
      TIM_Cmd((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), ENABLE);
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
  STM32Milli32TIMC$running = TRUE;
}

#line 208
static inline uint32_t STM32Milli32TIMC$Alarm$getAlarm(void )
{
  return STM32Milli32TIMC$alarm;
}

# 105 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = STM32Milli32TIMC$Alarm$getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 201 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
static inline uint32_t STM32Milli32TIMC$Alarm$getNow(void )
{
  uint32_t c;

#line 204
  c = ((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00))->CNT;
  return c;
}

# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = STM32Milli32TIMC$Alarm$getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$getNow();
}

# 125 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 128 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$fired(void )
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$fireTimers(/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$getNow());
}

# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$fired(void ){
#line 72
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$fired();
#line 72
}
#line 72
# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static inline void /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$toggle(void )
#line 62
{
  GPIO_TypeDef *port = (GPIO_TypeDef *)1073875968U;





  GPIO_ToggleBits(port, 8192U);
}

# 31 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$toggle(void ){
#line 31
  /*HplSTM32GeneralIOC.PortD.Bit13*/HplSTM32GeneralIOPinP$61$IO$toggle();
#line 31
}
#line 31
# 88 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1Toggle(void )
#line 88
{
  LedsP$Led1$toggle();
  ;
#line 90
  ;
}

# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC$Leds$led1Toggle(void ){
#line 72
  LedsP$Leds$led1Toggle();
#line 72
}
#line 72
# 65 "BlinkC.nc"
static inline void BlinkC$Timer1$fired(void )
{
  ;

  BlinkC$Leds$led1Toggle();
}

# 193 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$default$fired(uint8_t num)
{
}

# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$fired(uint8_t arg_0x2b430d39d020){
#line 72
  switch (arg_0x2b430d39d020) {
#line 72
    case 0U:
#line 72
      BlinkC$Timer1$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$default$fired(arg_0x2b430d39d020);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 86 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 147 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
static inline void STM32Milli32TIMC$Alarm$stop(void )
{
  STM32Milli32TIMC$disableInterrupt();
}

# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$stop(void ){
#line 62
  STM32Milli32TIMC$Alarm$stop();
#line 62
}
#line 62
# 60 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$stop();
}

# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$stop();
#line 67
}
#line 67
# 92 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$size_type dt){
#line 92
  STM32Milli32TIMC$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$start(t0, dt, TRUE);
}

# 118 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$startOneShotAt(t0, dt);
#line 118
}
#line 118
# 204 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
static inline uint16_t STM32Micro16TIMC$Alarm$getNow(void )
{
  uint16_t c;

#line 207
  c = ((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800))->CNT;
  return c;
}






static inline uint16_t STM32Micro16TIMC$Counter$get(void )
{
  return STM32Micro16TIMC$Alarm$getNow();
}

# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$get(void ){
#line 53
  unsigned short __nesc_result;
#line 53

#line 53
  __nesc_result = STM32Micro16TIMC$Counter$get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 221 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
static inline bool STM32Micro16TIMC$Counter$isOverflowPending(void )
{
  return TIM_GetITStatus((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800), (uint16_t )0x0001);
}

# 60 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static bool /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = STM32Micro16TIMC$Counter$isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 109 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
static inline void STM32Micro16TIMC$enableInterrupt(void )
{
  /* atomic removed: atomic calls only */
  {
    TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001, ENABLE);
    TIM_Cmd((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), ENABLE);
  }
  STM32Micro16TIMC$running = TRUE;
}

#line 103
inline static void STM32Micro16TIMC$set_alarm_interval(uint16_t interval)
#line 103
{
  (
  (TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400))->CNT = 1;
  ((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400))->ARR = (uint16_t )interval - 1;
}

#line 160
static inline void STM32Micro16TIMC$Alarm$startAt(uint16_t t0, uint16_t dt)
{

  uint16_t interval;

#line 164
  STM32Micro16TIMC$disableInterrupt();
  {

    uint16_t now = STM32Micro16TIMC$Alarm$getNow();
    uint16_t elapsed = now - t0;

#line 169
    now = ((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800))->CNT;

    if (elapsed >= dt) 
      {

        interval = 5;

        STM32Micro16TIMC$alarm = now + 5;
      }
    else 


      {
        uint16_t remaining = dt - elapsed;

#line 183
        if (remaining <= 1) 
          {
            interval = 5;

            STM32Micro16TIMC$alarm = now + 5;
          }
        else 

          {
            interval = remaining;

            STM32Micro16TIMC$alarm = now + remaining;
          }
      }


    STM32Micro16TIMC$set_alarm_interval(interval);
    STM32Micro16TIMC$enableInterrupt();
  }
}

# 92 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$size_type dt){
#line 92
  STM32Micro16TIMC$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 81 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getAlarm(void )
{

  return /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 + /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt;
}

# 105 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 76 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getNow(void )
{
  return /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$get();
}

# 98 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$getNow(void )
{
#line 86
  return /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 128 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$fireTimers(/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$fired(void ){
#line 72
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/gpio/HplSTM32GeneralIOPinP.nc"
static inline void /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$toggle(void )
#line 62
{
  GPIO_TypeDef *port = (GPIO_TypeDef *)1073875968U;





  GPIO_ToggleBits(port, 4096U);
}

# 31 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$toggle(void ){
#line 31
  /*HplSTM32GeneralIOC.PortD.Bit12*/HplSTM32GeneralIOPinP$60$IO$toggle();
#line 31
}
#line 31
# 73 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0Toggle(void )
#line 73
{
  LedsP$Led0$toggle();
  ;
#line 75
  ;
}

# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC$Leds$led0Toggle(void ){
#line 56
  LedsP$Leds$led0Toggle();
#line 56
}
#line 56
# 57 "BlinkC.nc"
static inline void BlinkC$Timer0$fired(void )
{
  ;

  BlinkC$Leds$led0Toggle();
}

# 193 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x2b430d39d020){
#line 72
  switch (arg_0x2b430d39d020) {
#line 72
    case 0U:
#line 72
      BlinkC$Timer0$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$default$fired(arg_0x2b430d39d020);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 150 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
static inline void STM32Micro16TIMC$Alarm$stop(void )
{
  STM32Micro16TIMC$disableInterrupt();
}

# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$stop(void ){
#line 62
  STM32Micro16TIMC$Alarm$stop();
#line 62
}
#line 62
# 92 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$stop(void )
{
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$stop();
}

# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$stop(void ){
#line 62
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$stop(void )
{
#line 61
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 67
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 92 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$size_type dt){
#line 92
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 118
}
#line 118
# 58 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 78 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
inline static void STM32Micro16TIMC$init_alarm_timer(void )
#line 78
{

  TIM_DeInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400));
  TIM_TimeBaseInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), &STM32Micro16TIMC$TIM_TimeBaseStructure);
}

#line 66
inline static void STM32Micro16TIMC$init_free_timer(void )
#line 66
{


  TIM_DeInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800));

  STM32Micro16TIMC$TIM_TimeBaseStructure.TIM_Period = (uint16_t )0xFFFF;

  TIM_TimeBaseInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800), &STM32Micro16TIMC$TIM_TimeBaseStructure);
  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800), (uint16_t )0x0001, ENABLE);
}









inline static void STM32Micro16TIMC$init_timer_irq(void )
#line 85
{

  NVIC_InitTypeDef conf;

  conf.NVIC_IRQChannel = TIM4_IRQn;
  conf.NVIC_IRQChannelSubPriority = 0;
  conf.NVIC_IRQChannelPreemptionPriority = 0;
  conf.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&conf);

  conf.NVIC_IRQChannel = TIM3_IRQn;
  conf.NVIC_IRQChannelSubPriority = 1;
  conf.NVIC_IRQChannelPreemptionPriority = 0;
  conf.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&conf);
}

#line 59
inline static void STM32Micro16TIMC$timer_clock_init(void )
{
  RCC_APB1PeriphClockCmd((uint32_t )0x00000004, ENABLE);
  RCC_APB1PeriphClockCmd((uint32_t )0x00000002, ENABLE);
}

#line 129
static inline error_t STM32Micro16TIMC$Init$init(void )
{
  GPIO_ResetBits((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0C00), (uint16_t )0x8000);
  STM32Micro16TIMC$timer_clock_init();

  STM32Micro16TIMC$init_timer_irq();
  STM32Micro16TIMC$init_free_timer();
  STM32Micro16TIMC$init_alarm_timer();
  /* atomic removed: atomic calls only */
  {
    STM32Micro16TIMC$alarm = 0;
  }
  TIM_Cmd((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800), ENABLE);
  return SUCCESS;
}

# 74 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
inline static void STM32Milli32TIMC$init_alarm_timer(void )
#line 74
{

  TIM_DeInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000));
  TIM_TimeBaseInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), &STM32Milli32TIMC$TIM_TimeBaseStructure);
}

#line 61
inline static void STM32Milli32TIMC$init_free_timer(void )
{


  TIM_DeInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00));

  STM32Milli32TIMC$TIM_TimeBaseStructure.TIM_Period = (uint32_t )0xFFFFFFFF;

  TIM_TimeBaseInit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00), &STM32Milli32TIMC$TIM_TimeBaseStructure);
  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00), (uint16_t )0x0001, ENABLE);
}









inline static void STM32Milli32TIMC$init_timer_irq(void )
#line 81
{

  NVIC_InitTypeDef conf;

  conf.NVIC_IRQChannel = TIM5_IRQn;
  conf.NVIC_IRQChannelSubPriority = 0;
  conf.NVIC_IRQChannelPreemptionPriority = 1;
  conf.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&conf);

  conf.NVIC_IRQChannel = TIM2_IRQn;
  conf.NVIC_IRQChannelSubPriority = 1;
  conf.NVIC_IRQChannelPreemptionPriority = 1;
  conf.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&conf);
}

#line 54
inline static void STM32Milli32TIMC$timer_clock_init(void )
{
  RCC_APB1PeriphClockCmd((uint32_t )0x00000008, ENABLE);
  RCC_APB1PeriphClockCmd((uint32_t )0x00000001, ENABLE);
}

#line 126
static inline error_t STM32Milli32TIMC$Init$init(void )
{
  GPIO_ResetBits((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0C00), (uint16_t )0x8000);
  STM32Milli32TIMC$timer_clock_init();

  STM32Milli32TIMC$init_timer_irq();
  STM32Milli32TIMC$init_free_timer();
  STM32Milli32TIMC$init_alarm_timer();
  /* atomic removed: atomic calls only */
  {
    STM32Milli32TIMC$alarm = 0;
  }
  TIM_Cmd((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00), ENABLE);
  return SUCCESS;
}

# 51 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP$SoftwareInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = STM32Milli32TIMC$Init$init();
#line 51
  __nesc_result = ecombine(__nesc_result, STM32Micro16TIMC$Init$init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 133 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$postTask();
}

static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$startTimer(num, /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$getNow(), dt, FALSE);
}

# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void BlinkC$Timer1$startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$startPeriodic(0U, dt);
#line 53
}
#line 53
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 133 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

static inline void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 53 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void BlinkC$Timer0$startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$startPeriodic(0U, dt);
#line 53
}
#line 53
# 50 "BlinkC.nc"
static inline void BlinkC$Boot$booted(void )
{
  BlinkC$Timer0$startPeriodic(3145728);
  BlinkC$Timer1$startPeriodic(1024);
}

# 49 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainP$Boot$booted(void ){
#line 49
  BlinkC$Boot$booted();
#line 49
}
#line 49
# 165 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x2b430d185cb0){
#line 64
  switch (arg_0x2b430d185cb0) {
#line 64
    case /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer:
#line 64
      /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired:
#line 64
      /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x2b430d185cb0);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 66 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/McuSleepC.nc"
static inline void McuSleepC$enable_interrupts(void )
#line 66
{

  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001, ENABLE);
  TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001, ENABLE);
}


static inline void McuSleepC$McuSleep$sleep(void )
#line 73
{
#line 113
  McuSleepC$enable_interrupts();
#line 167
  return;
}

# 59 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP$McuSleep$sleep(void ){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 72
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void )
{

  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 151
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 61 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$taskLoop(void ){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 174 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$overflow(void )
{
  return;
}

# 47 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void )
{
}

# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$overflow(void ){
#line 71
  /*LocalTimeMicroC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 71
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$overflow();
#line 71
}
#line 71
# 134 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$overflow(void )
{

  {
    /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$m_upper++;


    if ((/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$m_upper & /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void STM32Micro16TIMC$Counter$overflow(void ){
#line 71
  /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$overflow();
#line 71
}
#line 71
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$fired(void )
{
#line 71
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 157 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$fired(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt == 0) 
        {


          /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$fired();
        }
      else 
        {
          /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$set_alarm();
        }
    }
#line 171
    __nesc_atomic_end(__nesc_atomic); }
}

# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void STM32Micro16TIMC$Alarm$fired(void ){
#line 67
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$fired();
#line 67
}
#line 67
# 233 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
static inline void STM32Milli32TIMC$Counter$default$overflow(void )
#line 233
{
  return;
}

# 71 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void STM32Milli32TIMC$Counter$overflow(void ){
#line 71
  STM32Milli32TIMC$Counter$default$overflow();
#line 71
}
#line 71
# 56 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$postTask();
}

# 67 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void STM32Milli32TIMC$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$fired();
#line 67
}
#line 67
# 52 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 123 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

# 63 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$fired$runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$start(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$1$Timer$fired();
}

# 157 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
static void STM32Milli32TIMC$Alarm$startAt(uint32_t t0, uint32_t dt)
{

  uint32_t interval;

#line 161
  STM32Milli32TIMC$disableInterrupt();
  {

    uint32_t now = STM32Milli32TIMC$Alarm$getNow();
    uint32_t elapsed = now - t0;

#line 166
    now = ((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00))->CNT;

    if (elapsed >= dt) 
      {

        interval = 2;

        STM32Milli32TIMC$alarm = now + 2;
      }
    else 


      {
        uint32_t remaining = dt - elapsed;

#line 180
        if (remaining <= 1) 
          {
            interval = 2;

            STM32Milli32TIMC$alarm = now + 2;
          }
        else 

          {
            interval = remaining;

            STM32Milli32TIMC$alarm = now + remaining;
          }
      }


    STM32Milli32TIMC$set_alarm_interval(interval);
    STM32Milli32TIMC$enableInterrupt();
  }
}

#line 116
static void STM32Milli32TIMC$disableInterrupt(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 118
    {
      TIM_Cmd((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), DISABLE);
      TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001, DISABLE);
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  STM32Milli32TIMC$running = FALSE;
}

# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer$fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$postTask();
}

# 160 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 162
    {
#line 162
      {
        unsigned char __nesc_temp = 
#line 162
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 162
          __nesc_atomic_end(__nesc_atomic); 
#line 162
          return __nesc_temp;
        }
      }
    }
#line 165
    __nesc_atomic_end(__nesc_atomic); }
}

# 89 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$updateFromTimer$runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$1$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 63 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static void /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$fired$runTask(void )
{
  if (/*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$start(/*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMicroC.AlarmToTimerMicro32*/AlarmToTimerC$0$Timer$fired();
}

# 142 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type t0, /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 = t0;
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt = dt;
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$set_alarm();
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
}

#line 97
static void /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$set_alarm(void )
{

  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type now = /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$get();
#line 100
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type expires;
#line 100
  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type remaining;




  expires = /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 + /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt;


  remaining = (/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$to_size_type )(expires - now);
  now = /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$Counter$get();


  if (/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 <= now) 
    {
      if (expires >= /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 && 
      expires <= now) 
        {
#line 116
          remaining = 0;
        }
    }
  else 
#line 118
    {
      if (expires >= /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 || 
      expires <= now) 
        {
#line 121
          remaining = 0;
        }
    }

  if (remaining > /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$MAX_DELAY) 
    {
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 = now + /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$MAX_DELAY;
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt = remaining - /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$MAX_DELAY;
      remaining = /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$MAX_DELAY;
    }
  else 

    {
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_t0 += /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt;
      /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$m_dt = 0;
    }

  /*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$AlarmFrom$startAt((/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_size_type )now << 0, 
  (/*HilTimerMicroC.AlarmMicro32.TransformAlarm*/TransformAlarmC$0$from_size_type )remaining << 0);
}

# 81 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$Counter$get(void )
{
  /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type rv = 0;

#line 84
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$upper_count_type high = /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$m_upper;
      /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$from_size_type low = /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$get();

#line 88
      if (/*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$CounterFrom$get();
        }
      {
        /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type high_to = high;
        /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$to_size_type low_to = low >> /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$LOW_SHIFT_RIGHT;

#line 102
        rv = (high_to << /*CounterTMicro32C.TransformCounter*/TransformCounterC$0$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 104
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 119 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
static void STM32Micro16TIMC$disableInterrupt(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    {
      TIM_Cmd((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), DISABLE);
      TIM_ITConfig((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001, DISABLE);
    }
#line 124
    __nesc_atomic_end(__nesc_atomic); }
  STM32Micro16TIMC$running = FALSE;
}

# 62 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer$fired(num);
              break;
            }
        }
    }
  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

static void /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$updateFromTimer$runTask(void )
{




  uint32_t now = /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMicroC.VirtTimersMicro32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 241 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Micro16TIMC.nc"
  void TIM4_IRQHandler(void )
#line 241
{

  static uint32_t j = 0;

#line 244
  if (TIM_GetFlagStatus((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800), (uint16_t )0x0001) != RESET) {

      j++;
      if (j == 1) {
          GPIO_SetBits((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0C00), (uint16_t )0x8000);
        }
      (
      (TIM_TypeDef *)((uint32_t )0x40000000 + 0x0800))->SR = (uint16_t )~ (uint16_t )0x0001;
      STM32Micro16TIMC$Counter$overflow();
    }
}



  void TIM3_IRQHandler(void )
{
  if (TIM_GetFlagStatus((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001) != RESET) {

      STM32Micro16TIMC$Alarm$stop();
      STM32Micro16TIMC$Alarm$fired();
    }
  TIM_ClearITPendingBit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0400), (uint16_t )0x0001);
}

# 238 "/home/taoli/workspace/PortingTinyOs/tinyos-2.x/tos/chips/stm32f4/timer/STM32Milli32TIMC.nc"
  void TIM5_IRQHandler(void )
#line 238
{

  static uint32_t j = 0;

#line 241
  if (TIM_GetFlagStatus((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00), (uint16_t )0x0001) != RESET) {

      j++;
      if (j == 1) {
          GPIO_SetBits((GPIO_TypeDef *)((uint32_t )0x40000000 + 0x00020000 + 0x0C00), (uint16_t )0x8000);
        }
      (
      (TIM_TypeDef *)((uint32_t )0x40000000 + 0x0C00))->SR = (uint16_t )~ (uint16_t )0x0001;
      STM32Milli32TIMC$Counter$overflow();
    }
}



  void TIM2_IRQHandler(void )
{
  if (TIM_GetFlagStatus((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001) != RESET) {

      STM32Milli32TIMC$Alarm$stop();
      STM32Milli32TIMC$Alarm$fired();
    }
  TIM_ClearITPendingBit((TIM_TypeDef *)((uint32_t )0x40000000 + 0x0000), (uint16_t )0x0001);
}

