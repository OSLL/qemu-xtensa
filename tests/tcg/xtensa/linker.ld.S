#include "core-isa.h"

#ifndef XCHAL_VECBASE_RESET_VADDR
#define XCHAL_VECBASE_RESET_VADDR XCHAL_WINDOW_VECTORS_VADDR
#define XCHAL_WINDOW_OF4_VECOFS   0x00000000
#define XCHAL_WINDOW_UF4_VECOFS   0x00000040
#define XCHAL_WINDOW_OF8_VECOFS   0x00000080
#define XCHAL_WINDOW_UF8_VECOFS   0x000000C0
#define XCHAL_WINDOW_OF12_VECOFS  0x00000100
#define XCHAL_WINDOW_UF12_VECOFS  0x00000140
#endif

#define VECTORS_RESERVED_SIZE 0x1000
#define STACK_SIZE 0x00100000

#if XCHAL_HAVE_BE
OUTPUT_FORMAT("elf32-xtensa-be")
#else
OUTPUT_FORMAT("elf32-xtensa-le")
#endif
ENTRY(_start)

SECTIONS
{
    .init XCHAL_RESET_VECTOR_VADDR :
    {
        *(.init)
        *(.init.*)
    }

#if XCHAL_HAVE_WINDOWED
    .vector.window XCHAL_WINDOW_VECTORS_VADDR :
    {
    . = XCHAL_WINDOW_OF4_VECOFS;
        *(.vector.window_overflow_4)
    . = XCHAL_WINDOW_UF4_VECOFS;
        *(.vector.window_underflow_4)
    . = XCHAL_WINDOW_OF8_VECOFS;
        *(.vector.window_overflow_8)
    . = XCHAL_WINDOW_UF8_VECOFS;
        *(.vector.window_underflow_8)
    . = XCHAL_WINDOW_OF12_VECOFS;
        *(.vector.window_overflow_12)
    . = XCHAL_WINDOW_UF12_VECOFS;
        *(.vector.window_underflow_12)
    }
#endif
#if XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI >= 2
    .vector.level2 XCHAL_INTLEVEL2_VECTOR_VADDR :
    {
        *(.vector.level2)
    }
#endif
#if XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI >= 3
    .vector.level3 XCHAL_INTLEVEL3_VECTOR_VADDR :
    {
        *(.vector.level3)
    }
#endif
#if XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI >= 4
    .vector.level4 XCHAL_INTLEVEL4_VECTOR_VADDR :
    {
        *(.vector.level4)
    }
#endif
#if XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI >= 5
    .vector.level5 XCHAL_INTLEVEL5_VECTOR_VADDR :
    {
        *(.vector.level5)
    }
#endif
#if XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI >= 6
    .vector.level6 XCHAL_INTLEVEL6_VECTOR_VADDR :
    {
        *(.vector.level6)
    }
#endif
#if XCHAL_NUM_INTLEVELS + XCHAL_HAVE_NMI >= 7
    .vector.level7 XCHAL_INTLEVEL7_VECTOR_VADDR :
    {
        *(.vector.level7)
    }
#endif
    .vector.kernel XCHAL_KERNEL_VECTOR_VADDR :
    {
        *(.vector.kernel)
    }
    .vector.user XCHAL_USER_VECTOR_VADDR :
    {
        *(.vector.user)
    }
    .vector.double XCHAL_DOUBLEEXC_VECTOR_VADDR :
    {
        *(.vector.double)
    }

    .vector.text XCHAL_VECBASE_RESET_VADDR + VECTORS_RESERVED_SIZE :
    {
        *(.vector.window_overflow_4.*)
        *(.vector.window_underflow_4.*)
        *(.vector.window_overflow_8.*)
        *(.vector.window_underflow_8.*)
        *(.vector.window_overflow_12.*)
        *(.vector.window_underflow_12.*)

        *(.vector.level2.*)
        *(.vector.level3.*)
        *(.vector.level4.*)
        *(.vector.level5.*)
        *(.vector.level6.*)
        *(.vector.level7.*)

        *(.vector.kernel.*)
        *(.vector.user.*)
        *(.vector.double.*)
    }

    .text :
    {
        _ftext = .;
        *(.text .stub .text.* .gnu.linkonce.t.* .literal .literal.*)
        _etext = .;
    }

    . = DATA_SEGMENT_ALIGN(0x1000, 0x1000);

    .data :
    {
        . = ALIGN(4);
        _fdata = .;
        *(.data .data.* .gnu.linkonce.d.*)
        *(.data1)
        _gp = ALIGN(16);
        *(.sdata .sdata.* .gnu.linkonce.s.*)
        _edata = .;
    }

    .bss :
    {
        . = ALIGN(4);
        _fbss = .;
        *(.dynsbss)
        *(.sbss .sbss.* .gnu.linkonce.sb.*)
        *(.scommon)
        *(.dynbss)
        *(.bss .bss.* .gnu.linkonce.b.*)
        *(COMMON)
        _ebss = .;
        _end = .;
    }

    . = DATA_SEGMENT_END(.);
    _fstack = ALIGN(. + STACK_SIZE, 16);
}
