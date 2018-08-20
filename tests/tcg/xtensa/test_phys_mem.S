#include "macros.inc"

test_suite phys_mem

.purgem test_init

.macro test_init
    movi    a2, 0xc0000003 /* PPN */
    movi    a3, 0xc0000004 /* VPN */
    wdtlb   a2, a3
    witlb   a2, a3
.endm

test inst_fetch_no_phys
    set_vector kernel, 1f

    movi    a2, 0xc0000000
    jx      a2
1:
    movi    a2, 0xc0000000
    rsr     a3, excvaddr
    assert  eq, a2, a3
    rsr     a3, epc1
    assert  eq, a2, a3
    rsr     a3, exccause
    movi    a2, 14
    assert  eq, a2, a3
test_end

test read_no_phys
    set_vector kernel, 2f

    movi    a2, 0xc0000000
1:
    l32i    a3, a2, 0
    test_fail
2:
    movi    a2, 0xc0000000
    rsr     a3, excvaddr
    assert  eq, a2, a3
    movi    a2, 1b
    rsr     a3, epc1
    assert  eq, a2, a3
    rsr     a3, exccause
    movi    a2, 15
    assert  eq, a2, a3
test_end

test write_no_phys
    set_vector kernel, 2f

    movi    a2, 0xc0000000
1:
    s32i    a3, a2, 0
    test_fail
2:
    movi    a2, 0xc0000000
    rsr     a3, excvaddr
    assert  eq, a2, a3
    movi    a2, 1b
    rsr     a3, epc1
    assert  eq, a2, a3
    rsr     a3, exccause
    movi    a2, 15
    assert  eq, a2, a3
test_end

test_suite_end
