/*
 * Copyright (c) 2011, Max Filippov, Motorola Solutions, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Motorola Solutions nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stddef.h>

enum {
    SYS_exit = 1,
    SYS_read = 3,
    SYS_write = 4,
    SYS_open = 5,
    SYS_close = 6,

    SYS_argc = 1000,
    SYS_argv_sz = 1001,
    SYS_argv = 1002,
    SYS_memset = 1004,
};

static void simcall(uint32_t regs[16])
{
    switch (regs[2]) {
    case SYS_exit:
        printf("exit(%d)\n", regs[3]);
        exit(regs[3]);
        break;

    case SYS_read:
        {
            target_phys_addr_t len = regs[5];
            void *buf = cpu_physical_memory_map(regs[4], &len, 1);

            if (buf) {
                regs[2] = read(regs[3], buf, len);
                regs[3] = errno;
                cpu_physical_memory_unmap(buf, len, 1, len);
            } else {
                regs[2] = -1;
                regs[3] = 0;
            }
        }
        break;

    case SYS_write:
        {
            target_phys_addr_t len = regs[5];
            void *buf = cpu_physical_memory_map(regs[4], &len, 0);

            if (buf) {
                regs[2] = write(regs[3], buf, len);
                regs[3] = errno;
                cpu_physical_memory_unmap(buf, len, 0, len);
            } else {
                regs[2] = -1;
                regs[3] = 0;
            }
        }
        break;

    case SYS_open:
        {
            target_phys_addr_t len = 1024;
            void *buf = cpu_physical_memory_map(regs[3], &len, 0);

            if (buf && strnlen((char *)buf, len) < len) {
                regs[2] = open((char *)buf, regs[4], regs[5]);
                regs[3] = errno;
            } else {
                regs[2] = -1;
                regs[3] = 0;
            }
        }
        break;

    case SYS_close:
        if (regs[3] < 3) {
            regs[2] = regs[3] = 0;
        } else {
            regs[2] = close(regs[3]);
            regs[3] = errno;
        }
        break;

    case SYS_argc:
        regs[2] = 1;
        regs[3] = 0;
        break;

    case SYS_argv_sz:
        regs[2] = 128;
        regs[3] = 0;
        break;

    case SYS_argv:
        {
            struct Argv {
                uint32_t argptr[2];
                char text[120];
            } argv = {
                {0, 0},
                "test"
            };

            argv.argptr[0] = regs[3] + offsetof(struct Argv, text);
            cpu_memory_rw_debug(
                    env, regs[3], (uint8_t *)&argv, sizeof(argv), 1);
        }
        break;

    case SYS_memset:
        {
            target_phys_addr_t len = regs[5];
            void *buf = cpu_physical_memory_map(regs[3], &len, 1);

            assert(len == regs[5]);

            if (buf) {
                memset(buf, regs[4], len);
                regs[2] = regs[3];
                regs[3] = 0;
                cpu_physical_memory_unmap(buf, len, 1, len);
            }
        }
        break;

    default:
        printf("%s(%d)\n", __func__, regs[2]);
        break;
    }
}
