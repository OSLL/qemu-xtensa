/*
 * Copyright (c) 2011, Max Filippov, Open Source and Linux Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source and Linux Lab nor the
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

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stddef.h>
#include "exec.h"
#include "qemu-log.h"

enum {
    TARGET_SYS_exit = 1,
    TARGET_SYS_read = 3,
    TARGET_SYS_write = 4,
    TARGET_SYS_open = 5,
    TARGET_SYS_close = 6,

    TARGET_SYS_argc = 1000,
    TARGET_SYS_argv_sz = 1001,
    TARGET_SYS_argv = 1002,
    TARGET_SYS_memset = 1004,
};

void simcall(CPUState *env)
{
    uint32_t *regs = env->regs;

    switch (regs[2]) {
    case TARGET_SYS_exit:
        qemu_log("exit(%d) simcall\n", regs[3]);
        exit(regs[3]);
        break;

    case TARGET_SYS_read:
        {
            target_phys_addr_t len = regs[5];
            void *buf = cpu_physical_memory_map(regs[4], &len, 1);

            if (buf) {
                regs[2] = read(regs[3], buf, len);
                regs[3] = errno;
                cpu_physical_memory_unmap(buf, len, 1, len);
            } else {
                regs[2] = -1;
                regs[3] = EINVAL;
            }
        }
        break;

    case TARGET_SYS_write:
        {
            target_phys_addr_t len = regs[5];
            void *buf = cpu_physical_memory_map(regs[4], &len, 0);

            if (buf) {
                regs[2] = write(regs[3], buf, len);
                regs[3] = errno;
                cpu_physical_memory_unmap(buf, len, 0, len);
            } else {
                regs[2] = -1;
                regs[3] = EINVAL;
            }
        }
        break;

    case TARGET_SYS_open:
        {
            char name[1024];
            int rc;
            int i;

            for (i = 0; i < ARRAY_SIZE(name); ++i) {
                rc = cpu_memory_rw_debug(
                        env, regs[3] + i, (uint8_t *)name + i, 1, 0);
                if (rc != 0 || name[i] == 0) {
                    break;
                }
            }

            if (rc == 0 && i < ARRAY_SIZE(name)) {
                regs[2] = open(name, regs[4], regs[5]);
                regs[3] = errno;
            } else {
                regs[2] = -1;
                regs[3] = EINVAL;
            }
        }
        break;

    case TARGET_SYS_close:
        if (regs[3] < 3) {
            regs[2] = regs[3] = 0;
        } else {
            regs[2] = close(regs[3]);
            regs[3] = errno;
        }
        break;

    case TARGET_SYS_argc:
        regs[2] = 1;
        regs[3] = 0;
        break;

    case TARGET_SYS_argv_sz:
        regs[2] = 128;
        regs[3] = 0;
        break;

    case TARGET_SYS_argv:
        {
            struct Argv {
                uint32_t argptr[2];
                char text[120];
            } argv = {
                {0, 0},
                "test"
            };

            argv.argptr[0] = tswap32(regs[3] + offsetof(struct Argv, text));
            cpu_memory_rw_debug(
                    env, regs[3], (uint8_t *)&argv, sizeof(argv), 1);
        }
        break;

    case TARGET_SYS_memset:
        {
            uint32_t base = regs[3];
            uint32_t sz = regs[5];

            while (sz) {
                target_phys_addr_t len = sz;
                void *buf = cpu_physical_memory_map(base, &len, 1);

                if (buf && len) {
                    memset(buf, regs[4], len);
                    cpu_physical_memory_unmap(buf, len, 1, len);
                } else {
                    len = 1;
                }
                base += len;
                sz -= len;
            }
            regs[2] = regs[3];
            regs[3] = 0;
        }
        break;

    default:
        qemu_log("%s(%d): not implemented\n", __func__, regs[2]);
        break;
    }
}
