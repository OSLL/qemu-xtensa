/*
 * Copyright (c) 2014, Max Filippov, Open Source and Linux Lab.
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

#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "sysemu/block-backend.h"
#include "sysemu/char.h"
#include "sysemu/device_tree.h"
#include "qemu/error-report.h"
#include "bootparam.h"

#define DEFINE_BITS(prefix, reg, field, shift, len) \
    prefix##_##reg##_##field##_SHIFT = shift, \
    prefix##_##reg##_##field##_LEN = len, \
    prefix##_##reg##_##field = ((~0U >> (32 - (len))) << (shift))

/* Serial */

enum {
    ESP8266_UART_FIFO,
    ESP8266_UART_INT_RAW,
    ESP8266_UART_INT_ST,
    ESP8266_UART_INT_ENA,
    ESP8266_UART_INT_CLR,
    ESP8266_UART_CLKDIV,
    ESP8266_UART_AUTOBAUD,
    ESP8266_UART_STATUS,
    ESP8266_UART_CONF0,
    ESP8266_UART_CONF1,
    ESP8266_UART_LOWPULSE,
    ESP8266_UART_HIGHPULSE,
    ESP8266_UART_RXD_CNT,
    ESP8266_UART_DATE = 0x78 / 4,
    ESP8266_UART_ID,
    ESP8266_UART_MAX,
};

#define ESP8266_UART_BITS(reg, field, shift, len) \
    DEFINE_BITS(ESP8266_UART, reg, field, shift, len)

enum {
    ESP8266_UART_BITS(INT, RXFIFO_FULL, 0, 1),
    ESP8266_UART_BITS(INT, TXFIFO_EMPTY, 1, 1),
    ESP8266_UART_BITS(INT, RXFIFO_OVF, 4, 1),
};

enum {
    ESP8266_UART_BITS(CONF0, LOOPBACK, 14, 1),
    ESP8266_UART_BITS(CONF0, RXFIFO_RST, 17, 1),
};

enum {
    ESP8266_UART_BITS(CONF1, RXFIFO_FULL, 0, 7),
};

#define ESP8266_UART_GET(s, _reg, _field) \
    extract32(s->reg[ESP8266_UART_##_reg], \
              ESP8266_UART_##_reg##_##_field##_SHIFT, \
              ESP8266_UART_##_reg##_##_field##_LEN)

#define ESP8266_UART_FIFO_SIZE 0x80
#define ESP8266_UART_FIFO_MASK 0x7f

typedef struct Esp8266SerialState {
    MemoryRegion iomem;
    CharDriverState *chr;
    qemu_irq irq;

    unsigned rx_first;
    unsigned rx_last;
    uint8_t rx[ESP8266_UART_FIFO_SIZE];
    uint32_t reg[ESP8266_UART_MAX];
} Esp8266SerialState;

static unsigned esp8266_serial_rx_fifo_size(Esp8266SerialState *s)
{
    return (s->rx_last - s->rx_first) & ESP8266_UART_FIFO_MASK;
}

static int esp8266_serial_can_receive(void *opaque)
{
    Esp8266SerialState *s = opaque;

    return esp8266_serial_rx_fifo_size(s) != ESP8266_UART_FIFO_SIZE - 1;
}

static void esp8266_serial_irq_update(Esp8266SerialState *s)
{
    s->reg[ESP8266_UART_INT_ST] |= s->reg[ESP8266_UART_INT_RAW];
    if (s->reg[ESP8266_UART_INT_ST] & s->reg[ESP8266_UART_INT_ENA]) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void esp8266_serial_rx_irq_update(Esp8266SerialState *s)
{
    if (esp8266_serial_rx_fifo_size(s) >= ESP8266_UART_GET(s, CONF1, RXFIFO_FULL)) {
        s->reg[ESP8266_UART_INT_RAW] |= ESP8266_UART_INT_RXFIFO_FULL;
    } else {
        s->reg[ESP8266_UART_INT_RAW] &= ~ESP8266_UART_INT_RXFIFO_FULL;
    }

    if (!esp8266_serial_can_receive(s)) {
        s->reg[ESP8266_UART_INT_RAW] |= ESP8266_UART_INT_RXFIFO_OVF;
    } else {
        s->reg[ESP8266_UART_INT_RAW] &= ~ESP8266_UART_INT_RXFIFO_OVF;
    }
    esp8266_serial_irq_update(s);
}

static uint64_t esp8266_serial_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    Esp8266SerialState *s = opaque;

    if ((addr & 3) || size != 4) {
        return 0;
    }

    switch (addr / 4) {
    case ESP8266_UART_STATUS:
        return esp8266_serial_rx_fifo_size(s);

    case ESP8266_UART_FIFO:
        if (esp8266_serial_rx_fifo_size(s)) {
            uint8_t r = s->rx[s->rx_first++];

            s->rx_first &= ESP8266_UART_FIFO_MASK;
            esp8266_serial_rx_irq_update(s);
            return r;
        } else {
            return 0;
        }
    case ESP8266_UART_INT_RAW:
    case ESP8266_UART_INT_ST:
    case ESP8266_UART_INT_ENA:
    case ESP8266_UART_INT_CLR:
    case ESP8266_UART_CLKDIV:
    case ESP8266_UART_AUTOBAUD:
    case ESP8266_UART_CONF0:
    case ESP8266_UART_CONF1:
    case ESP8266_UART_LOWPULSE:
    case ESP8266_UART_HIGHPULSE:
    case ESP8266_UART_RXD_CNT:
    case ESP8266_UART_DATE:
    case ESP8266_UART_ID:
        return s->reg[addr / 4];

    default:
        qemu_log("%s: unexpected read @0x%"HWADDR_PRIx"\n", __func__, addr);
        break;
    }
    return 0;
}

static void esp8266_serial_receive(void *opaque, const uint8_t *buf, int size)
{
    Esp8266SerialState *s = opaque;
    unsigned i;

    for (i = 0; i < size && esp8266_serial_can_receive(s); ++i) {
        s->rx[s->rx_last++] = buf[i];
        s->rx_last &= ESP8266_UART_FIFO_MASK;
    }

    esp8266_serial_rx_irq_update(s);
}

static void esp8266_serial_ro(Esp8266SerialState *s, hwaddr addr,
                              uint64_t val, unsigned size)
{
}

static void esp8266_serial_tx(Esp8266SerialState *s, hwaddr addr,
                              uint64_t val, unsigned size)
{
    if (ESP8266_UART_GET(s, CONF0, LOOPBACK)) {
        if (esp8266_serial_can_receive(s)) {
            uint8_t buf[] = { (uint8_t)val };

            esp8266_serial_receive(s, buf, 1);
        }

    } else if (s->chr) {
        uint8_t buf[1] = { val };
        qemu_chr_fe_write(s->chr, buf, 1);
        s->reg[ESP8266_UART_INT_RAW] |= ESP8266_UART_INT_TXFIFO_EMPTY;
        esp8266_serial_irq_update(s);
    }
}

static void esp8266_serial_int_ena(Esp8266SerialState *s, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    s->reg[ESP8266_UART_INT_ENA] = val & 0x1ff;
    esp8266_serial_irq_update(s);
}

static void esp8266_serial_int_clr(Esp8266SerialState *s, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    s->reg[ESP8266_UART_INT_ST] &= ~val & 0x1ff;
    esp8266_serial_irq_update(s);
}

static void esp8266_serial_set_conf0(Esp8266SerialState *s, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    s->reg[ESP8266_UART_CONF0] = val & 0xffffff;
    if (ESP8266_UART_GET(s, CONF0, RXFIFO_RST)) {
        s->rx_first = s->rx_last = 0;
        esp8266_serial_rx_irq_update(s);
    }
}

static void esp8266_serial_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    Esp8266SerialState *s = opaque;
    static void (* const handler[])(Esp8266SerialState *s, hwaddr addr,
                                    uint64_t val, unsigned size) = {
        [ESP8266_UART_FIFO] = esp8266_serial_tx,
        [ESP8266_UART_INT_RAW] = esp8266_serial_ro,
        [ESP8266_UART_INT_ST] = esp8266_serial_ro,
        [ESP8266_UART_INT_ENA] = esp8266_serial_int_ena,
        [ESP8266_UART_INT_CLR] = esp8266_serial_int_clr,
        [ESP8266_UART_STATUS] = esp8266_serial_ro,
        [ESP8266_UART_CONF0] = esp8266_serial_set_conf0,
        [ESP8266_UART_LOWPULSE] = esp8266_serial_ro,
        [ESP8266_UART_HIGHPULSE] = esp8266_serial_ro,
        [ESP8266_UART_RXD_CNT] = esp8266_serial_ro,
    };

    if ((addr & 3) || size != 4 || addr / 4 >= ARRAY_SIZE(handler)) {
        return;
    }

    if (handler[addr / 4]) {
        handler[addr / 4](s, addr, val, size);
    } else {
        s->reg[addr / 4] = val;
    }
}

static void esp8266_serial_reset(void *opaque)
{
    Esp8266SerialState *s = opaque;

    memset(s->reg, 0, sizeof(s->reg));

    s->reg[ESP8266_UART_CLKDIV] = 0x2b6;
    s->reg[ESP8266_UART_AUTOBAUD] = 0x1000;
    s->reg[ESP8266_UART_CONF0] = 0x1c;
    s->reg[ESP8266_UART_CONF1] = 0x6060;
    s->reg[ESP8266_UART_LOWPULSE] = 0xfffff;
    s->reg[ESP8266_UART_HIGHPULSE] = 0xfffff;
    s->reg[ESP8266_UART_DATE] = 0x62000;
    s->reg[ESP8266_UART_ID] = 0x500;

    esp8266_serial_irq_update(s);
}

static const MemoryRegionOps esp8266_serial_ops = {
    .read = esp8266_serial_read,
    .write = esp8266_serial_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Esp8266SerialState *esp8266_serial_init(MemoryRegion *address_space,
                                               hwaddr base, const char *name,
                                               qemu_irq irq,
                                               CharDriverState *chr)
{
    Esp8266SerialState *s = g_malloc(sizeof(Esp8266SerialState));

    s->chr = chr;
    s->irq = irq;
    qemu_chr_add_handlers(s->chr, esp8266_serial_can_receive,
                          esp8266_serial_receive, NULL, s);
    memory_region_init_io(&s->iomem, NULL, &esp8266_serial_ops, s,
                          name, 0x100);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_serial_reset, s);
    return s;
}

/* GPIO */

#define ESP8266_GPIO_STRAP_SD_START     (0x4 << 16)
#define ESP8266_GPIO_STRAP_FLASH_START  (0x3 << 16)
#define ESP8266_GPIO_STRAP_UART_START   (0x2 << 16)

typedef struct Esp8266GpioState {
    MemoryRegion iomem;
    uint32_t in;
} Esp8266GpioState;

static uint32_t user_entry;

static uint64_t esp8266_gpio_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    Esp8266GpioState *s = opaque;

    switch (addr) {
    case 0x18: /*in*/
        return s->in;

    case 0x80: /*cheat: user entry*/
        return user_entry;

    default:
        fprintf(stderr, "%s, %x\n", __func__, (uint32_t)addr);
        break;
    }
    return 0;
}

static void esp8266_gpio_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    //Esp8266GpioState *s = opaque;

    switch (addr) {
    }
}

static const MemoryRegionOps esp8266_gpio_ops = {
    .read = esp8266_gpio_read,
    .write = esp8266_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_gpio_reset(void *opaque)
{
    Esp8266GpioState *s = opaque;

    s->in = ESP8266_GPIO_STRAP_FLASH_START | (0x7 << 29);
}

static Esp8266GpioState *esp8266_gpio_init(MemoryRegion *address_space,
                                           hwaddr base)
{
    Esp8266GpioState *s = g_malloc(sizeof(Esp8266GpioState));

    memory_region_init_io(&s->iomem, NULL, &esp8266_gpio_ops, s,
                          "esp8266.gpio", 0x100);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_gpio_reset, s);
    return s;

}

/* RTC */

typedef struct Esp8266RtcState {
    MemoryRegion iomem;
} Esp8266RtcState;

static uint64_t esp8266_rtc_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    //Esp8266RtcState *s = opaque;

    switch (addr) {
    case 0x08: /*?*/
        return 0;

    case 0x14: /*Reset reason: bits 0..3*/
        return 4;

    case 0x18: /*?*/
        return 5;

    default:
        fprintf(stderr, "%s, %x\n", __func__, (uint32_t)addr);
        break;
    }
    return 0;
}

static void esp8266_rtc_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    //Esp8266RtcState *s = opaque;

    switch (addr) {
    }
}

static const MemoryRegionOps esp8266_rtc_ops = {
    .read = esp8266_rtc_read,
    .write = esp8266_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_rtc_reset(void *opaque)
{
    //Esp8266RtcState *s = opaque;
}

static Esp8266RtcState *esp8266_rtc_init(MemoryRegion *address_space,
                                           hwaddr base)
{
    Esp8266RtcState *s = g_malloc(sizeof(Esp8266RtcState));

    memory_region_init_io(&s->iomem, NULL, &esp8266_rtc_ops, s,
                          "esp8266.rtc", 0x100);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_rtc_reset, s);
    return s;

}

static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}


static void esp8266_reset(void *opaque)
{
    XtensaCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static uint64_t esp8266_io_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    return 0;
}

static void esp8266_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
}

static const MemoryRegionOps esp8266_io_ops = {
    .read = esp8266_io_read,
    .write = esp8266_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void xtensa_esp8266_init(MachineState *machine)
{
#ifdef TARGET_WORDS_BIGENDIAN
    int be = 1;
#else
    int be = 0;
#endif
    const char *rom_filename = "esp8266.rom";
    MemoryRegion *system_memory = get_system_memory();
    XtensaCPU *cpu = NULL;
    CPUXtensaState *env = NULL;
    MemoryRegion *ram, *system_io;
    QemuOpts *machine_opts = qemu_get_machine_opts();
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = qemu_opt_get(machine_opts, "kernel");
    int n;

    if (!cpu_model) {
        cpu_model = "lx106";
    }

    for (n = 0; n < smp_cpus; n++) {
        cpu = cpu_xtensa_init(cpu_model);
        if (cpu == NULL) {
            error_report("unable to find CPU definition '%s'\n",
                         cpu_model);
            exit(EXIT_FAILURE);
        }
        env = &cpu->env;

        env->sregs[PRID] = n;
        qemu_register_reset(esp8266_reset, cpu);
        xtensa_select_static_vectors(env, 1);
        /* Need MMU initialized prior to ELF loading,
         * so that ELF gets loaded into virtual addresses
         */
        cpu_reset(CPU(cpu));
    }

    ram = g_malloc(sizeof(*ram));
    memory_region_init_ram(ram, NULL, "esp8266.dram", 0x00400000,
                           &error_abort);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(system_memory, 0x3ff00000, ram);

    system_io = g_malloc(sizeof(*system_io));
    memory_region_init_io(system_io, NULL, &esp8266_io_ops, NULL, "esp8266.io",
                          256 * 1024 * 1024);

    memory_region_add_subregion(system_memory, 0x60000000, system_io);

    if (!serial_hds[0]) {
        serial_hds[0] = qemu_chr_new("serial0", "null", NULL);
    }
    esp8266_serial_init(system_io, 0x00000000, "esp8266.uart0",
                        xtensa_get_extint(env, 5), serial_hds[0]);
    esp8266_gpio_init(system_io, 0x00000300);
    esp8266_rtc_init(system_io, 0x00000700);


    /* Use presence of kernel file name as 'boot from SRAM' switch. */
    if (kernel_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(kernel_filename, translate_phys_addr, cpu,
                &elf_entry, &elf_lowaddr, NULL, be, ELF_MACHINE, 0);
        if (success > 0) {
            user_entry = elf_entry;
        } else {
            error_report("could not load kernel '%s'\n",
                         kernel_filename);
            exit(EXIT_FAILURE);
        }
        rom_filename = "esp8266-call-user.rom";
    }

    rom_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, rom_filename);
    if (!rom_filename ||
        load_image_targphys(rom_filename, 0x40000000, 65536) < 0) {
        error_report("unable to load ROM image '%s'\n", rom_filename);
        exit(EXIT_FAILURE);
    }
}

static QEMUMachine xtensa_esp8266_machine = {
    .name = "esp8266",
    .desc = "ESP8266 (" XTENSA_DEFAULT_CPU_MODEL ")",
    .init = xtensa_esp8266_init,
    .max_cpus = 1,
};

static void xtensa_lx_machines_init(void)
{
    qemu_register_machine(&xtensa_esp8266_machine);
}

machine_init(xtensa_lx_machines_init);
