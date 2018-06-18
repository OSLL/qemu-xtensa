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

#define DEBUG_LOG(...) fprintf(stderr, __VA_ARGS__)

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

    if ((addr & 3) || size != 4 || addr / 4 >= ESP8266_UART_MAX) {
        return;
    }

    if (addr / 4 < ARRAY_SIZE(handler) && handler[addr / 4]) {
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

/* SPI */

enum {
    ESP8266_SPI_FLASH_CMD,
    ESP8266_SPI_FLASH_ADDR,
    ESP8266_SPI_FLASH_CTRL,
    ESP8266_SPI_FLASH_CTRL1,
    ESP8266_SPI_FLASH_STATUS,
    ESP8266_SPI_FLASH_CTRL2,
    ESP8266_SPI_FLASH_CLOCK,
    ESP8266_SPI_FLASH_USER,
    ESP8266_SPI_FLASH_USER1,
    ESP8266_SPI_FLASH_USER2,
    ESP8266_SPI_FLASH_USER3,
    ESP8266_SPI_FLASH_PIN,
    ESP8266_SPI_FLASH_SLAVE,
    ESP8266_SPI_FLASH_SLAVE1,
    ESP8266_SPI_FLASH_SLAVE2,
    ESP8266_SPI_FLASH_SLAVE3,
    ESP8266_SPI_FLASH_C0,
    ESP8266_SPI_FLASH_C1,
    ESP8266_SPI_FLASH_C2,
    ESP8266_SPI_FLASH_C3,
    ESP8266_SPI_FLASH_C4,
    ESP8266_SPI_FLASH_C5,
    ESP8266_SPI_FLASH_C6,
    ESP8266_SPI_FLASH_C7,

    ESP8266_SPI_FLASH_EXT0 = 0x3c,
    ESP8266_SPI_FLASH_EXT1,
    ESP8266_SPI_FLASH_EXT2,
    ESP8266_SPI_FLASH_EXT3,
    ESP8266_SPI_MAX,
};

#define ESP8266_MAX_FLASH_SZ (1 << 24)

#define ESP8266_SPI_FLASH_BITS(reg, field, shift, len) \
    DEFINE_BITS(ESP8266_SPI_FLASH, reg, field, shift, len)

#define ESP8266_SPI_GET_VAL(v, _reg, _field) \
    extract32(v, \
              ESP8266_SPI_FLASH_##_reg##_##_field##_SHIFT, \
              ESP8266_SPI_FLASH_##_reg##_##_field##_LEN)

#define ESP8266_SPI_GET(s, _reg, _field) \
    extract32(s->reg[ESP8266_SPI_FLASH_##_reg], \
              ESP8266_SPI_FLASH_##_reg##_##_field##_SHIFT, \
              ESP8266_SPI_FLASH_##_reg##_##_field##_LEN)

enum {
    ESP8266_SPI_FLASH_BITS(CMD, USR, 18, 1),
    ESP8266_SPI_FLASH_BITS(CMD, CE, 22, 1),
    ESP8266_SPI_FLASH_BITS(CMD, BE, 23, 1),
    ESP8266_SPI_FLASH_BITS(CMD, SE, 24, 1),
    ESP8266_SPI_FLASH_BITS(CMD, PP, 25, 1),
    ESP8266_SPI_FLASH_BITS(CMD, WRDI, 29, 1),
    ESP8266_SPI_FLASH_BITS(CMD, WREN, 30, 1),
    ESP8266_SPI_FLASH_BITS(CMD, READ, 31, 1),
};

enum {
    ESP8266_SPI_FLASH_BITS(ADDR, OFFSET, 0, 24),
    ESP8266_SPI_FLASH_BITS(ADDR, LENGTH, 24, 8),
};

enum {
    ESP8266_SPI_FLASH_BITS(CTRL, ENABLE_AHB, 17, 1),
};

enum {
    ESP8266_SPI_FLASH_BITS(STATUS, BUSY, 0, 1),
    ESP8266_SPI_FLASH_BITS(STATUS, WRENABLE, 1, 1),
};

enum {
    ESP8266_SPI_FLASH_BITS(CLOCK, CLKCNT_L, 0, 6),
    ESP8266_SPI_FLASH_BITS(CLOCK, CLKCNT_H, 6, 6),
    ESP8266_SPI_FLASH_BITS(CLOCK, CLKCNT_N, 12, 6),
    ESP8266_SPI_FLASH_BITS(CLOCK, CLK_DIV_PRE, 18, 13),
    ESP8266_SPI_FLASH_BITS(CLOCK, CLK_EQU_SYSCLK, 31, 1),
};

enum {
    ESP8266_SPI_FLASH_BITS(USER, FLASH_MODE, 2, 1),
};

enum {
    ESP8266_SPI_FLASH_BITS(USER2, COMMAND_VALUE, 0, 16),
    ESP8266_SPI_FLASH_BITS(USER2, COMMAND_BITLEN, 28, 4),
};

typedef struct Esp8266SpiState {
    MemoryRegion iomem;
    MemoryRegion cache;
    qemu_irq irq;
    void *flash_image;

    uint32_t reg[ESP8266_SPI_MAX];
} Esp8266SpiState;

static uint64_t esp8266_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp8266SpiState *s = opaque;

    DEBUG_LOG("%s: +0x%02x: ", __func__, (uint32_t)addr);
    if (addr / 4 >= ESP8266_SPI_MAX || addr % 4 || size > 4) {
        DEBUG_LOG("0\n");
        return 0;
    }
    else if (size == 4) {
        DEBUG_LOG("0x%08x\n", s->reg[addr / 4]);
        return s->reg[addr / 4];
    }

    uint32_t ret = 0;
    memcpy(&ret, &s->reg[addr / 4], size);
    DEBUG_LOG("0x%08x\n", ret);
    return ret;
}

static void esp8266_spi_cmd(Esp8266SpiState *s, hwaddr addr,
                            uint64_t val, unsigned size)
{
    if (val & ESP8266_SPI_FLASH_CMD_READ) {
        if (ESP8266_SPI_GET(s, USER, FLASH_MODE)) {
            DEBUG_LOG("%s: READ FLASH 0x%02x@0x%08x\n",
                      __func__,
                      ESP8266_SPI_GET(s, ADDR, LENGTH),
                      ESP8266_SPI_GET(s, ADDR, OFFSET));
            memcpy(s->reg + ESP8266_SPI_FLASH_C0,
                   s->flash_image + ESP8266_SPI_GET(s, ADDR, OFFSET),
                   (ESP8266_SPI_GET(s, ADDR, LENGTH) + 3) & 0x3c);
        } else {
            DEBUG_LOG("%s: READ ?????\n", __func__);
        }
    }
    if (val & ESP8266_SPI_FLASH_CMD_WRDI) {
        s->reg[ESP8266_SPI_FLASH_STATUS] &= ~ESP8266_SPI_FLASH_STATUS_WRENABLE;
    }
    if (val & ESP8266_SPI_FLASH_CMD_WREN) {
        s->reg[ESP8266_SPI_FLASH_STATUS] |= ESP8266_SPI_FLASH_STATUS_WRENABLE;
    }
    if (val & ESP8266_SPI_FLASH_CMD_USR) {
        DEBUG_LOG("%s: TX %04x[%d bits]\n",
                  __func__,
                  ESP8266_SPI_GET(s, USER2, COMMAND_VALUE),
                  ESP8266_SPI_GET(s, USER2, COMMAND_BITLEN));
    }
    if (val & ESP8266_SPI_FLASH_CMD_SE) {
        DEBUG_LOG("%s: SECTOR ERASE @0x%08x\n",
                  __func__,
                  ESP8266_SPI_GET(s, ADDR, OFFSET) & 0xfff);
        memset(s->flash_image + (ESP8266_SPI_GET(s, ADDR, OFFSET) & ~0xfff), 0xff, 4096);
    }
    if (val & ESP8266_SPI_FLASH_CMD_BE) {
        DEBUG_LOG("%s: BLOCK ERASE @0x%08x\n",
                  __func__,
                  ESP8266_SPI_GET(s, ADDR, OFFSET) & 0xffff);
        memset(s->flash_image + (ESP8266_SPI_GET(s, ADDR, OFFSET) & ~0xffff), 0xff, 65536);
    }
    if (val & ESP8266_SPI_FLASH_CMD_CE) {
        DEBUG_LOG("%s: CHIP ERASE\n",
                  __func__);
        memset(s->flash_image, 0xff, ESP8266_MAX_FLASH_SZ);
    }
    if (val & ESP8266_SPI_FLASH_CMD_PP) {
        DEBUG_LOG("%s: WRITE FLASH 0x%02x@0x%08x\n",
                  __func__,
                  ESP8266_SPI_GET(s, ADDR, LENGTH),
                  ESP8266_SPI_GET(s, ADDR, OFFSET));
        memcpy(s->flash_image + ESP8266_SPI_GET(s, ADDR, OFFSET),
               s->reg + ESP8266_SPI_FLASH_C0,
               (ESP8266_SPI_GET(s, ADDR, LENGTH) + 3) & 0x3c);
    }
}

static void esp8266_spi_write_ctrl(Esp8266SpiState *s, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    s->reg[ESP8266_SPI_FLASH_CTRL] = val;
    memory_region_set_enabled(&s->cache,
                              val & ESP8266_SPI_FLASH_CTRL_ENABLE_AHB);
}

static void esp8266_spi_status(Esp8266SpiState *s, hwaddr addr,
                               uint64_t val, unsigned size)
{
}

static void esp8266_spi_clock(Esp8266SpiState *s, hwaddr addr,
                              uint64_t val, unsigned size)
{
    DEBUG_LOG("%s: L: %d, H: %d, N: %d, PRE: %d, SYSCLK: %d\n",
              __func__,
              ESP8266_SPI_GET_VAL(val, CLOCK, CLKCNT_L),
              ESP8266_SPI_GET_VAL(val, CLOCK, CLKCNT_H),
              ESP8266_SPI_GET_VAL(val, CLOCK, CLKCNT_N),
              ESP8266_SPI_GET_VAL(val, CLOCK, CLK_DIV_PRE),
              ESP8266_SPI_GET_VAL(val, CLOCK, CLK_EQU_SYSCLK));
    s->reg[ESP8266_SPI_FLASH_CLOCK] = val;
}

static void esp8266_spi_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    Esp8266SpiState *s = opaque;
    static void (* const handler[])(Esp8266SpiState *s, hwaddr addr,
                                    uint64_t val, unsigned size) = {
        [ESP8266_SPI_FLASH_CMD] = esp8266_spi_cmd,
        [ESP8266_SPI_FLASH_CTRL] = esp8266_spi_write_ctrl,
        [ESP8266_SPI_FLASH_STATUS] = esp8266_spi_status,
        [ESP8266_SPI_FLASH_CLOCK] = esp8266_spi_clock,
    };

    DEBUG_LOG("%s: +0x%02x = 0x%08x\n",
            __func__, (uint32_t)addr, (uint32_t)val);
    if (addr / 4 >= ESP8266_SPI_MAX || addr % 4 || size != 4) {
        return;
    }
    if (addr / 4 < ARRAY_SIZE(handler) && handler[addr / 4]) {
        handler[addr / 4](s, addr, val, size);
    } else {
        s->reg[addr / 4] = val;
    }
}

static void esp8266_spi_reset(void *opaque)
{
    Esp8266SpiState *s = opaque;

    memset(s->reg, 0, sizeof(s->reg));
    memory_region_set_enabled(&s->cache, false);

    //esp8266_spi_irq_update(s);
}

static const MemoryRegionOps esp8266_spi_ops = {
    .read = esp8266_spi_read,
    .write = esp8266_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Esp8266SpiState *esp8266_spi_init(MemoryRegion *address_space,
                                         hwaddr base, const char *name,
                                         MemoryRegion *cache_space,
                                         hwaddr cache_base,
                                         const char *cache_name,
                                         qemu_irq irq, void **flash_image)
{
    Esp8266SpiState *s = g_malloc(sizeof(Esp8266SpiState));

    s->irq = irq;
    memory_region_init_io(&s->iomem, NULL, &esp8266_spi_ops, s,
                          name, 0x100);
    memory_region_init_rom_device(&s->cache, NULL, NULL, s,
                                  cache_name, ESP8266_MAX_FLASH_SZ,
                                  NULL);
    s->flash_image = memory_region_get_ram_ptr(&s->cache);
    if (flash_image) {
        *flash_image = s->flash_image;
    }
    memory_region_add_subregion(address_space, base, &s->iomem);
    memory_region_add_subregion(cache_space, cache_base, &s->cache);
    memory_region_set_enabled(&s->cache, false);
    qemu_register_reset(esp8266_spi_reset, s);
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
        DEBUG_LOG("%s, +0x%x\n", __func__, (uint32_t)addr);
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

    case 0x28: /*?*/
        return 1;

    default:
        DEBUG_LOG("%s, +0x%x\n", __func__, (uint32_t)addr);
        break;
    }
    return 0;
}

static void esp8266_rtc_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    //Esp8266RtcState *s = opaque;

    DEBUG_LOG("%s: +0x%02x: %08x\n",
              __func__, (uint32_t)addr, (uint32_t)val);
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

/* PINMUX */

enum {
    DUMMY0,

    MTDI,
    MTCK,
    MTMS,
    MTDO,
    U0RXD,
    U0TXD,
    SD_CLK,
    SD_DATA0,
    SD_DATA1,
    SD_DATA2,
    SD_DATA3,
    SD_CMD,
    GPIO0,
    GPIO2,
    GPIO4,
    GPIO5,

    ESP8266_PINMUX_MAX,
};

typedef struct Esp8266PinmuxState {
    MemoryRegion iomem;

    uint32_t reg[ESP8266_PINMUX_MAX];
} Esp8266PinmuxState;

static uint64_t esp8266_pinmux_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    Esp8266PinmuxState *s = opaque;

    if (addr % 4 == 0 && size == 4 && addr / 4 < ESP8266_PINMUX_MAX) {
        return s->reg[addr / 4];
    }
    return 0;
}

static unsigned esp8266_pinmux_function(uint32_t v)
{
    return ((v & 0x30) >> 4) | ((v & 0x100) >> 6);
}

static unsigned esp8266_pinmux_pullup(uint32_t v)
{
    return (v & 0x80) >> 7;
}

static unsigned esp8266_pinmux_pulldn(uint32_t v)
{
    return (v & 0x40) >> 6;
}

static unsigned esp8266_pinmux_oe(uint32_t v)
{
    return v & 0x1;
}

static void esp8266_pinmux_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
#define DEF_PINMUX_PIN(f0, f1, f2, f3, f4) \
    { .fn = {#f0, #f1, #f2, #f3, #f4} }
    static const struct {
        const char * const fn[5];
    } pin[] = {
        {},
        DEF_PINMUX_PIN(MTDI,        I2SI_DATA,  HSPIQ,      GPIO12, U0DTR),
        DEF_PINMUX_PIN(MTCK,        I2SI_BCK,   HSPID,      GPIO13, U0CTS),
        DEF_PINMUX_PIN(MTMS,        I2SI_WS,    HSPICLK,    GPIO14, U0DSR),
        DEF_PINMUX_PIN(MTDO,        I2SO_BCK,   HSPICS,     GPIO15, U0RTS),
        DEF_PINMUX_PIN(U0RXD,       I2SO_DATA,  ,           GPIO3,  CLK_XTAL),
        DEF_PINMUX_PIN(U0TXD,       SPICS1,     ,           GPIO1,  CLK_RTC),
        DEF_PINMUX_PIN(SD_CLK,      SPI_CLK,    ,           GPIO6,  U1CTS),
        DEF_PINMUX_PIN(SD_DATA0,    SPIQ,       ,           GPIO7,  U1TXD),
        DEF_PINMUX_PIN(SD_DATA1,    SPID,       ,           GPIO8,  U1RXD),
        DEF_PINMUX_PIN(SD_DATA2,    SPIHD,      ,           GPIO9,  HSPIHD),
        DEF_PINMUX_PIN(SD_DATA3,    SPIWP,      ,           GPIO10, HSPIWP),
        DEF_PINMUX_PIN(SD_CMD,      SPICS0,     ,           GPIO11, U1RTS),
        DEF_PINMUX_PIN(GPIO0,       SPICS2,     ,           ,       CLK_OUT),
        DEF_PINMUX_PIN(GPIO2,       I2SOWS,     U1TXD,      ,       U0TXD),
        DEF_PINMUX_PIN(GPIO4,       CLK_XTAL,   ,           ,       ),
        DEF_PINMUX_PIN(GPIO5,       CLK_RTC,    ,           ,       ),
    };
    Esp8266PinmuxState *s = opaque;
    unsigned i = addr / 4;

    if (addr < 4 || addr / 4 >= ESP8266_PINMUX_MAX || addr % 4 || size != 4) {
        return;
    }

    if ((s->reg[i] ^ val) & 0x1f1) {
        DEBUG_LOG("%s: ", pin[i].fn[0]);
        if (esp8266_pinmux_function(s->reg[i]) != esp8266_pinmux_function(val)) {
            DEBUG_LOG("[%s -> %s]",
                pin[i].fn[esp8266_pinmux_function(s->reg[i])],
                pin[i].fn[esp8266_pinmux_function(val)]);
        }
        if (esp8266_pinmux_oe(s->reg[i]) != esp8266_pinmux_oe(val)) {
            DEBUG_LOG("[oe: %d -> %d]",
                    esp8266_pinmux_oe(s->reg[i]),
                    esp8266_pinmux_oe(val));
        }
        if (esp8266_pinmux_pullup(s->reg[i]) != esp8266_pinmux_pullup(val)) {
            DEBUG_LOG("[pullup: %d -> %d]",
                    esp8266_pinmux_pullup(s->reg[i]),
                    esp8266_pinmux_pullup(val));
        }
        if (esp8266_pinmux_pulldn(s->reg[i]) != esp8266_pinmux_pulldn(val)) {
            DEBUG_LOG("[pulldn: %d -> %d]",
                    esp8266_pinmux_pulldn(s->reg[i]),
                    esp8266_pinmux_pulldn(val));
        }
        DEBUG_LOG("\n");
    }

    s->reg[i] = val;
}

static const MemoryRegionOps esp8266_pinmux_ops = {
    .read = esp8266_pinmux_read,
    .write = esp8266_pinmux_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_pinmux_reset(void *opaque)
{
    Esp8266PinmuxState *s = opaque;
    static const uint32_t reset_reg[] = {
        0x00000080,
        0x00000080,
        0x00000080,
        0x00000080,
        0x00000080,
        0x00000000,
        0x00000010,
        0x00000010,
        0x00000010,
        0x00000010,
        0x00000010,
        0x00000010,
        0x00000080,
        0x000000a0,
        0x00000000,
        0x00000000,
    };

    memcpy(s->reg, reset_reg, sizeof(s->reg));
}

static Esp8266PinmuxState *esp8266_pinmux_init(MemoryRegion *address_space,
                                           hwaddr base)
{
    Esp8266PinmuxState *s = g_malloc(sizeof(Esp8266PinmuxState));

    memory_region_init_io(&s->iomem, NULL, &esp8266_pinmux_ops, s,
                          "esp8266.pinmux", 0x100);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_pinmux_reset, s);
    return s;
}

/* DPORT area */

enum {
    ESP8266_DPORT_SPI = 0x3,
    ESP8266_DPORT_MACADDR = 0x14,
    ESP8266_DPORT_MAX = 0x40,
};

typedef struct Esp8266DportState {
    MemoryRegion iomem;
    Esp8266SpiState *spi;

    uint32_t reg[ESP8266_DPORT_MAX];
} Esp8266DportState;

static uint64_t esp8266_dport_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    Esp8266DportState *s = opaque;

    DEBUG_LOG("%s: +0x%02x: 0x%08x\n",
              __func__, (uint32_t)addr % (ESP8266_DPORT_MAX * 4),
              s->reg[(addr / 4) % ESP8266_DPORT_MAX]);
    return s->reg[(addr / 4) % ESP8266_DPORT_MAX];
}

static void esp8266_dport_write_spi(Esp8266DportState *s, hwaddr addr,
                                    uint64_t val, unsigned size)
{
    s->reg[ESP8266_DPORT_SPI] = val;
    if (val & 1) {
        s->reg[ESP8266_DPORT_SPI] |= 2;
    }
}

static void esp8266_dport_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    Esp8266DportState *s = opaque;
    static void (* const handler[])(Esp8266DportState *s, hwaddr addr,
                                    uint64_t val, unsigned size) = {
        [ESP8266_DPORT_SPI] = esp8266_dport_write_spi,
    };

    DEBUG_LOG("%s: +0x%02x = 0x%08x\n",
              __func__, (uint32_t)addr % (ESP8266_DPORT_MAX * 4),
              (uint32_t)val);
    if (addr % 4 || size != 4) {
        return;
    }
    if (addr / 4 < ARRAY_SIZE(handler) && handler[addr / 4]) {
        handler[addr / 4](s, addr, val, size);
    } else {
        s->reg[(addr / 4) % ESP8266_DPORT_MAX] = val;
    }
}

static const MemoryRegionOps esp8266_dport_ops = {
    .read = esp8266_dport_read,
    .write = esp8266_dport_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_dport_reset(void *opaque)
{
    Esp8266DportState *s = opaque;
    memset(s->reg, 0, sizeof(s->reg));
    s->reg[ESP8266_DPORT_MACADDR + 0] = 0x01234567;
    s->reg[ESP8266_DPORT_MACADDR + 1] = 0x89abcdef;
    s->reg[ESP8266_DPORT_MACADDR + 2] = 0x00008000;
}

static Esp8266DportState *esp8266_dport_init(MemoryRegion *address_space,
                                             hwaddr base, Esp8266SpiState *spi)
{
    Esp8266DportState *s = g_malloc(sizeof(Esp8266DportState));

    s->spi = spi;
    memory_region_init_io(&s->iomem, NULL, &esp8266_dport_ops, s,
                          "esp8266.dport", 0x10000);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_dport_reset, s);
    return s;
}

/* Internal I2C */

typedef struct Esp8266I2CDevice Esp8266I2CDevice;
struct Esp8266I2CDevice {
    uint8_t (*read)(void *dev, uint8_t reg);
    void (*write)(void *dev, uint8_t reg, uint8_t v);
};


enum {
    ESP8266_I2C_PLL_MAX = 8,
};

typedef struct Esp8266I2CPllState {
    Esp8266I2CDevice dev;

    uint8_t reg[ESP8266_I2C_PLL_MAX];
} Esp8266I2CPllState;

static uint8_t esp8266_i2c_pll_read(void *dev, uint8_t reg)
{
    Esp8266I2CPllState *s = dev;

    if (reg < ESP8266_I2C_PLL_MAX) {
        return s->reg[reg];
    } else {
        return 0;
    }
}

static void esp8266_i2c_pll_write(void *dev, uint8_t reg, uint8_t v)
{
    //Esp8266I2CPllState *s = dev;
}

static void esp8266_i2c_pll_reset(void *opaque)
{
    Esp8266I2CPllState *s = opaque;
    memset(s->reg, 0, sizeof(s->reg));
    s->reg[7] = 0xff;
}

static Esp8266I2CDevice *esp8266_i2c_pll_init(void)
{
    Esp8266I2CPllState *s = g_malloc(sizeof(Esp8266I2CPllState));

    s->dev.read = esp8266_i2c_pll_read;
    s->dev.write = esp8266_i2c_pll_write;
    qemu_register_reset(esp8266_i2c_pll_reset, s);
    return &s->dev;
}


enum {
    ESP8266_I2C_CMD,

    ESP8266_I2C_BUS_MAX = 2,
    ESP8266_I2C_DEVICE_MAX = 128,
};

#define ESP8266_I2C_BITS(reg, field, shift, len) \
    DEFINE_BITS(ESP8266_I2C, reg, field, shift, len)

#define ESP8266_I2C_GET_VAL(v, _reg, _field) \
    extract32(v, \
              ESP8266_I2C_##_reg##_##_field##_SHIFT, \
              ESP8266_I2C_##_reg##_##_field##_LEN)

#define ESP8266_I2C_GET(s, _reg, _field) \
    extract32(s->reg[ESP8266_I2C_##_reg], \
              ESP8266_I2C_##_reg##_##_field##_SHIFT, \
              ESP8266_I2C_##_reg##_##_field##_LEN)

enum {
    ESP8266_I2C_BITS(CMD, ADDR, 0, 7),
    ESP8266_I2C_BITS(CMD, REG, 8, 8),
    ESP8266_I2C_BITS(CMD, DATA, 16, 8),
    ESP8266_I2C_BITS(CMD, WRITE, 24, 1),
    ESP8266_I2C_BITS(CMD, BUSY, 25, 1),
};

typedef struct Esp8266I2CState {
    MemoryRegion iomem;
    Esp8266I2CDevice *device[ESP8266_I2C_BUS_MAX][ESP8266_I2C_DEVICE_MAX];
    uint32_t reg[ESP8266_I2C_BUS_MAX];
} Esp8266I2CState;

static uint64_t esp8266_i2c_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    Esp8266I2CState *s = opaque;

    DEBUG_LOG("%s: +0x%02x: 0x%08x\n",
              __func__, (uint32_t)addr % (ESP8266_I2C_BUS_MAX * 4),
              s->reg[addr / 4]);

    return s->reg[addr / 4];
}

static void esp8266_i2c_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    Esp8266I2CState *s = opaque;
    Esp8266I2CDevice *dev = s->device[addr / 4][ESP8266_I2C_GET_VAL(val, CMD, ADDR)];

    DEBUG_LOG("%s: +0x%02x = 0x%08x\n",
              __func__, (uint32_t)addr % (ESP8266_I2C_BUS_MAX * 4),
              (uint32_t)val);

    if (addr % 4 || size != 4) {
        return;
    }

    if (ESP8266_I2C_GET_VAL(val, CMD, WRITE)) {
        if (dev) {
            dev->write(dev, ESP8266_I2C_GET_VAL(val, CMD, REG),
                       ESP8266_I2C_GET_VAL(val, CMD, DATA));
        }
    } else {
        s->reg[addr / 4] = (val & (ESP8266_I2C_CMD_ADDR | ESP8266_I2C_CMD_REG));
        if (dev) {
            s->reg[addr / 4] |=
                dev->read(dev, ESP8266_I2C_GET_VAL(val, CMD, REG)) <<
                ESP8266_I2C_CMD_DATA_SHIFT;
        } else {
            s->reg[addr / 4] |= ESP8266_I2C_CMD_DATA;
        }
    }
}

static const MemoryRegionOps esp8266_i2c_ops = {
    .read = esp8266_i2c_read,
    .write = esp8266_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_i2c_reset(void *opaque)
{
    Esp8266I2CState *s = opaque;
    memset(s->reg, 0, sizeof(s->reg));
}

static Esp8266I2CState *esp8266_i2c_init(MemoryRegion *address_space,
                                         hwaddr base)
{
    Esp8266I2CState *s = g_malloc(sizeof(Esp8266I2CState));

    memory_region_init_io(&s->iomem, NULL, &esp8266_i2c_ops, s,
                          "esp8266.i2c", ESP8266_I2C_BUS_MAX * 4);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_i2c_reset, s);

    memset(s->device, 0, sizeof(s->device));
    s->device[1][0x62] = esp8266_i2c_pll_init();
    return s;
}

/* Other helpers */

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
    DEBUG_LOG("unassigned: read +0x%08x\n", (uint32_t)addr);
    switch (addr) {
    case 0x057c:
        return 0x80000000;
    case 0x0d4c:
        return 0x01000000;
    }
    return 0;
}

static void esp8266_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    DEBUG_LOG("unassigned: write +0x%08x = %08x\n", (uint32_t)addr, (uint32_t)val);
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
    Esp8266SpiState *spi;
    void *flash_image;
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
    memory_region_init_ram(ram, NULL, "esp8266.dram", 0x002f0000,
                           &error_abort);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(system_memory, 0x3ff10000, ram);

    system_io = g_malloc(sizeof(*system_io));
    memory_region_init_io(system_io, NULL, &esp8266_io_ops, NULL, "esp8266.io",
                          256 * 1024 * 1024);

    memory_region_add_subregion(system_memory, 0x60000000, system_io);

    if (!serial_hds[0]) {
        serial_hds[0] = qemu_chr_new("serial0", "null", NULL);
    }
    esp8266_serial_init(system_io, 0x00000000, "esp8266.uart0",
                        xtensa_get_extint(env, 5), serial_hds[0]);
    spi = esp8266_spi_init(system_io, 0x0000200, "esp8266.spi0",
                           system_memory, 0x40200000, "esp8266.flash",
                           xtensa_get_extint(env, 6), &flash_image);
    esp8266_gpio_init(system_io, 0x00000300);
    esp8266_rtc_init(system_io, 0x00000700);
    esp8266_pinmux_init(system_io, 0x00000800);
    esp8266_i2c_init(system_io, 0x00000d00);

    esp8266_dport_init(system_memory, 0x3ff00000, spi);

    /* Use presence of kernel file name as 'boot from SRAM' switch. */
    if (kernel_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(kernel_filename, translate_phys_addr, cpu,
                &elf_entry, &elf_lowaddr, NULL, be, ELF_MACHINE, 0);
        if (success > 0) {
            user_entry = elf_entry;
            rom_filename = "esp8266-call-user.rom";
        } else {
            int fd = open(kernel_filename, O_RDONLY);

            if (fd < 0) {
                error_report("could not load kernel '%s'\n",
                             kernel_filename);
                exit(EXIT_FAILURE);
            } else {
                if (read(fd, flash_image, ESP8266_MAX_FLASH_SZ) < 0) {
                    DEBUG_LOG("%s: couldn't load flash image\n", __func__);
                }
                close(fd);
            }
        }
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
