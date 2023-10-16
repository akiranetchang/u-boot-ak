// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

/*
 * Memory Functions
 *
 * Copied from FADS ROM, Dan Malek (dmalek@jlc.net)
 */

#include <common.h>
#include <console.h>
#include <bootretry.h>
#include <cli.h>
#include <command.h>
#include <console.h>
#include <display_options.h>
#ifdef CONFIG_MTD_NOR_FLASH
#include <flash.h>
#endif
#include <hash.h>
#include <log.h>
#include <mapmem.h>
#include <rand.h>
#include <watchdog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/compiler.h>
#include <linux/ctype.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

/* Create a compile-time value */
#ifdef MEM_SUPPORT_64BIT_DATA
#define SUPPORT_64BIT_DATA 1
#define HELP_Q ", .q"
#else
#define SUPPORT_64BIT_DATA 0
#define HELP_Q ""
#endif

static int mod_mem(struct cmd_tbl *, int, int, int, char * const []);

/* Display values from last command.
 * Memory modify remembered values are different from display memory.
 */
static ulong	dp_last_addr, dp_last_size;
static ulong	dp_last_length = 0x40;
static ulong	mm_last_addr, mm_last_size;

static	ulong	base_address = 0;
#ifdef CONFIG_CMD_MEM_SEARCH
static ulong dp_last_ms_length;
static u8 search_buf[64];
static uint search_len;
#endif


#ifdef AK
#else
/*
 * SiFive Platform DMA emulation
 *
 * Copyright (c) 2020 Wind River Systems, Inc.
 *
 * Author:
 *   Bin Meng <bin.meng@windriver.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

typedef uint64_t hwaddr;
#include "sifive_pdma.h"

#define DMA_CONTROL         0x000
#define   CONTROL_CLAIM     BIT(0)
#define   CONTROL_RUN       BIT(1)
#define   CONTROL_DONE_IE   BIT(14)
#define   CONTROL_ERR_IE    BIT(15)
#define   CONTROL_DONE      BIT(30)
#define   CONTROL_ERR       BIT(31)

#define DMA_NEXT_CONFIG     0x004
#define   CONFIG_REPEAT     BIT(2)
#define   CONFIG_ORDER      BIT(3)
#define   CONFIG_WRSZ_SHIFT 24
#define   CONFIG_RDSZ_SHIFT 28
#define   CONFIG_SZ_MASK    0xf

#define DMA_NEXT_BYTES      0x008
#define DMA_NEXT_DST        0x010
#define DMA_NEXT_SRC        0x018
#define DMA_EXEC_CONFIG     0x104
#define DMA_EXEC_BYTES      0x108
#define DMA_EXEC_DST        0x110
#define DMA_EXEC_SRC        0x118

/*
 * FU540/FU740 docs are incorrect with NextConfig.wsize/rsize reset values.
 * The reset values tested on Unleashed/Unmatched boards are 6 instead of 0.
 */
#define CONFIG_WRSZ_DEFAULT 6
#define CONFIG_RDSZ_DEFAULT 6

enum dma_chan_state {
    DMA_CHAN_STATE_IDLE,
    DMA_CHAN_STATE_STARTED,
    DMA_CHAN_STATE_ERROR,
    DMA_CHAN_STATE_DONE
};

static void sifive_pdma_run(SiFivePDMAState *s, int ch)
{
    uint64_t bytes = s->chan[ch].next_bytes;
    uint64_t dst = s->chan[ch].next_dst;
    uint64_t src = s->chan[ch].next_src;
    uint32_t config = s->chan[ch].next_config;
    int wsize, rsize, size, remainder;
    uint8_t buf[64];
    int n;

    /* do nothing if bytes to transfer is zero */
    if (!bytes) {
        goto done;
    }

    /*
     * The manual does not describe how the hardware behaviors when
     * config.wsize and config.rsize are given different values.
     * A common case is memory to memory DMA, and in this case they
     * are normally the same. Abort if this expectation fails.
     */
    wsize = (config >> CONFIG_WRSZ_SHIFT) & CONFIG_SZ_MASK;
    rsize = (config >> CONFIG_RDSZ_SHIFT) & CONFIG_SZ_MASK;
    if (wsize != rsize) {
        goto error;
    }

    /*
     * Calculate the transaction size
     *
     * size field is base 2 logarithm of DMA transaction size,
     * but there is an upper limit of 64 bytes per transaction.
     */
    size = wsize;
    if (size > 6) {
        size = 6;
    }
    size = 1 << size;
    remainder = bytes % size;

    /* indicate a DMA transfer is started */
    s->chan[ch].state = DMA_CHAN_STATE_STARTED;
    s->chan[ch].control &= ~CONTROL_DONE;
    s->chan[ch].control &= ~CONTROL_ERR;

    /* load the next_ registers into their exec_ counterparts */
    s->chan[ch].exec_config = config;
    s->chan[ch].exec_bytes = bytes;
    s->chan[ch].exec_dst = dst;
    s->chan[ch].exec_src = src;

    for (n = 0; n < bytes / size; n++) {
        cpu_physical_memory_read(s->chan[ch].exec_src, buf, size);
        cpu_physical_memory_write(s->chan[ch].exec_dst, buf, size);
        s->chan[ch].exec_src += size;
        s->chan[ch].exec_dst += size;
        s->chan[ch].exec_bytes -= size;
    }

    if (remainder) {
        cpu_physical_memory_read(s->chan[ch].exec_src, buf, remainder);
        cpu_physical_memory_write(s->chan[ch].exec_dst, buf, remainder);
        s->chan[ch].exec_src += remainder;
        s->chan[ch].exec_dst += remainder;
        s->chan[ch].exec_bytes -= remainder;
    }

    /* reload exec_ registers if repeat is required */
    if (s->chan[ch].next_config & CONFIG_REPEAT) {
        s->chan[ch].exec_bytes = bytes;
        s->chan[ch].exec_dst = dst;
        s->chan[ch].exec_src = src;
    }

done:
    /* indicate a DMA transfer is done */
    s->chan[ch].state = DMA_CHAN_STATE_DONE;
    s->chan[ch].control &= ~CONTROL_RUN;
    s->chan[ch].control |= CONTROL_DONE;
    return;

error:
    s->chan[ch].state = DMA_CHAN_STATE_ERROR;
    s->chan[ch].control |= CONTROL_ERR;
    return;
} 

#ifdef AK
static inline void sifive_pdma_update_irq(SiFivePDMAState *s, int ch)
{
    bool done_ie, err_ie;

    done_ie = !!(s->chan[ch].control & CONTROL_DONE_IE);
    err_ie = !!(s->chan[ch].control & CONTROL_ERR_IE);

    if (done_ie && (s->chan[ch].control & CONTROL_DONE)) {
        qemu_irq_raise(s->irq[ch * 2]);
    } else {
        qemu_irq_lower(s->irq[ch * 2]);
    }

    if (err_ie && (s->chan[ch].control & CONTROL_ERR)) {
        qemu_irq_raise(s->irq[ch * 2 + 1]);
    } else {
        qemu_irq_lower(s->irq[ch * 2 + 1]);
    }

    s->chan[ch].state = DMA_CHAN_STATE_IDLE;
}
#endif

static uint64_t sifive_pdma_readq(SiFivePDMAState *s, int ch, hwaddr offset)
{
    uint64_t val = 0;

    offset &= 0xfff;
    switch (offset) {
    case DMA_NEXT_BYTES:
        val = s->chan[ch].next_bytes;
        break;
    case DMA_NEXT_DST:
        val = s->chan[ch].next_dst;
        break;
    case DMA_NEXT_SRC:
        val = s->chan[ch].next_src;
        break;
    case DMA_EXEC_BYTES:
        val = s->chan[ch].exec_bytes;
        break;
    case DMA_EXEC_DST:
        val = s->chan[ch].exec_dst;
        break;
    case DMA_EXEC_SRC:
        val = s->chan[ch].exec_src;
        break;
    default:
        printf(       "%s: Unexpected 64-bit access to 0x%x\n",
                      __func__, (unsigned int)offset);
        break;
    }

    return val;
}

static uint32_t sifive_pdma_readl(SiFivePDMAState *s, int ch, hwaddr offset)
{
    uint32_t val = 0;

    offset &= 0xfff;
    switch (offset) {
    case DMA_CONTROL:
        val = s->chan[ch].control;
        break;
    case DMA_NEXT_CONFIG:
        val = s->chan[ch].next_config;
        break;
    case DMA_NEXT_BYTES:
        val = extract64(s->chan[ch].next_bytes, 0, 32);
        break;
    case DMA_NEXT_BYTES + 4:
        val = extract64(s->chan[ch].next_bytes, 32, 32);
        break;
    case DMA_NEXT_DST:
        val = extract64(s->chan[ch].next_dst, 0, 32);
        break;
    case DMA_NEXT_DST + 4:
        val = extract64(s->chan[ch].next_dst, 32, 32);
        break;
    case DMA_NEXT_SRC:
        val = extract64(s->chan[ch].next_src, 0, 32);
        break;
    case DMA_NEXT_SRC + 4:
        val = extract64(s->chan[ch].next_src, 32, 32);
        break;
    case DMA_EXEC_CONFIG:
        val = s->chan[ch].exec_config;
        break;
    case DMA_EXEC_BYTES:
        val = extract64(s->chan[ch].exec_bytes, 0, 32);
        break;
    case DMA_EXEC_BYTES + 4:
        val = extract64(s->chan[ch].exec_bytes, 32, 32);
        break;
    case DMA_EXEC_DST:
        val = extract64(s->chan[ch].exec_dst, 0, 32);
        break;
    case DMA_EXEC_DST + 4:
        val = extract64(s->chan[ch].exec_dst, 32, 32);
        break;
    case DMA_EXEC_SRC:
        val = extract64(s->chan[ch].exec_src, 0, 32);
        break;
    case DMA_EXEC_SRC + 4:
        val = extract64(s->chan[ch].exec_src, 32, 32);
        break;
    default:
        printf(       "%s: Unexpected 32-bit access to 0x%x\n",
                      __func__, offset);
        break;
    }

    return val;
}

static uint64_t sifive_pdma_read(void *opaque, hwaddr offset, unsigned size)
{
    SiFivePDMAState *s = opaque;
    int ch = SIFIVE_PDMA_CHAN_NO(offset);
    uint64_t val = 0;

    if (ch >= SIFIVE_PDMA_CHANS) {
        printf("%s: Invalid channel no %d\n",
                      __func__, ch);
        return 0;
    }

    switch (size) {
    case 8:
        val = sifive_pdma_readq(s, ch, offset);
        break;
    case 4:
        val = sifive_pdma_readl(s, ch, offset);
        break;
    default:
        printf("%s: Invalid read size %u to PDMA\n",
                      __func__, size);
        return 0;
    }

    return val;
}

static void sifive_pdma_writeq(SiFivePDMAState *s, int ch,
                               hwaddr offset, uint64_t value)
{
    offset &= 0xfff;
    switch (offset) {
    case DMA_NEXT_BYTES:
        s->chan[ch].next_bytes = value;
        break;
    case DMA_NEXT_DST:
        s->chan[ch].next_dst = value;
        break;
    case DMA_NEXT_SRC:
        s->chan[ch].next_src = value;
        break;
    case DMA_EXEC_BYTES:
    case DMA_EXEC_DST:
    case DMA_EXEC_SRC:
        /* these are read-only registers */
        break;
    default:
        printf("%s: Unexpected 64-bit access to 0x%x\n",
                      __func__, (unsigned int)offset);
        break;
    }
}

static void sifive_pdma_writel(SiFivePDMAState *s, int ch,
                               hwaddr offset, uint32_t value)
{
    bool claimed, run;

    offset &= 0xfff;
    switch (offset) {
    case DMA_CONTROL:
        claimed = !!(s->chan[ch].control & CONTROL_CLAIM);
        run = !!(s->chan[ch].control & CONTROL_RUN);

        if (!claimed && (value & CONTROL_CLAIM)) {
            /* reset Next* registers */
            s->chan[ch].next_config = (CONFIG_RDSZ_DEFAULT << CONFIG_RDSZ_SHIFT) |
                                      (CONFIG_WRSZ_DEFAULT << CONFIG_WRSZ_SHIFT);
            s->chan[ch].next_bytes = 0;
            s->chan[ch].next_dst = 0;
            s->chan[ch].next_src = 0;
        }

        /* claim bit can only be cleared when run is low */
        if (run && !(value & CONTROL_CLAIM)) {
            value |= CONTROL_CLAIM;
        }

        s->chan[ch].control = value;

        /*
         * If channel was not claimed before run bit is set,
         * or if the channel is disclaimed when run was low,
         * DMA won't run.
         */
        if (!claimed || (!run && !(value & CONTROL_CLAIM))) {
            s->chan[ch].control &= ~CONTROL_RUN;
            return;
        }

        if (value & CONTROL_RUN) {
            sifive_pdma_run(s, ch);
        }

#ifdef AK
        sifive_pdma_update_irq(s, ch);
#endif
        break;
    case DMA_NEXT_CONFIG:
        s->chan[ch].next_config = value;
        break;
    case DMA_NEXT_BYTES:
        s->chan[ch].next_bytes =
            deposit64(s->chan[ch].next_bytes, 0, 32, value);
        break;
    case DMA_NEXT_BYTES + 4:
        s->chan[ch].next_bytes =
            deposit64(s->chan[ch].next_bytes, 32, 32, value);
        break;
    case DMA_NEXT_DST:
        s->chan[ch].next_dst = deposit64(s->chan[ch].next_dst, 0, 32, value);
        break;
    case DMA_NEXT_DST + 4:
        s->chan[ch].next_dst = deposit64(s->chan[ch].next_dst, 32, 32, value);
        break;
    case DMA_NEXT_SRC:
        s->chan[ch].next_src = deposit64(s->chan[ch].next_src, 0, 32, value);
        break;
    case DMA_NEXT_SRC + 4:
        s->chan[ch].next_src = deposit64(s->chan[ch].next_src, 32, 32, value);
        break;
    case DMA_EXEC_CONFIG:
    case DMA_EXEC_BYTES:
    case DMA_EXEC_BYTES + 4:
    case DMA_EXEC_DST:
    case DMA_EXEC_DST + 4:
    case DMA_EXEC_SRC:
    case DMA_EXEC_SRC + 4:
        /* these are read-only registers */
        break;
    default:
        printf("%s: Unexpected 32-bit access to 0x%x\n",
                      __func__, (unsigned int)offset);
        break;
    }
}

static void sifive_pdma_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    SiFivePDMAState *s = opaque;
    int ch = SIFIVE_PDMA_CHAN_NO(offset);

    if (ch >= SIFIVE_PDMA_CHANS) {
        printf("%s: Invalid channel no %d\n",
                      __func__, ch);
        return;
    }

    switch (size) {
    case 8:
        sifive_pdma_writeq(s, ch, offset, value);
        break;
    case 4:
        sifive_pdma_writel(s, ch, offset, (uint32_t) value);
        break;
    default:
        printf("%s: Invalid write size %u to PDMA\n",
                      __func__, size);
        break;
    }
}

#ifdef AK
static const MemoryRegionOps sifive_pdma_ops = {
    .read = sifive_pdma_read,
    .write = sifive_pdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* there are 32-bit and 64-bit wide registers */
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    }
};

static void sifive_pdma_realize(DeviceState *dev, Error **errp)
{
    SiFivePDMAState *s = SIFIVE_PDMA(dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(dev), &sifive_pdma_ops, s,
                          TYPE_SIFIVE_PDMA, SIFIVE_PDMA_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    for (i = 0; i < SIFIVE_PDMA_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);
    }
}

static void sifive_pdma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "SiFive Platform DMA controller";
    dc->realize = sifive_pdma_realize;
}

static const TypeInfo sifive_pdma_info = {
    .name          = TYPE_SIFIVE_PDMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SiFivePDMAState),
    .class_init    = sifive_pdma_class_init,
};

static void sifive_pdma_register_types(void)
{
    type_register_static(&sifive_pdma_info);
}

type_init(sifive_pdma_register_types)
#endif /*AK*/

#endif /*AK*/

/* Memory Display
 *
 * Syntax:
 *	md{.b, .w, .l, .q} {addr} {len}
 */
#define DISP_LINE_LEN	16
static int do_mem_md(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	ulong	addr, length, bytes;
	const void *buf;
	int	size;
	int rc = 0;

	/* We use the last specified parameters, unless new ones are
	 * entered.
	 */
	addr = dp_last_addr;
	size = dp_last_size;
	length = dp_last_length;

	if (argc < 2)
		return CMD_RET_USAGE;

	if ((flag & CMD_FLAG_REPEAT) == 0) {
		/* New command specified.  Check for a size specification.
		 * Defaults to long if no or incorrect specification.
		 */
		if ((size = cmd_get_data_size(argv[0], 4)) < 0)
			return 1;

		/* Address is specified since argc > 1
		*/
		addr = hextoul(argv[1], NULL);
		addr += base_address;

		/* If another parameter, it is the length to display.
		 * Length is the number of objects, not number of bytes.
		 */
		if (argc > 2)
			length = hextoul(argv[2], NULL);
	}

	bytes = size * length;
	buf = map_sysmem(addr, bytes);

	/* Print the lines. */
	print_buffer(addr, buf, size, length, DISP_LINE_LEN / size);
	addr += bytes;
	unmap_sysmem(buf);

	dp_last_addr = addr;
	dp_last_length = length;
	dp_last_size = size;
	return (rc);
}

static int do_mem_mm(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	return mod_mem (cmdtp, 1, flag, argc, argv);
}

static int do_mem_nm(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	return mod_mem (cmdtp, 0, flag, argc, argv);
}

static int do_mem_mw(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	ulong writeval;  /* 64-bit if SUPPORT_64BIT_DATA */
	ulong	addr, count;
	int	size;
	void *buf, *start;
	ulong bytes;

	if ((argc < 3) || (argc > 4))
		return CMD_RET_USAGE;

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 1)
		return 1;

	/* Address is specified since argc > 1
	*/
	addr = hextoul(argv[1], NULL);
	addr += base_address;

	/* Get the value to write.
	*/
	if (SUPPORT_64BIT_DATA)
		writeval = simple_strtoull(argv[2], NULL, 16);
	else
		writeval = hextoul(argv[2], NULL);

	/* Count ? */
	if (argc == 4) {
		count = hextoul(argv[3], NULL);
	} else {
		count = 1;
	}

	bytes = size * count;
	start = map_sysmem(addr, bytes);
	buf = start;
	while (count-- > 0) {
		if (size == 4)
			*((u32 *)buf) = (u32)writeval;
		else if (SUPPORT_64BIT_DATA && size == 8)
			*((ulong *)buf) = writeval;
		else if (size == 2)
			*((u16 *)buf) = (u16)writeval;
		else
			*((u8 *)buf) = (u8)writeval;
		buf += size;
	}
	unmap_sysmem(start);
	return 0;
}

#ifdef CONFIG_CMD_MX_CYCLIC
static int do_mem_mdc(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	int i;
	ulong count;

	if (argc < 4)
		return CMD_RET_USAGE;

	count = dectoul(argv[3], NULL);

	for (;;) {
		do_mem_md (NULL, 0, 3, argv);

		/* delay for <count> ms... */
		for (i=0; i<count; i++)
			udelay(1000);

		/* check for ctrl-c to abort... */
		if (ctrlc()) {
			puts("Abort\n");
			return 0;
		}
	}

	return 0;
}

static int do_mem_mwc(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	int i;
	ulong count;

	if (argc < 4)
		return CMD_RET_USAGE;

	count = dectoul(argv[3], NULL);

	for (;;) {
		do_mem_mw (NULL, 0, 3, argv);

		/* delay for <count> ms... */
		for (i=0; i<count; i++)
			udelay(1000);

		/* check for ctrl-c to abort... */
		if (ctrlc()) {
			puts("Abort\n");
			return 0;
		}
	}

	return 0;
}
#endif /* CONFIG_CMD_MX_CYCLIC */

static int do_mem_cmp(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	ulong	addr1, addr2, count, ngood, bytes;
	int	size;
	int     rcode = 0;
	const char *type;
	const void *buf1, *buf2, *base;
	ulong word1, word2;  /* 64-bit if SUPPORT_64BIT_DATA */

	if (argc != 4)
		return CMD_RET_USAGE;

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;
	type = size == 8 ? "double word" :
	       size == 4 ? "word" :
	       size == 2 ? "halfword" : "byte";

	addr1 = hextoul(argv[1], NULL);
	addr1 += base_address;

	addr2 = hextoul(argv[2], NULL);
	addr2 += base_address;

	count = hextoul(argv[3], NULL);

	bytes = size * count;
	base = buf1 = map_sysmem(addr1, bytes);
	buf2 = map_sysmem(addr2, bytes);
	for (ngood = 0; ngood < count; ++ngood) {
		if (size == 4) {
			word1 = *(u32 *)buf1;
			word2 = *(u32 *)buf2;
		} else if (SUPPORT_64BIT_DATA && size == 8) {
			word1 = *(ulong *)buf1;
			word2 = *(ulong *)buf2;
		} else if (size == 2) {
			word1 = *(u16 *)buf1;
			word2 = *(u16 *)buf2;
		} else {
			word1 = *(u8 *)buf1;
			word2 = *(u8 *)buf2;
		}
		if (word1 != word2) {
			ulong offset = buf1 - base;
			printf("%s at 0x%08lx (%#0*lx) != %s at 0x%08lx (%#0*lx)\n",
				type, (ulong)(addr1 + offset), size, word1,
				type, (ulong)(addr2 + offset), size, word2);
			rcode = 1;
			break;
		}

		buf1 += size;
		buf2 += size;

		/* reset watchdog from time to time */
		if ((ngood % (64 << 10)) == 0)
			WATCHDOG_RESET();
	}
	unmap_sysmem(buf1);
	unmap_sysmem(buf2);

	printf("Total of %ld %s(s) were the same\n", ngood, type);
	return rcode;
}

static int do_mem_cp(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	ulong	addr, dest, count;
	void	*src, *dst;
	int	size;

	if (argc != 4)
		return CMD_RET_USAGE;

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;

	addr = hextoul(argv[1], NULL);
	addr += base_address;

	dest = hextoul(argv[2], NULL);
	dest += base_address;

	count = hextoul(argv[3], NULL);

	if (count == 0) {
		puts ("Zero length ???\n");
		return 1;
	}

	src = map_sysmem(addr, count * size);
	dst = map_sysmem(dest, count * size);

#ifdef CONFIG_MTD_NOR_FLASH
	/* check if we are copying to Flash */
	if (addr2info((ulong)dst)) {
		int rc;

		puts ("Copy to Flash... ");

		rc = flash_write((char *)src, (ulong)dst, count * size);
		if (rc != 0) {
			flash_perror(rc);
			unmap_sysmem(src);
			unmap_sysmem(dst);
			return (1);
		}
		puts ("done\n");
		unmap_sysmem(src);
		unmap_sysmem(dst);
		return 0;
	}
#endif

	memcpy(dst, src, count * size);

	unmap_sysmem(src);
	unmap_sysmem(dst);
	return 0;
}

#ifdef CONFIG_CMD_MEM_SEARCH
static int do_mem_search(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	ulong addr, length, bytes, offset;
	u8 *ptr, *end, *buf;
	bool quiet = false;
	ulong last_pos;		/* Offset of last match in 'size' units*/
	ulong last_addr;	/* Address of last displayed line */
	int limit = 10;
	int used_len;
	int count;
	int size;
	int i;

	/* We use the last specified parameters, unless new ones are entered */
	addr = dp_last_addr;
	size = dp_last_size;
	length = dp_last_ms_length;

	if (argc < 3)
		return CMD_RET_USAGE;

	if (!(flag & CMD_FLAG_REPEAT)) {
		/*
		 * Check for a size specification.
		 * Defaults to long if no or incorrect specification.
		 */
		size = cmd_get_data_size(argv[0], 4);
		if (size < 0 && size != CMD_DATA_SIZE_STR)
			return 1;

		argc--;
		argv++;
		while (argc && *argv[0] == '-') {
			int ch = argv[0][1];

			if (ch == 'q')
				quiet = true;
			else if (ch == 'l' && isxdigit(argv[0][2]))
				limit = hextoul(argv[0] + 2, NULL);
			else
				return CMD_RET_USAGE;
			argc--;
			argv++;
		}

		/* Address is specified since argc > 1 */
		addr = hextoul(argv[0], NULL);
		addr += base_address;

		/* Length is the number of objects, not number of bytes */
		length = hextoul(argv[1], NULL);

		/* Read the bytes to search for */
		end = search_buf + sizeof(search_buf);
		for (i = 2, ptr = search_buf; i < argc && ptr < end; i++) {
			if (MEM_SUPPORT_64BIT_DATA && size == 8) {
				u64 val = simple_strtoull(argv[i], NULL, 16);

				*(u64 *)ptr = val;
			} else if (size == -2) {  /* string */
				int len = min(strlen(argv[i]),
					      (size_t)(end - ptr));

				memcpy(ptr, argv[i], len);
				ptr += len;
				continue;
			} else {
				u32 val = hextoul(argv[i], NULL);

				switch (size) {
				case 1:
					*ptr = val;
					break;
				case 2:
					*(u16 *)ptr = val;
					break;
				case 4:
					*(u32 *)ptr = val;
					break;
				}
			}
			ptr += size;
		}
		search_len = ptr - search_buf;
	}

	/* Do the search */
	if (size == -2)
		size = 1;
	bytes = size * length;
	buf = map_sysmem(addr, bytes);
	last_pos = 0;
	last_addr = 0;
	count = 0;
	for (offset = 0;
	     offset < bytes && offset <= bytes - search_len && count < limit;
	     offset += size) {
		void *ptr = buf + offset;

		if (!memcmp(ptr, search_buf, search_len)) {
			uint align = (addr + offset) & 0xf;
			ulong match = addr + offset;

			if (!count || (last_addr & ~0xf) != (match & ~0xf)) {
				if (!quiet) {
					if (count)
						printf("--\n");
					print_buffer(match - align, ptr - align,
						     size,
						     ALIGN(search_len + align,
							   16) / size, 0);
				}
				last_addr = match;
				last_pos = offset / size;
			}
			count++;
		}
	}
	if (!quiet) {
		printf("%d match%s", count, count == 1 ? "" : "es");
		if (count == limit)
			printf(" (repeat command to check for more)");
		printf("\n");
	}
	env_set_hex("memmatches", count);
	env_set_hex("memaddr", last_addr);
	env_set_hex("mempos", last_pos);

	unmap_sysmem(buf);

	used_len = offset / size;
	dp_last_addr = addr + used_len;
	dp_last_size = size;
	dp_last_ms_length = length < used_len ? 0 : length - used_len;

	return count ? 0 : CMD_RET_FAILURE;
}
#endif

static int do_mem_base(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	if (argc > 1) {
		/* Set new base address.
		*/
		base_address = hextoul(argv[1], NULL);
	}
	/* Print the current base address.
	*/
	printf("Base Address: 0x%08lx\n", base_address);
	return 0;
}

static int do_mem_loop(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	ulong	addr, length, i, bytes;
	int	size;
	volatile ulong *llp;  /* 64-bit if SUPPORT_64BIT_DATA */
	volatile u32 *longp;
	volatile u16 *shortp;
	volatile u8 *cp;
	const void *buf;

	if (argc < 3)
		return CMD_RET_USAGE;

	/*
	 * Check for a size specification.
	 * Defaults to long if no or incorrect specification.
	 */
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;

	/* Address is always specified.
	*/
	addr = hextoul(argv[1], NULL);

	/* Length is the number of objects, not number of bytes.
	*/
	length = hextoul(argv[2], NULL);

	bytes = size * length;
	buf = map_sysmem(addr, bytes);

	/* We want to optimize the loops to run as fast as possible.
	 * If we have only one object, just run infinite loops.
	 */
	if (length == 1) {
		if (SUPPORT_64BIT_DATA && size == 8) {
			llp = (ulong *)buf;
			for (;;)
				i = *llp;
		}
		if (size == 4) {
			longp = (u32 *)buf;
			for (;;)
				i = *longp;
		}
		if (size == 2) {
			shortp = (u16 *)buf;
			for (;;)
				i = *shortp;
		}
		cp = (u8 *)buf;
		for (;;)
			i = *cp;
	}

	if (SUPPORT_64BIT_DATA && size == 8) {
		for (;;) {
			llp = (ulong *)buf;
			i = length;
			while (i-- > 0)
				*llp++;
		}
	}
	if (size == 4) {
		for (;;) {
			longp = (u32 *)buf;
			i = length;
			while (i-- > 0)
				*longp++;
		}
	}
	if (size == 2) {
		for (;;) {
			shortp = (u16 *)buf;
			i = length;
			while (i-- > 0)
				*shortp++;
		}
	}
	for (;;) {
		cp = (u8 *)buf;
		i = length;
		while (i-- > 0)
			*cp++;
	}
	unmap_sysmem(buf);

	return 0;
}

#ifdef CONFIG_LOOPW
static int do_mem_loopw(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	ulong	addr, length, i, bytes;
	int	size;
	volatile ulong *llp;  /* 64-bit if SUPPORT_64BIT_DATA */
	ulong	data;    /* 64-bit if SUPPORT_64BIT_DATA */
	volatile u32 *longp;
	volatile u16 *shortp;
	volatile u8 *cp;
	void *buf;

	if (argc < 4)
		return CMD_RET_USAGE;

	/*
	 * Check for a size specification.
	 * Defaults to long if no or incorrect specification.
	 */
	if ((size = cmd_get_data_size(argv[0], 4)) < 0)
		return 1;

	/* Address is always specified.
	*/
	addr = hextoul(argv[1], NULL);

	/* Length is the number of objects, not number of bytes.
	*/
	length = hextoul(argv[2], NULL);

	/* data to write */
	if (SUPPORT_64BIT_DATA)
		data = simple_strtoull(argv[3], NULL, 16);
	else
		data = hextoul(argv[3], NULL);

	bytes = size * length;
	buf = map_sysmem(addr, bytes);

	/* We want to optimize the loops to run as fast as possible.
	 * If we have only one object, just run infinite loops.
	 */
	if (length == 1) {
		if (SUPPORT_64BIT_DATA && size == 8) {
			llp = (ulong *)buf;
			for (;;)
				*llp = data;
		}
		if (size == 4) {
			longp = (u32 *)buf;
			for (;;)
				*longp = data;
		}
		if (size == 2) {
			shortp = (u16 *)buf;
			for (;;)
				*shortp = data;
		}
		cp = (u8 *)buf;
		for (;;)
			*cp = data;
	}

	if (SUPPORT_64BIT_DATA && size == 8) {
		for (;;) {
			llp = (ulong *)buf;
			i = length;
			while (i-- > 0)
				*llp++ = data;
		}
	}
	if (size == 4) {
		for (;;) {
			longp = (u32 *)buf;
			i = length;
			while (i-- > 0)
				*longp++ = data;
		}
	}
	if (size == 2) {
		for (;;) {
			shortp = (u16 *)buf;
			i = length;
			while (i-- > 0)
				*shortp++ = data;
		}
	}
	for (;;) {
		cp = (u8 *)buf;
		i = length;
		while (i-- > 0)
			*cp++ = data;
	}
}
#endif /* CONFIG_LOOPW */

#ifdef CONFIG_CMD_MEMTEST
static ulong mem_test_alt(vu_long *buf, ulong start_addr, ulong end_addr,
			  vu_long *dummy)
{
	vu_long *addr;
	ulong errs = 0;
	ulong val, readback;
	int j;
	vu_long offset;
	vu_long test_offset;
	vu_long pattern;
	vu_long temp;
	vu_long anti_pattern;
	vu_long num_words;
	static const ulong bitpattern[] = {
		0x00000001,	/* single bit */
		0x00000003,	/* two adjacent bits */
		0x00000007,	/* three adjacent bits */
		0x0000000F,	/* four adjacent bits */
		0x00000005,	/* two non-adjacent bits */
		0x00000015,	/* three non-adjacent bits */
		0x00000055,	/* four non-adjacent bits */
		0xaaaaaaaa,	/* alternating 1/0 */
	};

	num_words = (end_addr - start_addr) / sizeof(vu_long);

	/*
	 * Data line test: write a pattern to the first
	 * location, write the 1's complement to a 'parking'
	 * address (changes the state of the data bus so a
	 * floating bus doesn't give a false OK), and then
	 * read the value back. Note that we read it back
	 * into a variable because the next time we read it,
	 * it might be right (been there, tough to explain to
	 * the quality guys why it prints a failure when the
	 * "is" and "should be" are obviously the same in the
	 * error message).
	 *
	 * Rather than exhaustively testing, we test some
	 * patterns by shifting '1' bits through a field of
	 * '0's and '0' bits through a field of '1's (i.e.
	 * pattern and ~pattern).
	 */
	addr = buf;
	for (j = 0; j < sizeof(bitpattern) / sizeof(bitpattern[0]); j++) {
		val = bitpattern[j];
		for (; val != 0; val <<= 1) {
			*addr = val;
			*dummy  = ~val; /* clear the test data off the bus */
			readback = *addr;
			if (readback != val) {
				printf("FAILURE (data line): "
					"expected %08lx, actual %08lx\n",
						val, readback);
				errs++;
				if (ctrlc())
					return -1;
			}
			*addr  = ~val;
			*dummy  = val;
			readback = *addr;
			if (readback != ~val) {
				printf("FAILURE (data line): "
					"Is %08lx, should be %08lx\n",
						readback, ~val);
				errs++;
				if (ctrlc())
					return -1;
			}
		}
	}

	/*
	 * Based on code whose Original Author and Copyright
	 * information follows: Copyright (c) 1998 by Michael
	 * Barr. This software is placed into the public
	 * domain and may be used for any purpose. However,
	 * this notice must not be changed or removed and no
	 * warranty is either expressed or implied by its
	 * publication or distribution.
	 */

	/*
	* Address line test

	 * Description: Test the address bus wiring in a
	 *              memory region by performing a walking
	 *              1's test on the relevant bits of the
	 *              address and checking for aliasing.
	 *              This test will find single-bit
	 *              address failures such as stuck-high,
	 *              stuck-low, and shorted pins. The base
	 *              address and size of the region are
	 *              selected by the caller.

	 * Notes:	For best results, the selected base
	 *              address should have enough LSB 0's to
	 *              guarantee single address bit changes.
	 *              For example, to test a 64-Kbyte
	 *              region, select a base address on a
	 *              64-Kbyte boundary. Also, select the
	 *              region size as a power-of-two if at
	 *              all possible.
	 *
	 * Returns:     0 if the test succeeds, 1 if the test fails.
	 */
	pattern = (vu_long) 0xaaaaaaaa;
	anti_pattern = (vu_long) 0x55555555;

	debug("%s:%d: length = 0x%.8lx\n", __func__, __LINE__, num_words);
	/*
	 * Write the default pattern at each of the
	 * power-of-two offsets.
	 */
	for (offset = 1; offset < num_words; offset <<= 1)
		addr[offset] = pattern;

	/*
	 * Check for address bits stuck high.
	 */
	test_offset = 0;
	addr[test_offset] = anti_pattern;

	for (offset = 1; offset < num_words; offset <<= 1) {
		temp = addr[offset];
		if (temp != pattern) {
			printf("\nFAILURE: Address bit stuck high @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx\n",
				start_addr + offset*sizeof(vu_long),
				pattern, temp);
			errs++;
			if (ctrlc())
				return -1;
		}
	}
	addr[test_offset] = pattern;
	WATCHDOG_RESET();

	/*
	 * Check for addr bits stuck low or shorted.
	 */
	for (test_offset = 1; test_offset < num_words; test_offset <<= 1) {
		addr[test_offset] = anti_pattern;

		for (offset = 1; offset < num_words; offset <<= 1) {
			temp = addr[offset];
			if ((temp != pattern) && (offset != test_offset)) {
				printf("\nFAILURE: Address bit stuck low or"
					" shorted @ 0x%.8lx: expected 0x%.8lx,"
					" actual 0x%.8lx\n",
					start_addr + offset*sizeof(vu_long),
					pattern, temp);
				errs++;
				if (ctrlc())
					return -1;
			}
		}
		addr[test_offset] = pattern;
	}

	/*
	 * Description: Test the integrity of a physical
	 *		memory device by performing an
	 *		increment/decrement test over the
	 *		entire region. In the process every
	 *		storage bit in the device is tested
	 *		as a zero and a one. The base address
	 *		and the size of the region are
	 *		selected by the caller.
	 *
	 * Returns:     0 if the test succeeds, 1 if the test fails.
	 */
	num_words++;

	/*
	 * Fill memory with a known pattern.
	 */
	for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		WATCHDOG_RESET();
		addr[offset] = pattern;
	}

	/*
	 * Check each location and invert it for the second pass.
	 */
	for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		WATCHDOG_RESET();
		temp = addr[offset];
		if (temp != pattern) {
			printf("\nFAILURE (read/write) @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx)\n",
				start_addr + offset*sizeof(vu_long),
				pattern, temp);
			errs++;
			if (ctrlc())
				return -1;
		}

		anti_pattern = ~pattern;
		addr[offset] = anti_pattern;
	}

	/*
	 * Check each location for the inverted pattern and zero it.
	 */
	for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		WATCHDOG_RESET();
		anti_pattern = ~pattern;
		temp = addr[offset];
		if (temp != anti_pattern) {
			printf("\nFAILURE (read/write): @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx)\n",
				start_addr + offset*sizeof(vu_long),
				anti_pattern, temp);
			errs++;
			if (ctrlc())
				return -1;
		}
		addr[offset] = 0;
	}

	return errs;
}

static int compare_regions(volatile unsigned long *bufa,
			   volatile unsigned long *bufb, size_t count)
{
	volatile unsigned long  *p1 = bufa;
	volatile unsigned long  *p2 = bufb;
	int errs = 0;
	size_t i;

	for (i = 0; i < count; i++, p1++, p2++) {
		if (*p1 != *p2) {
			printf("FAILURE: 0x%08lx != 0x%08lx (delta=0x%08lx -> bit %ld) at offset 0x%08lx\n",
			       (unsigned long)*p1, (unsigned long)*p2,
			       *p1 ^ *p2, __ffs(*p1 ^ *p2),
				(unsigned long)(i * sizeof(unsigned long)));
			errs++;
		}
	}

	return errs;
}

static ulong test_bitflip_comparison(volatile unsigned long *bufa,
				     volatile unsigned long *bufb, size_t count)
{
	volatile unsigned long *p1 = bufa;
	volatile unsigned long *p2 = bufb;
	unsigned int j, k;
	unsigned long q;
	size_t i;
	int max;
	int errs = 0;

	max = sizeof(unsigned long) * 8;
	for (k = 0; k < max; k++) {
		q = 0x00000001L << k;
		for (j = 0; j < 8; j++) {
			WATCHDOG_RESET();
			q = ~q;
			p1 = (volatile unsigned long *)bufa;
			p2 = (volatile unsigned long *)bufb;
			for (i = 0; i < count; i++)
				*p1++ = *p2++ = (i % 2) == 0 ? q : ~q;

			errs += compare_regions(bufa, bufb, count);
		}

		if (ctrlc())
			return -1UL;
	}

	return errs;
}

static ulong mem_test_bitflip(vu_long *buf, ulong start, ulong end)
{
	/*
	 * Split the specified range into two halves.
	 * Note that mtest range is inclusive of start,end.
	 * Bitflip test instead uses a count (of 32-bit words).
	 */
	ulong half_size = (end - start + 1) / 2 / sizeof(unsigned long);

	return test_bitflip_comparison(buf, buf + half_size, half_size);
}

static ulong mem_test_quick(vu_long *buf, ulong start_addr, ulong end_addr,
			    vu_long pattern, int iteration)
{
	vu_long *end;
	vu_long *addr;
	ulong errs = 0;
	ulong incr, length;
	ulong val, readback;

	/* Alternate the pattern */
	incr = 1;
	if (iteration & 1) {
		incr = -incr;
		/*
		 * Flip the pattern each time to make lots of zeros and
		 * then, the next time, lots of ones.  We decrement
		 * the "negative" patterns and increment the "positive"
		 * patterns to preserve this feature.
		 */
		if (pattern & 0x80000000)
			pattern = -pattern;	/* complement & increment */
		else
			pattern = ~pattern;
	}
	length = (end_addr - start_addr) / sizeof(ulong);
	end = buf + length;
	printf("\rPattern %08lX  Writing..."
		"%12s"
		"\b\b\b\b\b\b\b\b\b\b",
		pattern, "");

	for (addr = buf, val = pattern; addr < end; addr++) {
		WATCHDOG_RESET();
		*addr = val;
		val += incr;
	}

	puts("Reading...");

	for (addr = buf, val = pattern; addr < end; addr++) {
		WATCHDOG_RESET();
		readback = *addr;
		if (readback != val) {
			ulong offset = addr - buf;

			printf("\nMem error @ 0x%08X: "
				"found %08lX, expected %08lX\n",
				(uint)(uintptr_t)(start_addr + offset*sizeof(vu_long)),
				readback, val);
			errs++;
			if (ctrlc())
				return -1;
		}
		val += incr;
	}

	return errs;
}

/*
 * Perform a memory test. A more complete alternative test can be
 * configured using CONFIG_SYS_ALT_MEMTEST. The complete test loops until
 * interrupted by ctrl-c or by a failure of one of the sub-tests.
 */
static int do_mem_mtest(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	ulong start, end;
	vu_long scratch_space;
	vu_long *buf, *dummy = &scratch_space;
	ulong iteration_limit = 0;
	ulong count = 0;
	ulong errs = 0;	/* number of errors, or -1 if interrupted */
	ulong pattern = 0;
	int iteration;

	start = CONFIG_SYS_MEMTEST_START;
	end = CONFIG_SYS_MEMTEST_END;

	if (argc > 1)
		if (strict_strtoul(argv[1], 16, &start) < 0)
			return CMD_RET_USAGE;

	if (argc > 2)
		if (strict_strtoul(argv[2], 16, &end) < 0)
			return CMD_RET_USAGE;

	if (argc > 3)
		if (strict_strtoul(argv[3], 16, &pattern) < 0)
			return CMD_RET_USAGE;

	if (argc > 4)
		if (strict_strtoul(argv[4], 16, &iteration_limit) < 0)
			return CMD_RET_USAGE;

	if (end < start) {
		printf("Refusing to do empty test\n");
		return -1;
	}

	printf("Testing %08lx ... %08lx:\n", start, end);
	debug("%s:%d: start %#08lx end %#08lx\n", __func__, __LINE__,
	      start, end);

	buf = map_sysmem(start, end - start);
	for (iteration = 0;
			!iteration_limit || iteration < iteration_limit;
			iteration++) {
		if (ctrlc()) {
			errs = -1UL;
			break;
		}

		printf("Iteration: %6d\r", iteration + 1);
		debug("\n");
		if (IS_ENABLED(CONFIG_SYS_ALT_MEMTEST)) {
			errs = mem_test_alt(buf, start, end, dummy);
			if (errs == -1UL)
				break;
			if (IS_ENABLED(CONFIG_SYS_ALT_MEMTEST_BITFLIP)) {
				count += errs;
				errs = mem_test_bitflip(buf, start, end);
			}
		} else {
			errs = mem_test_quick(buf, start, end, pattern,
					      iteration);
		}
		if (errs == -1UL)
			break;
		count += errs;
	}

	unmap_sysmem((void *)buf);

	if (errs == -1UL) {
		/* Memory test was aborted - write a newline to finish off */
		putc('\n');
	}
	printf("Tested %d iteration(s) with %lu errors.\n", iteration, count);

	return errs != 0;
}
#endif	/* CONFIG_CMD_MEMTEST */

/* Modify memory.
 *
 * Syntax:
 *	mm{.b, .w, .l, .q} {addr}
 */
static int
mod_mem(struct cmd_tbl *cmdtp, int incrflag, int flag, int argc,
	char *const argv[])
{
	ulong	addr;
	ulong i;  /* 64-bit if SUPPORT_64BIT_DATA */
	int	nbytes, size;
	void *ptr = NULL;

	if (argc != 2)
		return CMD_RET_USAGE;

	bootretry_reset_cmd_timeout();	/* got a good command to get here */
	/* We use the last specified parameters, unless new ones are
	 * entered.
	 */
	addr = mm_last_addr;
	size = mm_last_size;

	if ((flag & CMD_FLAG_REPEAT) == 0) {
		/* New command specified.  Check for a size specification.
		 * Defaults to long if no or incorrect specification.
		 */
		if ((size = cmd_get_data_size(argv[0], 4)) < 0)
			return 1;

		/* Address is specified since argc > 1
		*/
		addr = hextoul(argv[1], NULL);
		addr += base_address;
	}

	/* Print the address, followed by value.  Then accept input for
	 * the next value.  A non-converted value exits.
	 */
	do {
		ptr = map_sysmem(addr, size);
		printf("%08lx:", addr);
		if (size == 4)
			printf(" %08x", *((u32 *)ptr));
		else if (SUPPORT_64BIT_DATA && size == 8)
			printf(" %0lx", *((ulong *)ptr));
		else if (size == 2)
			printf(" %04x", *((u16 *)ptr));
		else
			printf(" %02x", *((u8 *)ptr));

		nbytes = cli_readline(" ? ");
		if (nbytes == 0 || (nbytes == 1 && console_buffer[0] == '-')) {
			/* <CR> pressed as only input, don't modify current
			 * location and move to next. "-" pressed will go back.
			 */
			if (incrflag)
				addr += nbytes ? -size : size;
			nbytes = 1;
			/* good enough to not time out */
			bootretry_reset_cmd_timeout();
		}
#ifdef CONFIG_BOOT_RETRY_TIME
		else if (nbytes == -2) {
			break;	/* timed out, exit the command	*/
		}
#endif
		else {
			char *endp;
			if (SUPPORT_64BIT_DATA)
				i = simple_strtoull(console_buffer, &endp, 16);
			else
				i = hextoul(console_buffer, &endp);
			nbytes = endp - console_buffer;
			if (nbytes) {
				/* good enough to not time out
				 */
				bootretry_reset_cmd_timeout();
				if (size == 4)
					*((u32 *)ptr) = i;
				else if (SUPPORT_64BIT_DATA && size == 8)
					*((ulong *)ptr) = i;
				else if (size == 2)
					*((u16 *)ptr) = i;
				else
					*((u8 *)ptr) = i;
				if (incrflag)
					addr += size;
			}
		}
	} while (nbytes);
	if (ptr)
		unmap_sysmem(ptr);

	mm_last_addr = addr;
	mm_last_size = size;
	return 0;
}

#ifdef CONFIG_CMD_CRC32

static int do_mem_crc(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	int flags = 0;
	int ac;
	char * const *av;

	if (argc < 3)
		return CMD_RET_USAGE;

	av = argv + 1;
	ac = argc - 1;
#ifdef CONFIG_CRC32_VERIFY
	if (strcmp(*av, "-v") == 0) {
		flags |= HASH_FLAG_VERIFY | HASH_FLAG_ENV;
		av++;
		ac--;
	}
#endif

	return hash_command("crc32", flags, cmdtp, flag, ac, av);
}

#endif

#ifdef CONFIG_CMD_RANDOM
static int do_random(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	unsigned long addr, len;
	unsigned long seed; // NOT INITIALIZED ON PURPOSE
	unsigned int *buf, *start;
	unsigned char *buf8;
	unsigned int i;

	if (argc < 3 || argc > 4)
		return CMD_RET_USAGE;

	len = hextoul(argv[2], NULL);
	addr = hextoul(argv[1], NULL);

	if (argc == 4) {
		seed = hextoul(argv[3], NULL);
		if (seed == 0) {
			printf("The seed cannot be 0. Using 0xDEADBEEF.\n");
			seed = 0xDEADBEEF;
		}
	} else {
		seed = get_timer(0) ^ rand();
	}

	srand(seed);
	start = map_sysmem(addr, len);
	buf = start;
	for (i = 0; i < (len / 4); i++)
		*buf++ = rand();

	buf8 = (unsigned char *)buf;
	for (i = 0; i < (len % 4); i++)
		*buf8++ = rand() & 0xFF;

	unmap_sysmem(start);
	printf("%lu bytes filled with random data\n", len);

	return CMD_RET_SUCCESS;
}
#endif

/**************************************************/
U_BOOT_CMD(
	md,	3,	1,	do_mem_md,
	"memory display",
	"[.b, .w, .l" HELP_Q "] address [# of objects]"
);


U_BOOT_CMD(
	mm,	2,	1,	do_mem_mm,
	"memory modify (auto-incrementing address)",
	"[.b, .w, .l" HELP_Q "] address"
);


U_BOOT_CMD(
	nm,	2,	1,	do_mem_nm,
	"memory modify (constant address)",
	"[.b, .w, .l" HELP_Q "] address"
);

U_BOOT_CMD(
	mw,	4,	1,	do_mem_mw,
	"memory write (fill)",
	"[.b, .w, .l" HELP_Q "] address value [count]"
);

U_BOOT_CMD(
	cp,	4,	1,	do_mem_cp,
	"memory copy",
	"[.b, .w, .l" HELP_Q "] source target count"
);

U_BOOT_CMD(
	cmp,	4,	1,	do_mem_cmp,
	"memory compare",
	"[.b, .w, .l" HELP_Q "] addr1 addr2 count"
);

#ifdef CONFIG_CMD_MEM_SEARCH
/**************************************************/
U_BOOT_CMD(
	ms,	255,	1,	do_mem_search,
	"memory search",
	"[.b, .w, .l" HELP_Q ", .s] [-q | -<n>] address #-of-objects <value>..."
	"  -q = quiet, -l<val> = match limit"
);
#endif

#ifdef CONFIG_CMD_CRC32

#ifndef CONFIG_CRC32_VERIFY

U_BOOT_CMD(
	crc32,	4,	1,	do_mem_crc,
	"checksum calculation",
	"address count [addr]\n    - compute CRC32 checksum [save at addr]"
);

#else	/* CONFIG_CRC32_VERIFY */

U_BOOT_CMD(
	crc32,	5,	1,	do_mem_crc,
	"checksum calculation",
	"address count [addr]\n    - compute CRC32 checksum [save at addr]\n"
	"-v address count crc\n    - verify crc of memory area"
);

#endif	/* CONFIG_CRC32_VERIFY */

#endif

#ifdef CONFIG_CMD_MEMINFO
static int do_mem_info(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	puts("DRAM:  ");
	print_size(gd->ram_size, "\n");

	return 0;
}
#endif

U_BOOT_CMD(
	base,	2,	1,	do_mem_base,
	"print or set address offset",
	"\n    - print address offset for memory commands\n"
	"base off\n    - set address offset for memory commands to 'off'"
);

U_BOOT_CMD(
	loop,	3,	1,	do_mem_loop,
	"infinite loop on address range",
	"[.b, .w, .l" HELP_Q "] address number_of_objects"
);

#ifdef CONFIG_LOOPW
U_BOOT_CMD(
	loopw,	4,	1,	do_mem_loopw,
	"infinite write loop on address range",
	"[.b, .w, .l" HELP_Q "] address number_of_objects data_to_write"
);
#endif /* CONFIG_LOOPW */

#ifdef CONFIG_CMD_MEMTEST
U_BOOT_CMD(
	mtest,	5,	1,	do_mem_mtest,
	"simple RAM read/write test",
	"[start [end [pattern [iterations]]]]"
);
#endif	/* CONFIG_CMD_MEMTEST */

#ifdef CONFIG_CMD_MX_CYCLIC
U_BOOT_CMD(
	mdc,	4,	1,	do_mem_mdc,
	"memory display cyclic",
	"[.b, .w, .l" HELP_Q "] address count delay(ms)"
);

U_BOOT_CMD(
	mwc,	4,	1,	do_mem_mwc,
	"memory write cyclic",
	"[.b, .w, .l" HELP_Q "] address value delay(ms)"
);
#endif /* CONFIG_CMD_MX_CYCLIC */

#ifdef CONFIG_CMD_MEMINFO
U_BOOT_CMD(
	meminfo,	3,	1,	do_mem_info,
	"display memory information",
	""
);
#endif

#ifdef CONFIG_CMD_RANDOM
U_BOOT_CMD(
	random,	4,	0,	do_random,
	"fill memory with random pattern",
	"<addr> <len> [<seed>]\n"
	"   - Fill 'len' bytes of memory starting at 'addr' with random data\n"
);
#endif
