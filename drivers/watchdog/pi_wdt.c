// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013 Altera Corporation <www.altera.com>
 */

#include <clk.h>
#include <common.h>
#include <dm.h>
#include <reset.h>
#include <wdt.h>
#include <asm/io.h>
#include <linux/bitops.h>

#define DW_WDT_CR	0x00
#define DW_WDT_TORR	0x04
#define DW_WDT_CRR	0x0C

#define DW_WDT_CR_EN_OFFSET	0x00
#define DW_WDT_CR_RMOD_OFFSET	0x01
#define DW_WDT_CRR_RESTART_VAL	0x76

struct pi_wdt_priv {
	void __iomem	*base;
	unsigned int	clk_khz;
	struct reset_ctl_bulk resets;
};

/*
 * Set the watchdog time interval.
 * Counter is 32 bit.
 */
static int pi_wdt_settimeout(void __iomem *base, unsigned int clk_khz,
				     unsigned int timeout)
{
	signed int i;

	/* calculate the timeout range value */
	i = fls(timeout * clk_khz - 1) - 16;
	i = clamp(i, 0, 15);

	writel(i | (i << 4), base + DW_WDT_TORR);

	return 0;
}

static void pi_wdt_enable(void __iomem *base)
{
	writel(BIT(DW_WDT_CR_EN_OFFSET), base + DW_WDT_CR);
}

static unsigned int pi_wdt_is_enabled(void __iomem *base)
{
	return readl(base + DW_WDT_CR) & BIT(0);
}

static void pi_wdt_reset_common(void __iomem *base)
{
	if (pi_wdt_is_enabled(base))
		/* restart the watchdog counter */
		writel(DW_WDT_CRR_RESTART_VAL, base + DW_WDT_CRR);
}

static int pi_wdt_reset(struct udevice *dev)
{
	struct pi_wdt_priv *priv = dev_get_priv(dev);

	pi_wdt_reset_common(priv->base);

	return 0;
}

static int pi_wdt_stop(struct udevice *dev)
{
	struct pi_wdt_priv *priv = dev_get_priv(dev);

	pi_wdt_reset(dev);
	writel(0, priv->base + DW_WDT_CR);

        if (CONFIG_IS_ENABLED(DM_RESET)) {
		int ret;

		ret = reset_assert_bulk(&priv->resets);
		if (ret)
			return ret;

		ret = reset_deassert_bulk(&priv->resets);
		if (ret)
			return ret;
	}

	return 0;
}

static int pi_wdt_start(struct udevice *dev, u64 timeout, ulong flags)
{
	struct pi_wdt_priv *priv = dev_get_priv(dev);

	pi_wdt_stop(dev);

	/* set timer in miliseconds */
	pi_wdt_settimeout(priv->base, priv->clk_khz, timeout);

	pi_wdt_enable(priv->base);

	/* reset the watchdog */
	return pi_wdt_reset(dev);
}

static int pi_wdt_probe(struct udevice *dev)
{
	struct pi_wdt_priv *priv = dev_get_priv(dev);
	__maybe_unused int ret;

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

#if CONFIG_IS_ENABLED(CLK)
	struct clk clk;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret)
		return ret;

	ret = clk_enable(&clk);
	if (ret)
		goto err;

	priv->clk_khz = clk_get_rate(&clk) / 1000;
	if (!priv->clk_khz) {
		ret = -EINVAL;
		goto err;
	}
#else
	priv->clk_khz = CONFIG_DW_WDT_CLOCK_KHZ;
#endif

	if (CONFIG_IS_ENABLED(DM_RESET)) {
		ret = reset_get_bulk(dev, &priv->resets);
		if (ret)
			goto err;

		ret = reset_deassert_bulk(&priv->resets);
		if (ret)
			goto err;
	}

	/* reset to disable the watchdog */
	return pi_wdt_stop(dev);

err:
#if CONFIG_IS_ENABLED(CLK)
	clk_free(&clk);
#endif
	return ret;
}

static const struct wdt_ops pi_wdt_ops = {
	.start = pi_wdt_start,
	.reset = pi_wdt_reset,
	.stop = pi_wdt_stop,
};

static const struct udevice_id pi_wdt_ids[] = {
	{ .compatible = "snps,dw-wdt"},
	{ .compatible = "pi,pi-wdt"},
	{}
};

U_BOOT_DRIVER(pi_wdt) = {
	.name = "pi_wdt",
	.id = UCLASS_WDT,
	.of_match = pi_wdt_ids,
	.priv_auto	= sizeof(struct pi_wdt_priv),
	.probe = pi_wdt_probe,
	.ops = &pi_wdt_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
