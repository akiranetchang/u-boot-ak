// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Sifive, Inc.
 * Author: Sagar Kadam <sagar.kadam@sifive.com>
 */

#include <common.h>
#include <dm.h>
#include <reset-uclass.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <linux/bitops.h>

#define PRCI_RESETREG_OFFSET 0x28

struct pi_reset_priv {
	void *base;
	/* number of reset signals */
	int nr_reset;
};

static int pi_rst_trigger(struct reset_ctl *rst, bool level)
{
	struct pi_reset_priv *priv = dev_get_priv(rst->dev);
	int id = rst->id;
	int regval = readl(priv->base + PRCI_RESETREG_OFFSET);

	/* Derive bitposition from rst id */
	if (level)
		/* Reset deassert */
		regval |= BIT(id);
	else
		/* Reset assert */
		regval &= ~BIT(id);

	writel(regval, priv->base + PRCI_RESETREG_OFFSET);

	return 0;
}

static int pi_reset_assert(struct reset_ctl *rst)
{
	return pi_rst_trigger(rst, false);
}

static int pi_reset_deassert(struct reset_ctl *rst)
{
	return pi_rst_trigger(rst, true);
}

static int pi_reset_request(struct reset_ctl *rst)
{
	struct pi_reset_priv *priv = dev_get_priv(rst->dev);

	debug("%s(rst=%p) (dev=%p, id=%lu) (nr_reset=%d)\n", __func__,
	      rst, rst->dev, rst->id, priv->nr_reset);

	if (rst->id > priv->nr_reset)
		return -EINVAL;

	return 0;
}

static int pi_reset_probe(struct udevice *dev)
{
	struct pi_reset_priv *priv = dev_get_priv(dev);

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -ENOMEM;

	return 0;
}

int pi_reset_bind(struct udevice *dev, ulong count)
{
	struct udevice *rst_dev;
	struct pi_reset_priv *priv;
	int ret;

	ret = device_bind_driver_to_node(dev, "pi-reset", "reset",
					 dev_ofnode(dev), &rst_dev);
	if (ret) {
		dev_err(dev, "failed to bind pi_reset driver (ret=%d)\n", ret);
		return ret;
	}
	priv = malloc(sizeof(struct pi_reset_priv));
	priv->nr_reset = count;
	dev_set_priv(rst_dev, priv);

	return 0;
}

const struct reset_ops pi_reset_ops = {
	.request = pi_reset_request,
	.rst_assert = pi_reset_assert,
	.rst_deassert = pi_reset_deassert,
};

U_BOOT_DRIVER(pi_reset) = {
	.name		= "pi-reset",
	.id		= UCLASS_RESET,
	.ops		= &pi_reset_ops,
	.probe		= pi_reset_probe,
	.priv_auto	= sizeof(struct pi_reset_priv),
};
