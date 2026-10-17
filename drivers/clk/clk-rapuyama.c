// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama clock gates (N8/E6/E7/C7)
 *
 * No public documentation exists, reversed from power_resources.dll of the
 * E6 RM-609 firmware. Two levels:
 *
 * - hw gates, built at boot as (reg, mask, polarity, gate id). Every gate reg
 *   R has R + 4 = set bits and R + 8 = clear bits, polarity 1 means set = on,
 *   polarity 0 means clear = on. Some gates are flipped through the modem
 *   (sisa) while its link is up, with the link down the firmware writes the
 *   same regs directly, which is what this does.
 * - PowerResourceManager resources (what drivers ask for, 1 based ids in the
 *   order of its name table), each one = up to 3 hw gates (table at
 *   0x800cbee8). The dt clock index is the resource id, e.g. 0x11 =
 *   EHwClkI2C1 (used by i2c0.dll), 0x1f = EHwClkBB38M4AnaClk1 (bt uart).
 *
 * Gates the firmware drives through other means (gpio driven clocks, the
 * irq driven usb power domain, plain software ones) are left out. The gate
 * regs are never read (the firmware doesn't either), so the state at boot is
 * whatever symbian left. Resources needed by drivers that don't enable their
 * clock themselves (i2c-omap only reads the rate, earlycon) are critical.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define RY_CGC_SET	0x4
#define RY_CGC_CLR	0x8
#define RY_GATE_MAX	0x30
#define RY_RES_MAX	0x3a

/* cgc base = phys 0x20032f00 (one gate sits at 0x20032fe0) */
struct ry_gate_desc {
	u16 reg;
	u16 mask;
	u8 pol;
	bool settle;	/* power domain, give it time after switching on */
};

/* gate id -> reg, from power_resources 0x800c7104.. (FUN_800c7534 calls) */
static const struct ry_gate_desc ry_gates[RY_GATE_MAX] = {
	[0x06] = { 0x1d0, 0x0010, 1 },
	[0x07] = { 0x1d0, 0x0040, 1 },
	[0x08] = { 0x1d0, 0x0080, 1 },
	[0x09] = { 0x1d0, 0x0001, 1 },
	[0x0a] = { 0x1d0, 0x0004, 1 },
	[0x0b] = { 0x1d0, 0x0008, 1 },
	[0x0c] = { 0x1d0, 0x0100, 1 },
	[0x0d] = { 0x1d0, 0x0400, 1 },
	[0x0e] = { 0x1d0, 0x0800, 1 },
	[0x0f] = { 0x180, 0x0020, 1 },
	[0x10] = { 0x180, 0x0040, 1 },
	[0x11] = { 0x180, 0x0800, 1 },
	[0x12] = { 0x1b0, 0x0010, 1 },
	[0x13] = { 0x1b0, 0x0020, 1 },
	[0x14] = { 0x1b0, 0x0040, 1 },
	/*
	 * hs usb power domains (FUN_800c7634): same set/clear reg pair, the
	 * firmware then waits for a done irq (0x114/0x115), a delay does here
	 */
	[0x15] = { 0x250, 0x2000, 0, true },
	[0x16] = { 0x250, 0x1000, 0, true },
	[0x17] = { 0x190, 0x0020, 1 },
	[0x18] = { 0x190, 0x0102, 1 },
	[0x19] = { 0x190, 0x0004, 1 },
	[0x1e] = { 0x150, 0x0008, 0 },
	[0x1f] = { 0x150, 0x0004, 0 },
	[0x20] = { 0x1d0, 0x0002, 1 },
	[0x21] = { 0x1d0, 0x2000, 1 },
	[0x22] = { 0x1d0, 0x1000, 1 },
	[0x23] = { 0x1d0, 0x0020, 1 },
	[0x24] = { 0x1d0, 0x8000, 1 },
	[0x25] = { 0x280, 0x0080, 1 },
	[0x26] = { 0x280, 0x0100, 1 },
	[0x27] = { 0x180, 0x0010, 1 },
	[0x28] = { 0x1b0, 0x0008, 1 },
	[0x29] = { 0x240, 0x0010, 0 },
	[0x2a] = { 0x150, 0x0200, 0 },
	[0x2b] = { 0x0e0, 0x0080, 1 },
	[0x2c] = { 0x190, 0x0200, 1 },
	[0x2d] = { 0x190, 0x0008, 1 },
};

struct ry_res_desc {
	u8 id;
	u8 gate[3];
	bool critical;
};

/* resource id -> gates, power_resources 0x800cbee8 */
static const struct ry_res_desc ry_res[] = {
	{ 0x0d, { 0x0f } },			/* EHwClkDomISP */
	{ 0x0f, { 0x15, 0x1e } },		/* EHwClkHSUSB */
	{ 0x0e, { 0x16, 0x1f, 0x18 } },		/* EHwClkDomHSUSB */
	{ 0x0a, { 0x01 } },			/* EHwClkEdCommon */
	{ 0x0b, { 0x02 } },			/* EHwClkExtSysClk */
	{ 0x0c, { 0x10 } },			/* EHwClkCamera */
	{ 0x11, { 0x07, 0x12 }, true },		/* EHwClkI2C1, i2c0.dll */
	{ 0x12, { 0x08, 0x13 } },		/* EHwClkI2C2 */
	{ 0x13, { 0x09, 0x14 } },		/* EHwClkIrDA1M */
	{ 0x14, { 0x0a, 0x03 } },		/* EHwClkIrDA4M */
	{ 0x15, { 0x0b } },			/* EHwClkLoSSI */
	{ 0x16, { 0x04 } },			/* EHwClkMeSSI */
	{ 0x1c, { 0x0c } },			/* EHwClkVGALite */
	{ 0x17, { 0x05 } },			/* EHwClkRFBI */
	{ 0x18, { 0x17 } },			/* EHwClkViSSI */
	{ 0x1a, { 0x0d } },			/* EHwClkSSI1 */
	{ 0x1b, { 0x0e } },			/* EHwClkMcuSpi */
	{ 0x19, { 0x17 } },			/* EHwClkSSI0 */
	{ 0x25, { 0x1a } },			/* EHwClkExtSysClk19M2 */
	{ 0x26, { 0x1b } },			/* EHwClkGDMA */
	{ 0x27, { 0x1c } },			/* EHwClkMemCard0 */
	{ 0x28, { 0x1d } },			/* EHwClkMemCard1 */
	{ 0x1e, { 0x20, 0x21, 0x2a } },		/* EHwClkBB38M4AnaClk0 */
	{ 0x1f, { 0x22 }, true },		/* EHwClkBB38M4AnaClk1, bt uart */
	{ 0x20, { 0x2b } },			/* EHwClkDspSpi0 */
	{ 0x21, { 0x27 } },			/* EHwClkDspSpi1 */
	{ 0x22, { 0x28 } },			/* EHwClkACodec */
	{ 0x10, { 0x06, 0x2c, 0x11 } },		/* EHwClkI2C0 */
	{ 0x23, { 0x23, 0x25 } },		/* EHwClkASIBT */
	{ 0x24, { 0x24, 0x26 } },		/* EHwClkASIMusti */
	{ 0x1d, { 0x19, 0x29 } },		/* ERegulatorVMEM */
	{ 0x33, { 0x2d } },			/* EHwClkEMC */
	{ 0x38, { 0x2e, 0x1a } },		/* EHwClkBTExtSysClk */
	{ 0x39, { 0x2f } },			/* EHwClkMemIf */
};

struct ry_clk {
	struct clk_hw hw;
	void __iomem *base;
	const struct ry_res_desc *res;
};

static DEFINE_SPINLOCK(ry_cgc_lock);

#define to_ry_clk(_hw) container_of(_hw, struct ry_clk, hw)

static bool ry_gate_known(u8 g)
{
	return g && g < RY_GATE_MAX && ry_gates[g].mask;
}

static void ry_gate_write(void __iomem *base, u8 g, bool on)
{
	const struct ry_gate_desc *d = &ry_gates[g];

	writel(d->mask, base + d->reg + (on == d->pol ? RY_CGC_SET : RY_CGC_CLR));
}

static int ry_clk_enable(struct clk_hw *hw)
{
	struct ry_clk *c = to_ry_clk(hw);
	unsigned long flags;
	int i;

	for (i = 0; i < 3; i++) {
		u8 g = c->res->gate[i];

		if (!ry_gate_known(g))
			continue;
		spin_lock_irqsave(&ry_cgc_lock, flags);
		ry_gate_write(c->base, g, true);
		spin_unlock_irqrestore(&ry_cgc_lock, flags);
		if (ry_gates[g].settle)
			udelay(500);
	}
	return 0;
}

static void ry_clk_disable(struct clk_hw *hw)
{
	struct ry_clk *c = to_ry_clk(hw);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ry_cgc_lock, flags);
	for (i = 2; i >= 0; i--)
		if (ry_gate_known(c->res->gate[i]))
			ry_gate_write(c->base, c->res->gate[i], false);
	spin_unlock_irqrestore(&ry_cgc_lock, flags);
}

static const struct clk_ops ry_clk_ops = {
	.enable		= ry_clk_enable,
	.disable	= ry_clk_disable,
};

static void __init rapuyama_cgc_init(struct device_node *np)
{
	struct clk_hw_onecell_data *data;
	struct clk_parent_data parent = { .index = 0 };
	void __iomem *base;
	int i, ret;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("rapuyama-cgc: can't map\n");
		return;
	}

	data = kzalloc(struct_size(data, hws, RY_RES_MAX), GFP_KERNEL);
	if (!data)
		return;
	data->num = RY_RES_MAX;
	for (i = 0; i < RY_RES_MAX; i++)
		data->hws[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < ARRAY_SIZE(ry_res); i++) {
		const struct ry_res_desc *r = &ry_res[i];
		struct clk_init_data init = { };
		struct ry_clk *c;
		char *name;

		c = kzalloc(sizeof(*c), GFP_KERNEL);
		name = kasprintf(GFP_KERNEL, "ry-res-%02x", r->id);
		if (!c || !name)
			return;

		c->base = base;
		c->res = r;
		init.name = name;
		init.ops = &ry_clk_ops;
		/* everything hangs off the first clock of the node for now */
		init.parent_data = &parent;
		init.num_parents = 1;
		init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED |
			     (r->critical ? CLK_IS_CRITICAL : 0);
		c->hw.init = &init;

		ret = clk_hw_register(NULL, &c->hw);
		if (ret) {
			pr_err("rapuyama-cgc: res %#x: %d\n", r->id, ret);
			continue;
		}
		data->hws[r->id] = &c->hw;
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, data);
	if (ret)
		pr_err("rapuyama-cgc: provider: %d\n", ret);
}
CLK_OF_DECLARE(rapuyama_cgc, "nokia,rapuyama-cgc", rapuyama_cgc_init);