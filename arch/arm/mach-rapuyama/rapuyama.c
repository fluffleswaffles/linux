// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama (N8/E6/E7/C7) machine
 *
 * Mostly a plain DT machine. It also maps the first 1 MB of the SoC bus
 * (phys 0x20000000: cbus master, clock/power block, soc reset) at a fixed
 * address so early code can reach it, which the bring-up checkpoints use:
 * with "e6.stop=N" on the command line the kernel powers the phone off (pmic
 * watchdog expire over cbus, like symbian's force power off) when it reaches
 * checkpoint N. Phone switching off right away = got there, the usual soc
 * watchdog reset ~12 s in = died before it. No console needed.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/irqflags.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define RY_BUS_PHYS	0x20000000
#define RY_BUS_VIRT	0xf8000000
#define RY_BUS(off)	((void __iomem *)(RY_BUS_VIRT + (off)))

/* cbus master, offsets from karap 0x800b8494 (base ptr 0x20030704 there) */
#define RY_CBUS		0x30700
#define CBUS_EN		0x04
#define CBUS_BUSY	0x08
#define CBUS_WR_CMD	0x1c
#define CBUS_DATA	0x20
#define CBUS_BITS	0x2c
#define CBUS_WR_MODE	0x3c

static struct map_desc rapuyama_io_desc[] __initdata = {
	{
		.virtual	= RY_BUS_VIRT,
		.pfn		= __phys_to_pfn(RY_BUS_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE,
	},
};

static int rapuyama_stop;
static bool rapuyama_mapped;

static int __init rapuyama_stop_param(char *s)
{
	return kstrtoint(s, 0, &rapuyama_stop);
}
early_param("e6.stop", rapuyama_stop_param);

static void __init rapuyama_map_io(void)
{
	iotable_init(rapuyama_io_desc, ARRAY_SIZE(rapuyama_io_desc));
	rapuyama_mapped = true;
}

/* pmic dev 1 reg 0x17 (watchdog) = 0 -> the pmic switches the phone off */
static void rapuyama_pmic_off(void)
{
	void __iomem *cb = RY_BUS(RY_CBUS);
	int n = 0x10000;

	writel(1, cb + CBUS_EN);
	writel(1, cb + CBUS_WR_MODE);
	writel(15, cb + CBUS_BITS);
	writel(0, cb + CBUS_DATA);
	writel((1 << 6) | 0x17, cb + CBUS_WR_CMD);
	while (readl(cb + CBUS_BUSY) && --n)
		cpu_relax();
	writel(0, cb + CBUS_EN);
}

void rapuyama_checkpoint(int n);

void rapuyama_checkpoint(int n)
{
	if (!rapuyama_mapped || n != rapuyama_stop)
		return;
	local_irq_disable();
	rapuyama_pmic_off();
	for (;;)
		cpu_relax();
}

static const char *const rapuyama_dt_compat[] __initconst = {
	"nokia,rapuyama",
	NULL
};

DT_MACHINE_START(RAPUYAMA, "Nokia RapuYama")
	.dt_compat	= rapuyama_dt_compat,
	.map_io		= rapuyama_map_io,
MACHINE_END