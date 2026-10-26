// SPDX-License-Identifier: GPL-2.0-only
/* the firmware's shallow WFI path (power.dll FUN_80146f74), e6.idle=1 replaces nohlt */
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/seq_file.h>
#include <asm/cacheflush.h>
#include <asm/irqflags.h>
#include <asm/outercache.h>
#include <asm/system_misc.h>

#include "idle.h"

#define RY_CLOCK_READY BIT(10)
#define RY_CLOCK_BUSY GENMASK(15, 14)
#define RY_IDLE_CONSTRAINED BIT(3)

static bool requested, ready;
static void __iomem *clock, *l2;
static void *scratch;
static u64 entries, skipped, wait_ns;

void rapuyama_idle_wfi(void *scratch, void __iomem *l2_sync, void __iomem *clock);

static int __init rapuyama_idle_param(char *s){return kstrtobool(s, &requested);}
early_param("e6.idle", rapuyama_idle_param);

static void rapuyama_cpu_idle(void){
	unsigned long flags;
	u64 start;

	/* hlt can take effect before scratch memory and the device maps exist */
	if(!READ_ONCE(ready)){cpu_relax();return;}
	if(!(readl_relaxed(clock+0x130)&RY_CLOCK_READY) ||
		(readl_relaxed(clock+0x120)&RY_CLOCK_BUSY)){
		skipped++;
		return;
	}
	asm volatile("mrs %0, cpsr\n\tcpsid f" : "=r"(flags) : : "memory", "cc");
	/* power.dll's constrained path keeps the system clocks available */
	writel_relaxed(RY_IDLE_CONSTRAINED, clock+0x144);
	start=ktime_get_mono_fast_ns();
	rapuyama_idle_wfi(scratch, l2+0x730, clock);
	wait_ns+=ktime_get_mono_fast_ns()-start;
	entries++;
	if(!(flags&PSR_F_BIT)){local_fiq_enable();}
}

void __init rapuyama_idle_init_early(void){
	if(requested){arm_pm_idle=rapuyama_cpu_idle;}
}

static int rapuyama_idle_show(struct seq_file *f, void *unused){
	unsigned long flags;
	u64 n, skip, ns;

	local_irq_save(flags);
	n=entries;skip=skipped;ns=wait_ns;
	local_irq_restore(flags);
	seq_printf(f, "requested %u\nready %u\nentries %llu\nskipped %llu\nwait_ns %llu\n",
		requested, READ_ONCE(ready), n, skip, ns);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rapuyama_idle);

static int __init rapuyama_idle_init(void){
	struct page *page;
	void *cached;
	phys_addr_t phys;
	int ret;

	if(!of_machine_is_compatible("nokia,e6")){return 0;}
	debugfs_create_file("rapuyama-idle", 0444, NULL, NULL, &rapuyama_idle_fops);
	if(!requested){return 0;}
	clock=ioremap(0x20033000, 0x200);
	l2=ioremap(0x20500000, 0x1000);
	if(!clock || !l2){ret=-ENOMEM;goto unmap;}
	if(readl(l2)!=0x41000486 || !(readl(l2+0x100)&1)){
		ret=-ENODEV;
		goto unmap;
	}
	page=alloc_page(GFP_KERNEL|__GFP_ZERO);
	if(!page){ret=-ENOMEM;goto unmap;}
	cached=page_address(page);
	phys=page_to_phys(page);
	dmac_flush_range(cached, cached+PAGE_SIZE);
	outer_flush_range(phys, phys+PAGE_SIZE);
	/* firmware 0x11 is fully blocking; ARM DMA-coherent memory is buffered */
	scratch=vmap(&page, 1, VM_MAP, pgprot_stronglyordered(PAGE_KERNEL));
	if(!scratch){__free_page(page);ret=-ENOMEM;goto unmap;}
	/* all mappings stay for the lifetime of this built-in idle hook */
	WRITE_ONCE(ready, true);
	pr_info("rapuyama-idle: firmware shallow WFI ready\n");
	return 0;

unmap:
	if(clock){iounmap(clock);}
	if(l2){iounmap(l2);}
	clock=l2=NULL;
	pr_err("rapuyama-idle: preparation failed (%d), keeping polling idle\n", ret);
	return ret;
}
late_initcall(rapuyama_idle_init);