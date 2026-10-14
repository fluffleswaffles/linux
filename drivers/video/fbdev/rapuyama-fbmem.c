// SPDX-License-Identifier: GPL-2.0-only
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>

struct rapuyama_fbmem {
	struct resource resource;
	struct resource *mem;
	bool mem_claimed;
	u32 pseudo_palette[16];
};

static int rapuyama_fbmem_setcolreg(unsigned int regno, unsigned int red, unsigned int green, unsigned int blue, unsigned int transp, struct fb_info *info)
{
	struct rapuyama_fbmem *fb = info->par;
	u32 value;

	if (regno >= ARRAY_SIZE(fb->pseudo_palette))
		return -EINVAL;

	value = (red >> (16 - info->var.red.length)) << info->var.red.offset;
	value |= (green >> (16 - info->var.green.length)) <<
		 info->var.green.offset;
	value |= (blue >> (16 - info->var.blue.length)) <<
		 info->var.blue.offset;
	if (info->var.transp.length)
		value |= (transp >> (16 - info->var.transp.length)) <<
			 info->var.transp.offset;

	fb->pseudo_palette[regno] = value;
	return 0;
}

static const struct fb_ops rapuyama_fbmem_ops = {
	.owner = THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
	.fb_setcolreg = rapuyama_fbmem_setcolreg,
};

static int rapuyama_fbmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *mem_np;
	struct rapuyama_fbmem *fb;
	struct fb_info *info;
	struct resource res;
	resource_size_t size;
	u32 width, height, stride, bpp;
	u64 row_bytes, required;
	int ret;

	ret = of_property_read_u32(dev->of_node, "width", &width);
	if (ret)
		return dev_err_probe(dev, ret, "missing width\n");
	ret = of_property_read_u32(dev->of_node, "height", &height);
	if (ret)
		return dev_err_probe(dev, ret, "missing height\n");
	ret = of_property_read_u32(dev->of_node, "stride", &stride);
	if (ret)
		return dev_err_probe(dev, ret, "missing stride\n");
	ret = of_property_read_u32(dev->of_node, "bits-per-pixel", &bpp);
	if (ret)
		return dev_err_probe(dev, ret, "missing bits-per-pixel\n");

	if (!width || !height || (bpp != 16 && bpp != 32))
		return dev_err_probe(dev, -EINVAL, "unsupported framebuffer mode\n");

	row_bytes = (u64)width * bpp;
	row_bytes = (row_bytes + 7) / 8;
	required = (u64)stride * height;
	if (stride < row_bytes)
		return dev_err_probe(dev, -EINVAL, "invalid framebuffer stride or size\n");

	mem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_np)
		return dev_err_probe(dev, -EINVAL, "missing memory-region\n");
	ret = of_address_to_resource(mem_np, 0, &res);
	of_node_put(mem_np);
	if (ret)
		return dev_err_probe(dev, ret, "invalid memory-region\n");

	size = resource_size(&res);
	if (size > U32_MAX || required > size)
		return dev_err_probe(dev, -EINVAL, "framebuffer mode exceeds memory-region\n");

	info = framebuffer_alloc(sizeof(*fb), dev);
	if (!info)
		return -ENOMEM;
	fb = info->par;
	fb->resource = res;
	fb->mem = request_mem_region(res.start, size, dev_name(dev));
	if (!fb->mem) {
		dev_warn(dev, "cannot claim reserved framebuffer resource %pR\n",
			 &res);
		fb->mem = &fb->resource;
	} else {
		fb->mem_claimed = true;
	}

	info->screen_base = ioremap_wc(res.start, size);
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto err_release;
	}
	info->screen_size = size;
	info->pseudo_palette = fb->pseudo_palette;
	info->fbops = &rapuyama_fbmem_ops;
	info->flags = FBINFO_HWACCEL_DISABLED;

	strscpy(info->fix.id, "rapuyama-fbmem", sizeof(info->fix.id));
	info->fix.smem_start = res.start;
	info->fix.smem_len = size;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.line_length = stride;

	info->var.xres = width;
	info->var.yres = height;
	info->var.xres_virtual = width;
	info->var.yres_virtual = height;
	info->var.bits_per_pixel = bpp;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;
	if (bpp == 16) {
		info->var.red = (struct fb_bitfield){ .offset = 11, .length = 5 };
		info->var.green = (struct fb_bitfield){ .offset = 5, .length = 6 };
		info->var.blue = (struct fb_bitfield){ .offset = 0, .length = 5 };
	} else {
		info->var.red = (struct fb_bitfield){ .offset = 16, .length = 8 };
		info->var.green = (struct fb_bitfield){ .offset = 8, .length = 8 };
		info->var.blue = (struct fb_bitfield){ .offset = 0, .length = 8 };
		info->var.transp = (struct fb_bitfield){ .offset = 24, .length = 8 };
	}

	platform_set_drvdata(pdev, info);
	ret = register_framebuffer(info);
	if (ret)
		goto err_unmap;

	dev_info(dev, "mapped %ux%u %u-bpp staging buffer at %pa; panel refresh is not implemented\n",
		 width, height, bpp, &res.start);
	return 0;

err_unmap:
	iounmap(info->screen_base);
err_release:
	if (fb->mem_claimed)
		release_mem_region(fb->mem->start, resource_size(fb->mem));
	framebuffer_release(info);
	return ret;
}

static void rapuyama_fbmem_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct rapuyama_fbmem *fb = info->par;

	unregister_framebuffer(info);
	iounmap(info->screen_base);
	if (fb->mem_claimed)
		release_mem_region(fb->mem->start, resource_size(fb->mem));
	framebuffer_release(info);
}

static const struct of_device_id rapuyama_fbmem_of_match[] = {
	{ .compatible = "nokia,rapuyama-fbmem" },
	{}
};
MODULE_DEVICE_TABLE(of, rapuyama_fbmem_of_match);

static struct platform_driver rapuyama_fbmem_driver = {
	.probe = rapuyama_fbmem_probe,
	.remove_new = rapuyama_fbmem_remove,
	.driver = {
		.name = "rapuyama-fbmem",
		.of_match_table = rapuyama_fbmem_of_match,
	},
};
module_platform_driver(rapuyama_fbmem_driver);

MODULE_DESCRIPTION("RapuYama reserved framebuffer memory interface");
MODULE_LICENSE("GPL");
