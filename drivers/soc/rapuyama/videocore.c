// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

static int rapuyama_videocore_probe(struct platform_device *pdev)
{
	return dev_err_probe(&pdev->dev, -EOPNOTSUPP, "VideoCore transport is not implemented yet\n");
}

static const struct of_device_id rapuyama_videocore_of_match[] = {
	{ .compatible = "nokia,rapuyama-videocore" }, {}
};
MODULE_DEVICE_TABLE(of, rapuyama_videocore_of_match);

static struct platform_driver rapuyama_videocore_driver = {
	.probe = rapuyama_videocore_probe,
	.driver = {
		.name = "rapuyama-videocore",
		.of_match_table = rapuyama_videocore_of_match,
	},
};
module_platform_driver(rapuyama_videocore_driver);

MODULE_DESCRIPTION("Nokia RapuYama VideoCore transport scaffold");
MODULE_LICENSE("GPL");
