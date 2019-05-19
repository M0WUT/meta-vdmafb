/*
 * AXI VDMA frame buffer driver
 * Based on: ocfb.c, simplefb.c, axi_hdmi_crtc.c
 *
 * Copyright (C) 2014 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 *
 * Licensed under the GPL-2.
 *
 * Example devicetree contents:
 		axi_vdma_vga: axi_vdma_vga@7e000000 {
			compatible = "topic,vdma-fb";
			reg = <0x7e000000 0x10000>;
			dmas = <&axi_vdma_0 0>;
			dma-names = "video";
			width = <1024>;
			height = <600>;
			horizontal-front-porch = <160>;
			horizontal-back-porch = <160>;
			horizontal-sync = <136>;
			vertical-front-porch = <17>;
			vertical-back-porch = <18>;
		};
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/dma/xilinx_dma.h>

/* Register locations */
#define VDMAFB_BASE_ADDRESS 0x43000000
#define VDMAFB_CONTROL_OFFSET		0x00
#define VDMAFB_STATUS_OFFSET		0x04
#define VDMAFB_VERSION_OFFSET		0x2C
//Driver only support MM2S (DMA out)
#define VDMAFB_VSIZE_OFFSET 		0x50
#define VDMAFB_HSIZE_OFFSET 		0x54
#define VDMAFB_STRIDE_OFFSET 		0x58
#define VDMAFB_START_ADDRESS_BASE 	0x5C


static void __iomem *base = NULL;




struct vdmafb_dev {
	struct backlight_device *backlight;
	struct fb_info info;
	void __iomem *regs;
	/* Physical and virtual addresses of framebuffer */
	phys_addr_t fb_phys;
	void __iomem *fb_virt;
	/* VDMA handle */
	struct dma_chan *dma;
	struct dma_interleaved_template *dma_template;
	u32 frames;
	/* Palette data */
	u32 pseudo_palette[16];
};

static inline u32 vdmafb_readreg(void* base, uint32_t offset)
{
	uint32_t data =  *((uint32_t*)(base + offset));
	return data;
}

static inline void vdmafb_writereg(void* base, uint32_t offset, u32 data)
{
	printk(KERN_INFO "Writing data %u to address %x\n", data, offset);
	*((uint32_t*)(base + offset)) = data;
}


static int vdmafb_setupfb(void *base, dma_addr_t dma_address)
{

	/* Enable display - DMA engine doesn't start until VSIZE has been written */
	u32 version_reg = vdmafb_readreg(base, VDMAFB_VERSION_OFFSET);
	int major = (version_reg >> 28) & 0x0F;
	int minor = (version_reg >> 20) & 0xFF;
	printk("VDMA version %d.%d\n", major, minor);
	printk(KERN_INFO "Status register pre-config: %d", vdmafb_readreg(base, VDMAFB_STATUS_OFFSET));
	vdmafb_writereg(base, VDMAFB_CONTROL_OFFSET, 0x03);
	vdmafb_writereg(base, VDMAFB_START_ADDRESS_BASE, (uint32_t)dma_address);
	vdmafb_writereg(base, VDMAFB_STRIDE_OFFSET, 1280 * 4);
	vdmafb_writereg(base, VDMAFB_HSIZE_OFFSET, 1280 * 4);
	vdmafb_writereg(base, VDMAFB_VSIZE_OFFSET, 720);
	printk(KERN_INFO "Status register post-config: %d", vdmafb_readreg(base, VDMAFB_STATUS_OFFSET));
	msleep(1000);
	printk(KERN_INFO "Status register post-sleep: %d", vdmafb_readreg(base, VDMAFB_STATUS_OFFSET));
	
	return 0;
}

static void vdmafb_init_fix(struct vdmafb_dev *fbdev)
{
	struct fb_var_screeninfo *var = &fbdev->info.var;
	struct fb_fix_screeninfo *fix = &fbdev->info.fix;

	strcpy(fix->id, "vdma-fb");
	fix->line_length = var->xres * (var->bits_per_pixel/8);
	fix->smem_len = fix->line_length * var->yres;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->visual = FB_VISUAL_TRUECOLOR;
}

static void vdmafb_init_var(struct vdmafb_dev *fbdev, struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fb_var_screeninfo *var = &fbdev->info.var;
	int ret;

	ret = of_property_read_u32(np, "width", &var->xres);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property, assume 1024\n");
		var->xres = 1024;
	}

	ret = of_property_read_u32(np, "height", &var->yres);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property, assume 768\n");
		var->yres = 768;
	}

	/*
	 * Xilinx VDMA requires clients to submit exactly the number of frame
	 * stores, but doesn't supply a way to retrieve that number. Pass the
	 * xlnx,num-fstores value of the VDMA node to num-fstores here.
	 */
	ret = of_property_read_u32(np, "num-fstores", &fbdev->frames);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse num-fstores property, assume 3\n");
		fbdev->frames = 1;
	}

	var->accel_flags = FB_ACCEL_NONE;
	var->activate = FB_ACTIVATE_NOW;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->bits_per_pixel = 32;
	/* Clock settings */
	var->pixclock = KHZ2PICOS(51200);
	var->vmode = FB_VMODE_NONINTERLACED;
	of_property_read_u32(np, "horizontal-sync", &var->hsync_len);
	of_property_read_u32(np, "horizontal-front-porch", &var->left_margin);
	of_property_read_u32(np, "horizontal-back-porch", &var->right_margin);
	of_property_read_u32(np, "vertical-sync", &var->vsync_len);
	of_property_read_u32(np, "vertical-front-porch", &var->upper_margin);
	of_property_read_u32(np, "vertical-back-porch", &var->lower_margin);
	/* TODO: sync */
	/* 32 BPP */
	var->transp.offset = 24;
	var->transp.length = 8;
	var->red.offset = 16;
	var->red.length = 8;
	var->green.offset = 8;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
}

static int vdmafb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= 16)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

static struct fb_ops vdmafb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= vdmafb_setcolreg,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
};

static int vdmafb_probe(struct platform_device *pdev)
{
	
	int ret = 0;
	struct vdmafb_dev *fbdev;
	struct resource *res;
	int fbsize;
	struct backlight_properties props;
	struct backlight_device *bl;

	printk(KERN_INFO "Starting to load framebuffer driver, wish me luck!\n");
	fbdev = devm_kzalloc(&pdev->dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;
	/*
	platform_set_drvdata(pdev, fbdev);

	fbdev->info.fbops = &vdmafb_ops;
	fbdev->info.device = &pdev->dev;
	fbdev->info.par = fbdev;

	fbdev->dma_template = devm_kzalloc(&pdev->dev,
		sizeof(struct dma_interleaved_template) +
		sizeof(struct data_chunk), GFP_KERNEL);
	if (!fbdev->dma_template)
		return -ENOMEM;

	vdmafb_init_var(fbdev, pdev);
	vdmafb_init_fix(fbdev);

	// Request I/O resource 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "I/O resource request failed\n");
		return -ENXIO;
	}
	res->flags &= ~IORESOURCE_CACHEABLE;
	//fbdev->regs = devm_ioremap_resource(&pdev->dev, res);
	fbdev->regs = ioremap(VDMAFB_BASE_ADDRESS, SZ_4K);
	if (IS_ERR(fbdev->regs))
		return PTR_ERR(fbdev->regs);

	// Allocate framebuffer memory 
	fbsize = fbdev->info.fix.smem_len;
	fbdev->fb_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(fbsize),
					    &fbdev->fb_phys, GFP_KERNEL);
	if (!fbdev->fb_virt) {
		dev_err(&pdev->dev,
			"Frame buffer memory allocation failed\n");
		return -ENOMEM;
	}
	fbdev->info.fix.smem_start = fbdev->fb_phys;
	fbdev->info.screen_base = fbdev->fb_virt;
	fbdev->info.pseudo_palette = fbdev->pseudo_palette;

	pr_debug("%s virt=%p phys=%x size=%d\n", __func__,
		fbdev->fb_virt, fbdev->fb_phys, fbsize);

	// Clear framebuffer 
	memset_io(fbdev->fb_virt, 0, fbsize);

	
	fbdev->dma = dma_request_slave_channel(&pdev->dev, "video");
	if (IS_ERR_OR_NULL(fbdev->dma)) {
		dev_err(&pdev->dev, "Failed to allocate DMA channel (%d).\n", ret);
		if (fbdev->dma)
			ret = PTR_ERR(fbdev->dma);
		else
			ret = -EPROBE_DEFER;
		goto err_dma_free;
	}
	*/
	
	// Setup and enable the framebuffer 
	base = ioremap(VDMAFB_BASE_ADDRESS, SZ_4K);
	dma_addr_t dma_address;
	dma_alloc_coherent(&pdev->dev, 1280 * 720 * 4, &dma_address, GFP_KERNEL);
	vdmafb_setupfb(base, dma_address);
	/*
	ret = fb_alloc_cmap(&fbdev->info.cmap, 256, 0);
	if (ret) {
		dev_err(&pdev->dev, "fb_alloc_cmap failed\n");
	}

	// Register framebuffer 
	ret = register_framebuffer(&fbdev->info);
	if (ret) {
		dev_err(&pdev->dev, "Framebuffer registration failed\n");
		goto err_channel_free;
	}
	*/
	/* Register backlight */
	/*
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 1023;
	bl = backlight_device_register("backlight", &pdev->dev, fbdev,
				       &vdmafb_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "error %ld on backlight register\n",
				PTR_ERR(bl));
	} else {
		fbdev->backlight = bl;
		bl->props.power = FB_BLANK_UNBLANK;
		bl->props.fb_blank = FB_BLANK_UNBLANK;
		bl->props.brightness = vdmafb_bl_get_brightness(bl);
	}
	*/
	return 0;

/*
err_dma_free:
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(fbsize), fbdev->fb_virt,
			  fbdev->fb_phys);
*/

}

static int vdmafb_remove(struct platform_device *pdev)
{
	struct vdmafb_dev *fbdev = platform_get_drvdata(pdev);
	/*
	if (fbdev->backlight)
		backlight_device_unregister(fbdev->backlight);
	*/
	unregister_framebuffer(&fbdev->info);
	/* Disable display */
	//vdmafb_writereg(fbdev, VDMAFB_BACKLIGHT_CONTROL, 0);
	vdmafb_writereg(fbdev, VDMAFB_CONTROL_OFFSET, 3);
	dma_release_channel(fbdev->dma);
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(fbdev->info.fix.smem_len),
			  fbdev->fb_virt, fbdev->fb_phys);
	fb_dealloc_cmap(&fbdev->info.cmap);
	return 0;
}

static struct of_device_id vdmafb_match[] = {
	{ .compatible = "topic,vdma-fb", },
	{},
};
MODULE_DEVICE_TABLE(of, vdmafb_match);

static struct platform_driver vdmafb_driver = {
	.probe  = vdmafb_probe,
	.remove	= vdmafb_remove,
	.driver = {
		.name = "vdmafb_fb",
		.of_match_table = vdmafb_match,
	}
};
module_platform_driver(vdmafb_driver);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_DESCRIPTION("Driver for VDMA controlled framebuffer");
MODULE_LICENSE("GPL v2");
