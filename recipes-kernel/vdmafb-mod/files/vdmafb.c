/*
 * AXI VDMA frame buffer driver
 * Based on: ocfb.c, simplefb.c, axi_hdmi_crtc.c
 *
 * Copyright (C) 2014 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 * 
 * Modified: Dan McGraw <dan.m0wut@gmail.com>
 * Modifications to get compatibility with VDMA v6.3
 * 
 * Licensed under the GPL-2.
 *
 * dma_address: IO address for the VDMA control registers
 * 
 * resolution: 	string, changing this requires the video section to be clocked
 * 				at different speed. All supported options are listed
 * 
 * resolution	Clock Speed (MHz)	Horizontal pixels	Vertical lines
 * "720p" 		74.25				1280				720
 * "1080p"		165					1920				1080
 * 
 * format: bits per pixel (bpp) and how they are packed. VDMA settings will need to match
 * All supported options are listed (VDMA really likes bytes per pixel to be a power of 2)
 * 
 * format		total bpp	transparency bits(MSB)	red bits	green bits	blue bits(LSB)	
 * "rgba"		32 			8						8			8			8
 * "rgb565"		16			0						5			6			5
 * 
 * Example devicetree contents:
 		axi_vdma_vga: axi_vdma_vga@7e000000 {
			compatible = "topic,vdma-fb";
			reg = <0x7e000000 0x10000>;
			dmas = <&axi_vdma_0 0>;		
			dma_address = <0x43000000>;
			num-fstores = <1>;
			resolution = "720p"
			format = "rgba"
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
#define VDMAFB_CONTROL_OFFSET		0x00
#define VDMAFB_STATUS_OFFSET		0x04
#define VDMAFB_VERSION_OFFSET		0x2C
//Driver only support MM2S (DMA out)
#define VDMAFB_VSIZE_OFFSET 		0x50
#define VDMAFB_HSIZE_OFFSET 		0x54
#define VDMAFB_STRIDE_OFFSET 		0x58
#define VDMAFB_START_ADDRESS_BASE 	0x5C

#define DEBUG 1


struct vdmafb_dev {
	struct fb_info info;
	// Base Address for VDMA control registers
	uint32_t vdma_base_addr;
	void __iomem *regs;
	// Physical and virtual addresses of framebuffer 
	dma_addr_t fb_phys;
	void __iomem *fb_virt;
	// Number of Framebuffers
	u32 frames;
	// Palette data 
	u32 pseudo_palette[16];
};

static inline u32 vdmafb_readreg(struct vdmafb_dev* dev, uint32_t offset)
{
	uint32_t data =  *((uint32_t*)(dev->regs + offset));
	return data;
}

static inline void vdmafb_writereg(struct vdmafb_dev* dev, uint32_t offset, u32 data)
{
	#if DEBUG
		printk(KERN_INFO "Writing data %u to address %x\n", data, offset);
	#endif
	*((uint32_t*)(dev->regs + offset)) = data;
}

static int vdmafb_setup_vdma(struct vdmafb_dev* fbdev)
{
	struct fb_var_screeninfo *var = &fbdev->info.var;
	// Enable display - DMA engine doesn't start until VSIZE has been written 
	u32 version_reg = vdmafb_readreg(fbdev, VDMAFB_VERSION_OFFSET);
	int major = (version_reg >> 28) & 0x0F;
	int minor = (version_reg >> 20) & 0xFF;
	if (major != 6 || minor != 32) {
		printk(KERN_ERR "VDMA version %d.%d is incompatible with driver which expects v6.32\n", major, minor);
		printk(KERN_ERR "This may also be due to an incorrect VDMA base address being supplied\n");
		return -EINVAL;
	}
	
	#if DEBUG
		printk(KERN_INFO "Status register pre-config: %d", vdmafb_readreg(fbdev, VDMAFB_STATUS_OFFSET));
	#endif	

	vdmafb_writereg(fbdev, VDMAFB_CONTROL_OFFSET, 0x03);
	vdmafb_writereg(fbdev, VDMAFB_START_ADDRESS_BASE, (uint32_t)fbdev->fb_phys);
	// These are line_length not var->xres as it's bytes per line, not pixels
	vdmafb_writereg(fbdev, VDMAFB_STRIDE_OFFSET, fbdev->info.fix.line_length);
	vdmafb_writereg(fbdev, VDMAFB_HSIZE_OFFSET, fbdev->info.fix.line_length);
	vdmafb_writereg(fbdev, VDMAFB_VSIZE_OFFSET, var->yres);

	#if DEBUG
		printk(KERN_INFO "Status register post-config: %d", vdmafb_readreg(fbdev, VDMAFB_STATUS_OFFSET));
		msleep(1000);
		printk(KERN_INFO "Status register post-sleep: %d", vdmafb_readreg(fbdev, VDMAFB_STATUS_OFFSET));
	#endif
	return 0;
}

static void vdmafb_init_fix(struct vdmafb_dev *fbdev)
{
	struct fb_var_screeninfo *var = &fbdev->info.var;
	struct fb_fix_screeninfo *fix = &fbdev->info.fix;

	strcpy(fix->id, "vdma-fb");
	fix->line_length = var->xres * (var->bits_per_pixel/8);
	fix->smem_len = fix->line_length * var->yres * fbdev->frames;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->visual = FB_VISUAL_TRUECOLOR;
}

static void vdmafb_init_var(struct vdmafb_dev *fbdev, struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fb_var_screeninfo *var = &fbdev->info.var;
	int ret;

	ret = of_property_read_u32(np, "dma_address", &fbdev->vdma_base_addr);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse VDMA address. Assume 0x43000000\n");
		fbdev->vdma_base_addr = 0x43000000;
	}
	#if DEBUG
		printk(KERN_INFO "VDMA Base Address: %#010x", fbdev->vdma_base_addr);
	#endif

	const char *resolution_string;
	ret = of_property_read_string(np, "resolution", &resolution_string);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse resolution\n");
	}

	// Resolution settings defined here
	if (!strcmp(resolution_string, "720p"))
	{
		var->xres = 1280;
		var->yres = 720;
		var->pixclock = KHZ2PICOS(74250);
	}
	else if (!strcmp(resolution_string, "1080p"))
	{
		var->xres = 1920;
		var->yres = 1080;
		var->pixclock = KHZ2PICOS(165000);
	}
	else
	{
		dev_err(&pdev->dev, "Unsupported resolution passed to VDMA module\n");
	}
	
	printk(KERN_INFO "Framebuffer resolution set to %u x %u", var->xres, var->yres);


	/*
	 * Xilinx VDMA requires clients to submit exactly the number of frame
	 * stores, but doesn't supply a way to retrieve that number. Pass the
	 * xlnx,num-fstores value of the VDMA node to num-fstores here.
	 */
	ret = of_property_read_u32(np, "num-fstores", &fbdev->frames);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse num-fstores property, assume 1\n");
		fbdev->frames = 1;
	}
	#if DEBUG
		printk(KERN_INFO "Read number of framebuffers as %u", fbdev->frames);
	#endif

	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;

	// Fixed Framebuffer settings
	var->vmode = FB_VMODE_NONINTERLACED;
	var->accel_flags = FB_ACCEL_NONE;
	var->activate = FB_ACTIVATE_NOW;

	// Colour per pixel settings
	const char *format_string;
	ret = of_property_read_string(np, "format", &format_string);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse pixel format\n");
	}

	// Format settings defined here
	if (!strcmp(format_string, "rgba"))
	{
		var->bits_per_pixel = 32;
		var->transp.length = 8;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
	}
	else if (!strcmp(format_string, "rgb565"))
	{
		var->bits_per_pixel = 16;
		var->transp.length = 0;
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
	}
	else
	{
		dev_err(&pdev->dev, "Unsupported pixel format passed to VDMA module\n");
	}
	
	printk(KERN_INFO "Pixel format set to %s", format_string);
	
	// I've assumed that order is always (MSB -> LSB) transp, red, green, blue 
	var->blue.offset = 0;
	var->green.offset = var->blue.length;
	var->red.offset = var->green.offset + var->green.length;
	var->transp.offset = var->red.offset + var->red.length;
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

	printk(KERN_INFO "Starting to load framebuffer driver, wish me luck!\n");
	fbdev = devm_kzalloc(&pdev->dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;
	
	platform_set_drvdata(pdev, fbdev);

	fbdev->info.fbops = &vdmafb_ops;
	fbdev->info.device = &pdev->dev;
	fbdev->info.par = fbdev;

	vdmafb_init_var(fbdev, pdev);
	vdmafb_init_fix(fbdev);
	
	// Request I/O resource 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "I/O resource request failed\n");
		return -ENXIO;
	}
	res->flags &= ~IORESOURCE_CACHEABLE;

	// Map VDMA control registers into virtual memory
	fbdev->regs = ioremap(fbdev->vdma_base_addr, SZ_4K);
	if (IS_ERR(fbdev->regs))
		return PTR_ERR(fbdev->regs);
	
	// Sanity check Framebuffer size
	if (!fbdev->info.fix.smem_len) {
		dev_err(&pdev->dev, "Framebuffer size must be non-zero");
	}

	// Allocate Framebuffer Memory
	fbdev->fb_virt = dma_alloc_coherent(&pdev->dev, fbdev->info.fix.smem_len, &fbdev->fb_phys, GFP_KERNEL);
	if (!fbdev->fb_virt) {
		dev_err(&pdev->dev,"Frame buffer memory allocation failed\n");
		return -ENOMEM;
	}
	
	fbdev->info.fix.smem_start = fbdev->fb_phys;
	fbdev->info.screen_base = fbdev->fb_virt;
	fbdev->info.pseudo_palette = fbdev->pseudo_palette;

	// Clear framebuffer 
	memset_io(fbdev->fb_virt, 0, fbdev->info.fix.smem_len);

	// Setup and enable the framebuffer 
	if (vdmafb_setup_vdma(fbdev))
		return EINVAL; // Version incompatability


	//Create Colour map information
	ret = fb_alloc_cmap(&fbdev->info.cmap, 256, 0);
	if (ret) {
		dev_err(&pdev->dev, "fb_alloc_cmap failed\n");
	}

	// Register framebuffer
	ret = register_framebuffer(&fbdev->info);
	if (ret) {
		dev_err(&pdev->dev, "Framebuffer registration failed\n");
		goto err_dma_free;
	}

	return 0;


err_dma_free:
	dma_free_coherent(&pdev->dev, fbdev->info.fix.smem_len, fbdev->fb_virt,
			  fbdev->fb_phys);

}

static int vdmafb_remove(struct platform_device *pdev)
{
	struct vdmafb_dev *fbdev = platform_get_drvdata(pdev);
	unregister_framebuffer(&fbdev->info);
	/* Disable display */
	vdmafb_writereg(fbdev, VDMAFB_CONTROL_OFFSET, 3);
	dma_free_coherent(&pdev->dev, fbdev->info.fix.smem_len,
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
