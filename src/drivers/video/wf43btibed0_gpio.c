/*
 * linux/drivers/video/hecubafb.c -- FB driver for Hecuba controller
 *
 * Copyright (C) 2006, Jaya Kumar
 * This work was sponsored by CIS(M) Sdn Bhd
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Layout is based on skeletonfb.c by James Simmons and Geert Uytterhoeven.
 * This work was possible because of apollo display code from E-Ink's website
 * http://support.eink.com/community
 * All information used to write this code is from public material made
 * available by E-Ink on its support site. Some commands such as 0xA4
 * were found by looping through cmd=0x00 thru 0xFF and supplying random
 * values. There are other commands that the display is capable of,
 * beyond the 5 used here but they are more complex.
 *
 * This driver is written to be used with the Hecuba display controller
 * board, and tested with the EInk 800x600 display in 1 bit mode.
 * The interface between Hecuba and the host is TTL based GPIO. The
 * GPIO requirements are 8 writable data lines and 6 lines for control.
 * Only 4 of the controls are actually used here but 6 for future use.
 * The driver requires the IO addresses for data and control GPIO at
 * load time. It is also possible to use this display with a standard
 * PC parallel port.
 *
 * General notes:
 * - User must set hecubafb_enable=1 to enable it
 * - User must set dio_addr=0xIOADDR cio_addr=0xIOADDR c2io_addr=0xIOADDR
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/uaccess.h>

#include <asm/io.h>
/*#include <asm/hardware.h>
#include <asm/arch/at91_pio.h>
#include <asm/arch/gpio.h>
*/
#include <mach/hardware.h>
#include <mach/at91_pio.h>
#include <mach/gpio.h>

#define WF43_RS AT91_PIN_PB0
#define WF43_WR AT91_PIN_PB23
#define WF43_RD AT91_PIN_PB22
#define WF43_ON AT91_PIN_PA27
#define WF43_CS AT91_PIN_PB21
#define WF43_RESET AT91_PIN_PB1

#define WF43_D0 AT91_PIN_PB24
#define WF43_D1 AT91_PIN_PB25
#define WF43_D2 AT91_PIN_PB26
#define WF43_D3 AT91_PIN_PB27
#define WF43_D4 AT91_PIN_PB28
#define WF43_D5 AT91_PIN_PB29
#define WF43_D6 AT91_PIN_PB30
#define WF43_D7 AT91_PIN_PB31

/* Apollo controller specific defines */
#define APOLLO_START_NEW_IMG	0xA0
#define APOLLO_STOP_IMG_DATA	0xA1
#define APOLLO_DISPLAY_IMG	0xA2
#define APOLLO_ERASE_DISPLAY	0xA3
#define APOLLO_INIT_DISPLAY	0xA4

/* Hecuba interface specific defines */
/* WUP is inverted, CD is inverted, DS is inverted */
#define HCB_NWUP_BIT	0x01
#define HCB_NDS_BIT 	0x02
#define HCB_RW_BIT 	0x04
#define HCB_NCD_BIT 	0x08
#define HCB_ACK_BIT 	0x80

/* Display specific information */
#define DPY_W 480
#define DPY_H 272
#define BPP 32

void Send_WF43_Byte(unsigned char data);
void Write_WF43_C(unsigned char command, unsigned char args);
void Write_WF43_D(unsigned char command);
unsigned char command_args[10];
void Initial_SSD1963 (void);
void FULL_ON(unsigned long dat);
void WindowSet(unsigned int s_x,unsigned int e_x,unsigned int s_y,unsigned int e_y);
void Write_Command(unsigned char command);
void SendData(unsigned long color);
void FillWin(unsigned long dat,unsigned short x, unsigned short y, unsigned short w, unsigned short h);

static u32 pseudo_palette[16];

struct hecubafb_par {
	struct fb_info *info;
};

static struct fb_fix_screeninfo hecubafb_fix __devinitdata = {
	.id =		"hecubafb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.line_length =  DPY_W*BPP/8,
	.accel =	FB_ACCEL_NONE,
};

static struct fb_var_screeninfo hecubafb_var __devinitdata = {
	.xres		= DPY_W,
	.yres		= DPY_H,
	.xres_virtual	= DPY_W,
	.yres_virtual	= DPY_H,
	.bits_per_pixel	= BPP,
	.nonstd		= 0,
/*
	.red.offset	= 11,
	.red.length	= 5,
	.green.offset	= 5,
	.green.length	= 6,
	.blue.offset	= 0,
	.blue.length	= 5,
	.transp.offset	= 0,
	.transp.length	= 0,
*/
	.red.offset	= 16,
	.red.length	= 8,
	.green.offset	= 8,
	.green.length	= 8,
	.blue.offset	= 0,
	.blue.length	= 8,
	.transp.offset	= 0,
	.transp.length	= 0,
	
};


/*//static struct at91_gpio_bank *gpio;
static int gpio_banks;


static inline void __iomem *pin_to_controller(unsigned pin)
{
	pin -= PIN_BASE;
	pin /= 32;
	if (likely(pin < gpio_banks))
		return gpio[pin].regbase;

	return NULL;
}

static inline unsigned pin_to_mask(unsigned pin)
{
	pin -= PIN_BASE;
	return 1 << (pin % 32);
}


int at91_set_gpio_value(unsigned pin, int value)
{
//	void __iomem	*pio = pin_to_controller(pin);
//	unsigned	mask = pin_to_mask(pin);
        int mask=pin;

//	if (!pio)
//		return -EINVAL;
//	__raw_writel(mask, pio + (value ? PIO_SODR : PIO_CODR));
	__raw_writel(mask, pin + (value ? PIO_SODR : PIO_CODR));
	return 0;
}
*/


static void hecubafb_dpy_update(struct hecubafb_par *par,\
 unsigned int dx, unsigned int dy, unsigned int w, unsigned int h)
{
	unsigned int x,y;
//      unsigned short *videomemory = par->info->screen_base;
//	unsigned long r,g,b,vd;
        unsigned long *videomemory = par->info->screen_base;

	if(w && h) {
        WindowSet(dx,dx+w-1,dy,dy+h-1);
        Write_Command(0x2c);
	for(y=dy;y<(dy+h);y++)
	  for(x=dx;x<(dx+w);x++)
	  {
            SendData((videomemory[y*DPY_W + x]));
/*	    vd=videomemory[y*480 + x];
	    b=(vd<<3)&0xf8;
	    g=((vd>>3)&0xfc);
	    r=((vd>>8)&0xf8);
	    SendData(b | g<<8 | r<<16);*/
	  }
	}
}

static int hecubafb_setcoloreg(u_int regno, u_int red, u_int green, u_int blue, u_int transp, struct fb_info *info)
{
  if(regno > 256) return 1;
  

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}

  
  return 0;
}

static void hecubafb_fillrect(struct fb_info *info,
				   const struct fb_fillrect *rect)
{
	struct hecubafb_par *par = info->par;

	sys_fillrect(info, rect);

        printk(KERN_INFO "R");
	hecubafb_dpy_update(par, rect->dx, rect->dy, rect->width, rect->height);
}

static void hecubafb_copyarea(struct fb_info *info,
				   const struct fb_copyarea *area)
{
	struct hecubafb_par *par = info->par;

	sys_copyarea(info, area);

        printk(KERN_INFO "U");
//	hecubafb_dpy_update(par);
	hecubafb_dpy_update(par, area->dx, area->dy, area->width, area->height);
}

static void hecubafb_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct hecubafb_par *par = info->par;

	sys_imageblit(info, image);

//        printk(KERN_INFO "I");
//	hecubafb_dpy_update(par);
	hecubafb_dpy_update(par, image->dx, image->dy, image->width, image->height);
}

static ssize_t hecubafb_read(struct fb_info *info, char  *buf, size_t count, loff_t *ppos)
{
unsigned long p = *ppos;
unsigned int fb_mem_len;
unsigned int xres;

  xres = info->var.xres;
  fb_mem_len = (xres * info->var.yres)*BPP/8;
  
  if(p >= fb_mem_len)
    return 0;
  if(count >= fb_mem_len)
    count = fb_mem_len;
  if(count + p >= fb_mem_len)
    count = fb_mem_len - p;
    
  if(count){
    char *base_addr;
    base_addr = info->screen_base;
    count -= copy_to_user(buf, base_addr + p, count);
    
    if(!count)
      return -EFAULT;
    *ppos += count;
  };
  return count;
  
}

static ssize_t hecubafb_write(struct fb_info *info, const char *buf,
				size_t count, loff_t *ppos)
{
	unsigned long p;
	int err=-EINVAL;
	struct hecubafb_par *par;
	unsigned int xres;
	unsigned int fbmemlength;

	p = *ppos;
	par = info->par;
	xres = info->var.xres;
	fbmemlength = (xres * info->var.yres)*BPP/8;

	if (p > fbmemlength)
		return -ENOSPC;

	err = 0;
	if ((count + p) > fbmemlength) {
		count = fbmemlength - p;
		err = -ENOSPC;
	}

	if (count) {
		char *base_addr;

		base_addr = info->screen_base;
		count -= copy_from_user(base_addr + p, buf, count);
		*ppos += count;
		err = -EFAULT;
	}

	if (count)
	  {
//	    printk(KERN_INFO "p=%d, count=%d\n",p,count);
//  	    hecubafb_dpy_update(par,0,p/DPY_W,DPY_W,(p+count)/DPY_W-p/DPY_W+1);
//  	    hecubafb_dpy_update(par,0,p/(DPY_W*BPP/8),DPY_W,((p+count)/(DPY_W*BPP/8))-(p/(DPY_W*BPP/8)));
  	    hecubafb_dpy_update(par,p-((p/(DPY_W*BPP/8))*DPY_W),p/(DPY_W*BPP/8),DPY_W,((p+count)/(DPY_W*BPP/8))-(p/(DPY_W*BPP/8)));
//  	    hecubafb_dpy_update(par,0,0,480,272);
	    return count;
	  }
	return err;
}

static struct fb_ops hecubafb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= hecubafb_setcoloreg,
        .fb_read	= hecubafb_read,
	.fb_write	= hecubafb_write,
//        .fb_read	= fb_sys_read,
//	.fb_write	= fb_sys_write,
	.fb_fillrect	= hecubafb_fillrect,
	.fb_copyarea	= hecubafb_copyarea,
	.fb_imageblit	= hecubafb_imageblit,
};

void Send_WF43_Byte(unsigned char data)
{ 
__raw_writel(0xff000000, 0xfefff634);//write to CODR
__raw_writel(data<<24,   0xfefff630);//write to SODR

/*  gpio_set_value(WF43_D0,data&1);
  gpio_set_value(WF43_D1,(data>>1)&1);
  gpio_set_value(WF43_D2,(data>>2)&1);
  gpio_set_value(WF43_D3,(data>>3)&1);
  gpio_set_value(WF43_D4,(data>>4)&1);
  gpio_set_value(WF43_D5,(data>>5)&1);
  gpio_set_value(WF43_D6,(data>>6)&1);
  gpio_set_value(WF43_D7,(data>>7)&1);*/
}
/*id Write_WF43_C(unsigned char command, unsigned char args)
{ 
int i;
  if(args==0)
  {
    gpio_set_value(WF43_RD,1);
    gpio_set_value(WF43_WR,0);
    gpio_set_value(WF43_RS,0);
    Send_WF43_Byte(command);
    gpio_set_value(WF43_CS,0);
    gpio_set_value(WF43_CS,1);
    gpio_set_value(WF43_RS,1);
    gpio_set_value(WF43_WR,1);
  }
  else
  {
    gpio_set_value(WF43_RD,1);
    gpio_set_value(WF43_WR,0);
    gpio_set_value(WF43_RS,0);
    Send_WF43_Byte(command);
    gpio_set_value(WF43_CS,0);
    gpio_set_value(WF43_CS,1);
    gpio_set_value(WF43_RS,1);
    gpio_set_value(WF43_WR,1);

    for(i=0;i<args;i++)
    {
      gpio_set_value(WF43_RD,1);
      gpio_set_value(WF43_WR,0);
      gpio_set_value(WF43_RS,1);
      Send_WF43_Byte(command_args[i]);
      gpio_set_value(WF43_CS,0);
      gpio_set_value(WF43_CS,1);
      gpio_set_value(WF43_RS,1);
      gpio_set_value(WF43_WR,1);
    }
  }
}

void Write_WF43_D(unsigned char command)
{ 
    gpio_set_value(WF43_RD,1);
    gpio_set_value(WF43_WR,0);
    gpio_set_value(WF43_RS,1);
    Send_WF43_Byte(command);
    gpio_set_value(WF43_CS,0);
    gpio_set_value(WF43_CS,1);
    gpio_set_value(WF43_RS,1);
    gpio_set_value(WF43_WR,1);
}
*/
static int __devinit hecubafb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	int retval = -ENOMEM;
	int videomemorysize;
	unsigned char *videomemory;
	struct hecubafb_par *par;
//	int tmp;

	videomemorysize = (DPY_W*DPY_H)*BPP/8;

	
/*#define WF43_RS AT91_PIN_PB0
#define WF43_WR AT91_PIN_PB23
#define WF43_RD AT91_PIN_PB22
#define WF43_ON AT91_PIN_PA27
#define WF43_CS AT91_PIN_PB21
#define WF43_RESET AT91_PIN_PB1*/


	at91_set_GPIO_periph(WF43_WR,1);
	gpio_direction_output(WF43_WR,1);
	at91_set_GPIO_periph(WF43_RS,1);
	gpio_direction_output(WF43_RS,1);
	at91_set_GPIO_periph(WF43_RD,1);
	gpio_direction_output(WF43_RD,1);
	at91_set_GPIO_periph(WF43_ON,1);
	gpio_direction_output(WF43_ON,1);
	at91_set_GPIO_periph(WF43_CS,1);
	gpio_direction_output(WF43_CS,1);
	at91_set_GPIO_periph(WF43_RESET,1);
	gpio_direction_output(WF43_RESET,1);

	at91_set_GPIO_periph(WF43_D0,1);
	at91_set_GPIO_periph(WF43_D1,1);
	at91_set_GPIO_periph(WF43_D2,1);
	at91_set_GPIO_periph(WF43_D3,1);
	at91_set_GPIO_periph(WF43_D4,1);
	at91_set_GPIO_periph(WF43_D5,1);
	at91_set_GPIO_periph(WF43_D6,1);
	at91_set_GPIO_periph(WF43_D7,1);
	gpio_direction_output(WF43_D0,1);
	gpio_direction_output(WF43_D1,1);
	gpio_direction_output(WF43_D2,1);
	gpio_direction_output(WF43_D3,1);
	gpio_direction_output(WF43_D4,1);
	gpio_direction_output(WF43_D5,1);
	gpio_direction_output(WF43_D6,1);
	gpio_direction_output(WF43_D7,1);
	
	
	gpio_set_value(WF43_RD,1);
	
	gpio_set_value(WF43_RESET,0);
	udelay(100);
	gpio_set_value(WF43_RESET,1);
	udelay(100);
	

        Initial_SSD1963();
	

	if (!(videomemory = vmalloc(videomemorysize)))
		return retval;

//	memset(videomemory, 0, videomemorysize);

 
	info = framebuffer_alloc(sizeof(struct hecubafb_par), &dev->dev);
	if (!info)
		goto err;

	info->screen_base = (char __iomem *) videomemory;
	info->fbops = &hecubafb_ops;

	info->var = hecubafb_var;
	
	info->fix = hecubafb_fix;

	info->fix.smem_len = videomemorysize;
	par = info->par;
	par->info = info;
	info->pseudo_palette = pseudo_palette;
	hecubafb_dpy_update(par,0,0,480,272);

	info->flags = FBINFO_FLAG_DEFAULT;


	retval = register_framebuffer(info);

	if (retval < 0)
		goto err1;
	platform_set_drvdata(dev, info);

	printk(KERN_INFO
	       "fb%d: Hecuba frame buffer device, using %dK of video memory\n",
	       info->node, videomemorysize >> 10);


	return 0;
err1:
	framebuffer_release(info);
err:
	vfree(videomemory);
	return retval;
}


void Write_Command(unsigned char command)
{

  gpio_set_value(WF43_WR,0);
  gpio_set_value(WF43_RS,0);
  Send_WF43_Byte(command);
  gpio_set_value(WF43_CS,0);
  gpio_set_value(WF43_CS,1);
  gpio_set_value(WF43_WR,1);

}

//;******************************************************************************
void Write_Data(unsigned char data1)
{
  gpio_set_value(WF43_RD,1);
  gpio_set_value(WF43_WR,0);
  gpio_set_value(WF43_RS,1);
  Send_WF43_Byte(data1);
  gpio_set_value(WF43_CS,0);
  gpio_set_value(WF43_CS,1);
  gpio_set_value(WF43_WR,1);
}
//==============================================================
void Command_Write(unsigned char command,unsigned char data1)
{
Write_Command(command);
Write_Data(data1);
}
//==============================================================
void SendData(unsigned long color)
{
Write_Data((color)>>16);  // color is red
Write_Data((color)>>8);  	// color is green
Write_Data(color);  		// color is blue
}
//======================================================
// initial
//======================================================
void Initial_SSD1963 (void)
{
Write_Command(0x01);     //Software Reset
Write_Command(0x01);
Write_Command(0x01);
Command_Write(0xe0,0x01);    //START PLL
Command_Write(0xe0,0x03);    //LOCK PLL

Write_Command(0xb0);		//SET LCD MODE  SET TFT 18Bits MODE
Write_Data(0x08);			//SET TFT MODE & hsync+Vsync+DEN MODE
Write_Data(0x80);			//SET TFT MODE & hsync+Vsync+DEN MODE
Write_Data(0x01);			//SET horizontal size=480-1 HightByte
Write_Data(0xdf);		    //SET horizontal size=480-1 LowByte
Write_Data(0x01);			//SET vertical size=272-1 HightByte
Write_Data(0x0f);			//SET vertical size=272-1 LowByte
Write_Data(0x00);			//SET even/odd line RGB seq.=RGB

Command_Write(0xf0,0x00);	//SET pixel data I/F format=8bit
Command_Write(0x3a,0x60);   // SET R G B format = 6 6 6
//Command_Write(0x3a,0x70);   // SET R G B format = 8 8 8

//Write_Command(0xe6);   	//scan
//Write_Data(0xff);			//

Write_Command(0xe6);   		//SET PCLK freq=9MHz  ; pixel clock frequency
Write_Data(0x01);
Write_Data(0x45);
Write_Data(0x47);

Write_Command(0xb4);		//SET HBP, 
Write_Data(0x02);			//SET HSYNC Tatol 525
Write_Data(0x0d);
Write_Data(0x00);			//SET HBP 43
Write_Data(0x2b);
Write_Data(0x28);			//SET VBP 41=40+1
Write_Data(0x00);			//SET Hsync pulse start position
Write_Data(0x00);
Write_Data(0x00);			//SET Hsync pulse subpixel start position
	
Write_Command(0xb6); 		//SET VBP, 
Write_Data(0x01);			//SET Vsync total 286=285+1
Write_Data(0x1d);
Write_Data(0x00);			//SET VBP=12
Write_Data(0x0c);
Write_Data(0x09);			//SET Vsync pulse 10=9+1
Write_Data(0x00);			//SET Vsync pulse start position
Write_Data(0x00);

Write_Command(0x2a);		//SET column address
Write_Data(0x00);			//SET start column address=0
Write_Data(0x00);
Write_Data(0x01);			//SET end column address=479
Write_Data(0xdf);

Write_Command(0x2b);		//SET page address
Write_Data(0x00);			//SET start page address=0
Write_Data(0x00);
Write_Data(0x01);			//SET end page address=271
Write_Data(0x0f);

Write_Command(0x29);		//SET display on

}
//======================================================
void WindowSet(unsigned int s_x,unsigned int e_x,unsigned int s_y,unsigned int e_y)
{
Write_Command(0x2a);		//SET page address
Write_Data((s_x)>>8);			//SET start page address=0
Write_Data(s_x);
Write_Data((e_x)>>8);			//SET end page address=319
Write_Data(e_x);

Write_Command(0x2b);		//SET column address
Write_Data((s_y)>>8);			//SET start column address=0
Write_Data(s_y);
Write_Data((e_y)>>8);			//SET end column address=239
Write_Data(e_y);
}
/*
//=======================================
void FULL_ON(unsigned long dat)
{
unsigned int x,y,z;
WindowSet(0x0000,0x01Df,0x0000,0x010F);
Write_Command(0x2c);
for(x=0;x<272;x++)
	{
        for(y= 0;y<40;y++)
                {
      		  for(z= 0;z<12;z++)
		  {
		    SendData(dat);
		  }
                }
	}
}
void FillWin(unsigned long dat,unsigned short x, unsigned short y, unsigned short w, unsigned short h)
{
//unsigned int x,y,z;
WindowSet(x,x+w,y,y+h);
Write_Command(0x2c);
for(x=0;x<w;x++)
	{
        for(y=0;y<h;y++)
                {
      	//	  for(z= 0;z<12;z++)
		  {
		    SendData(dat);
		  }
                }
	}
}
//=======================================
FRAME()
{
unsigned int i,j;
WindowSet(0x0000,0x01Df,0x0000,0x010F);
Write_Command(0x2c);
for(j= 0 ;j<40;j++)
  {
   for(i=0;i<12;i++)
   {
    SendData(0xffffff);             //white
   }
   }

 for(j=0;j<270;j++)
 {
 SendData(0xff0000);             //red
   for(i=0;i<239;i++)
   {
      SendData(0x000000);             //black
      SendData(0x000000);             //black
   }
 SendData(0x0000ff);             //blue
  }

 for(j= 0 ;j<40;j++)
  {
   for(i=0;i<12;i++)
   {
    SendData(0xffffff);             //white
   }
  }
}*/
//=======================================
/*id main()
{
IC_UD = 0;
IC_LR = 1;
Initial_SSD1963();

while(1)
        {
		//FRAME();
		STP_SC();
        FULL_ON(0xff0000);   // red
		STP_SC();
        FULL_ON(0x00ff00);   // green
		STP_SC();
        FULL_ON(0x0000ff);	 // blue
		STP_SC();
        //FULL_ON(0xff00ff);
		STP_SC();
        //FULL_ON(0x00ffff);
		STP_SC();
      
        }
}*/




static int __devexit hecubafb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
//		fb_deferred_io_cleanup(info);
		unregister_framebuffer(info);
		vfree((void __force *)info->screen_base);
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver hecubafb_driver = {
	.probe	= hecubafb_probe,
	.remove = hecubafb_remove,
	.driver	= {
		.name	= "hecubafb",
	},
};

static struct platform_device *hecubafb_device;

static int __init hecubafb_init(void)
{
	int ret;

/*f (!hecubafb_enable) {
		printk(KERN_ERR "Use hecubafb_enable to enable the device\n");
		return -ENXIO;
	}*/

	ret = platform_driver_register(&hecubafb_driver);
	if (!ret) {
		hecubafb_device = platform_device_alloc("hecubafb", 0);
		if (hecubafb_device)
			ret = platform_device_add(hecubafb_device);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(hecubafb_device);
			platform_driver_unregister(&hecubafb_driver);
		}
	}
	return ret;

}

static void __exit hecubafb_exit(void)
{
	platform_device_unregister(hecubafb_device);
	platform_driver_unregister(&hecubafb_driver);
}

/*dule_param(nosplash, uint, 0);
MODULE_PARM_DESC(nosplash, "Disable doing the splash screen");
module_param(hecubafb_enable, uint, 0);
MODULE_PARM_DESC(hecubafb_enable, "Enable communication with Hecuba board");
module_param(dio_addr, ulong, 0);
MODULE_PARM_DESC(dio_addr, "IO address for data, eg: 0x480");
module_param(cio_addr, ulong, 0);
MODULE_PARM_DESC(cio_addr, "IO address for control, eg: 0x400");
module_param(c2io_addr, ulong, 0);
MODULE_PARM_DESC(c2io_addr, "IO address for secondary control, eg: 0x408");
module_param(splashval, ulong, 0);
MODULE_PARM_DESC(splashval, "Splash pattern: 0x00 is black, 0x01 is white");
module_param(irq, uint, 0);
MODULE_PARM_DESC(irq, "IRQ for the Hecuba board");
*/

module_init(hecubafb_init);
module_exit(hecubafb_exit);

MODULE_DESCRIPTION("fbdev driver for Hecuba board");
MODULE_AUTHOR("Jaya Kumar");
MODULE_LICENSE("GPL");
