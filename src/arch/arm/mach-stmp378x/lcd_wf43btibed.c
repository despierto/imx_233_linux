/* Based on SSD1289 driver */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <mach/regs-lcdif.h>
#include <mach/regs-lradc.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-apbh.h>
#include <mach/gpio.h>
#include <mach/pins.h>
#include <mach/pinmux.h>
#include <mach/lcdif.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/cputype.h>
#include <linux/ctype.h>
#include "mpulcd_common.h"
#include "lcd_wf43btibed.h"
#include <linux/vmalloc.h>

#define LCD_USE_16BITS_BUS  0

//#define WF43_32BPP

#define LCDIF_DATA_SETUP_XCLKS 1
#define LCDIF_DATA_HOLD_XCLKS  1
#define LCDIF_CMD_SETUP_XCLKS  3
#define LCDIF_CMD_HOLD_XCLKS   3

SSD1289EntryMode ddi_controller_ssd1289_entry_mode;
SSD1289DisplayControl ddi_controller_ssd1289_DisplayControl;
SSD1289LCDDrivingWaveformControl ddi_controller_ssd1289_LCDDrivingWaveformControl;
SSD1289FrameCycleControl ddi_controller_ssd1289_FrameCycleControl;
SSD1289DriverOutputControl ddi_controller_ssd1289_DriverOutputControl;

void WindowSet(unsigned short sx, unsigned short ex, unsigned short sy, unsigned short ey);

void mpulcd_init_lcdif(void)
{
    volatile hw_lcdif_ctrl_t reg;
    volatile hw_lcdif_ctrl1_t reg1;
    volatile hw_lcdif_Timing_t timing;

    mpulcd_lcdif_reset();

    // initialize lcdif ctrl
    reg.U = HW_LCDIF_CTRL_RD();

    reg.B.DATA_FORMAT_24_BIT    = 0;
    reg.B.DATA_FORMAT_18_BIT    = 0;
    reg.B.DATA_FORMAT_16_BIT    = 0;
    reg.B.LCDIF_MASTER          = 1;
    reg.B.ENABLE_PXP_HANDSHAKE  = 0;
#if LCD_USE_16BITS_BUS
    reg.B.WORD_LENGTH       	= WORDLENGTH_16BITS;
    reg.B.LCD_DATABUS_WIDTH     = HW_LCDIF_BUS_WIDTH_16BIT;
    reg.B.CSC_DATA_SWIZZLE      = NO_SWAP;
    reg.B.INPUT_DATA_SWIZZLE    = NO_SWAP;
#else
    reg.B.WORD_LENGTH       	= WORDLENGTH_8BITS;
    reg.B.LCD_DATABUS_WIDTH     = HW_LCDIF_BUS_WIDTH_8BIT;
    reg.B.CSC_DATA_SWIZZLE      = 0;
    reg.B.INPUT_DATA_SWIZZLE    = 0;
    
#endif
    reg.B.DATA_SELECT           = DATA_MODE;
    reg.B.DOTCLK_MODE           = 0;
    reg.B.VSYNC_MODE            = 0;
    reg.B.BYPASS_COUNT          = 0;
    reg.B.DVI_MODE              = 0;
    reg.B.SHIFT_NUM_BITS        = 0;
    reg.B.DATA_SHIFT_DIR        = 0;
    reg.B.WAIT_FOR_VSYNC_EDGE   = 0;

    HW_LCDIF_CTRL_WR(reg.U);

    // initialize lcdif ctrl-1
    reg1.U = HW_LCDIF_CTRL1_RD();

    reg1.B.RESET               = LCDRESET_HIGH;
    reg1.B.MODE86              = BUSMODE_8080;
    reg1.B.CUR_FRAME_DONE_IRQ_EN = 0;
    reg1.B.BUSY_ENABLE         = 0;
#ifdef WF43_32BPP
    reg1.B.BYTE_PACKING_FORMAT = 0x07;
#else
    reg1.B.BYTE_PACKING_FORMAT = 0x0f;
#endif
    HW_LCDIF_CTRL1_WR(reg1.U);

    // initialize lcdif timing
    timing.m_u8DataSetup = LCDIF_DATA_SETUP_XCLKS;
    timing.m_u8DataHold  = LCDIF_DATA_HOLD_XCLKS;
    timing.m_u8CmdSetup  = LCDIF_CMD_SETUP_XCLKS;
    timing.m_u8CmdHold   = LCDIF_CMD_HOLD_XCLKS;

    HW_LCDIF_TIMING_WR(timing.U);
}

void WindowSet(unsigned short sx, unsigned short ex, unsigned short sy, unsigned short ey)
{
    mpulcd_setup_pannel_register(CMD_MODE, 0x2a);
    mpulcd_setup_pannel_register(DATA_MODE, sx>>8);//sx
    mpulcd_setup_pannel_register(DATA_MODE, sx);//sx
    mpulcd_setup_pannel_register(DATA_MODE, ex>>8 );//ex
    mpulcd_setup_pannel_register(DATA_MODE, ex );//ex

    mpulcd_setup_pannel_register(CMD_MODE, 0x2b);
    mpulcd_setup_pannel_register(DATA_MODE, sy>>8);//sy
    mpulcd_setup_pannel_register(DATA_MODE, sy);//sy
    mpulcd_setup_pannel_register(DATA_MODE, ey>>8 );//ey
    mpulcd_setup_pannel_register(DATA_MODE, ey );//ey
}

void mpulcd_init_panel_hw(void)
{
    gpio_request(PINID_LCD_ENABLE,"wf43");
    gpio_direction_output(PINID_LCD_ENABLE,1);
    
    ddi_controller_ssd1289_entry_mode.V = 0x6030;
    ddi_controller_ssd1289_DisplayControl.V = 0x0231;

    ddi_controller_ssd1289_LCDDrivingWaveformControl.V = 0x0600;
    ddi_controller_ssd1289_FrameCycleControl.V = 0;

    // RL=1, REV=1, BGR=1, TB=0 (orig)
    ddi_controller_ssd1289_DriverOutputControl.V = 0x693F;

    ddi_controller_ssd1289_entry_mode.B.AM = 1;
    ddi_controller_ssd1289_entry_mode.B.ID = 1;

    ddi_controller_ssd1289_DisplayControl.B.D1 = 1;

    mpulcd_setup_pannel_register(CMD_MODE, 0x01);
    mpulcd_setup_pannel_register(CMD_MODE, 0x01);
    mpulcd_setup_pannel_register(CMD_MODE, 0x01);
    mdelay(10);

    mpulcd_setup_pannel_register(CMD_MODE, 0xe0);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mdelay(10);
    mpulcd_setup_pannel_register(CMD_MODE, 0xe0);
    mpulcd_setup_pannel_register(DATA_MODE, 0x03);

    mpulcd_setup_pannel_register(CMD_MODE, 0x36);
    mpulcd_setup_pannel_register(DATA_MODE, 0x08);

    mpulcd_setup_pannel_register(CMD_MODE, 0xb0);
    mpulcd_setup_pannel_register(DATA_MODE, 0x0c);
    mpulcd_setup_pannel_register(DATA_MODE, 0x80);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mpulcd_setup_pannel_register(DATA_MODE, 0xdf);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mpulcd_setup_pannel_register(DATA_MODE, 0x0f);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);

    mpulcd_setup_pannel_register(CMD_MODE, 0xf0);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(CMD_MODE, 0x3a);
    mpulcd_setup_pannel_register(DATA_MODE, 0x60);

    mpulcd_setup_pannel_register(CMD_MODE, 0xe6);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mpulcd_setup_pannel_register(DATA_MODE, 0x45);
    mpulcd_setup_pannel_register(DATA_MODE, 0x47);

    mpulcd_setup_pannel_register(CMD_MODE, 0xb4);
    mpulcd_setup_pannel_register(DATA_MODE, 0x02);
    mpulcd_setup_pannel_register(DATA_MODE, 0x0d);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x2b);
    mpulcd_setup_pannel_register(DATA_MODE, 0x28);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);


    mpulcd_setup_pannel_register(CMD_MODE, 0xb6);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mpulcd_setup_pannel_register(DATA_MODE, 0x1d);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x0c);
    mpulcd_setup_pannel_register(DATA_MODE, 0x09);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);

    mpulcd_setup_pannel_register(CMD_MODE, 0x2a);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mpulcd_setup_pannel_register(DATA_MODE, 0xdf);

    mpulcd_setup_pannel_register(CMD_MODE, 0x2b);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x00);
    mpulcd_setup_pannel_register(DATA_MODE, 0x01);
    mpulcd_setup_pannel_register(DATA_MODE, 0x0f);

    mpulcd_setup_pannel_register(CMD_MODE, 0x29);
    mpulcd_setup_pannel_register(CMD_MODE, 0x2c);
    
//    WindowSet(0,479,0,271);
    mdelay(10);

}

void mpulcd_display_on( void )
{
volatile hw_lcdif_ctrl1_t reg1;

    mpulcd_setup_pannel_register(CMD_MODE, 0x29);

    mpulcd_setup_pannel_register(CMD_MODE, 0x2c);

    HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_DATA_SELECT);
    HW_LCDIF_TRANSFER_COUNT_SET(BF_LCDIF_TRANSFER_COUNT_V_COUNT(272) |
                                BF_LCDIF_TRANSFER_COUNT_H_COUNT(480*3));

    reg1.U = HW_LCDIF_CTRL1_RD();
    reg1.B.CUR_FRAME_DONE_IRQ_EN = 1;
    HW_LCDIF_CTRL1_WR(reg1.U);

    stmp3xxx_lcdif_run();

}

void mpulcd_display_off( void )
{
volatile hw_lcdif_ctrl1_t reg1;

    HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_RUN);
    reg1.U = HW_LCDIF_CTRL1_RD();
    reg1.B.CUR_FRAME_DONE_IRQ_EN = 0;
    HW_LCDIF_CTRL1_WR(reg1.U);
    mpulcd_setup_pannel_register(CMD_MODE, 0x28);
}

int mpulcd_blank_panel(int blank)
{
    int ret = 0;

    switch (blank) 
    {
        case FB_BLANK_NORMAL:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_UNBLANK:
            break;

        default:
            ret = -EINVAL;
    }

    return ret;
}

static struct stmp3xxx_platform_fb_entry fb_entry = {
    .name       	= "wf43",
    .x_res      	= 272,
    .y_res      	= 480,
#ifdef WF43_32BPP
    .bpp        	= 32,
#else
    .bpp        	= 24,
#endif
    .cycle_time_ns  	= 150,
    .lcd_type   	= STMP3XXX_LCD_PANEL_SYSTEM,
    .init_panel 	= mpulcd_init_panel,
    .release_panel  	= mpulcd_release_panel,
    .blank_panel    	= mpulcd_blank_panel,
    .run_panel  	= mpulcd_display_on,
    .stop_panel 	= mpulcd_display_off,
    .pan_display    	= stmp3xxx_lcdif_pan_display,
};

static int __init register_devices(void)
{
    stmp3xxx_lcd_register_entry(&fb_entry, stmp3xxx_framebuffer.dev.platform_data);
    return 0;
}

subsys_initcall(register_devices);

