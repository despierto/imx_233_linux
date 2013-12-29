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

#define LCD_USE_16BITS_BUS  0

static struct pin_group *lcd_pins;

struct pin_desc mpulcd_pin_desc[] =
{
    { PINID_LCD_D00, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D01, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D02, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D03, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D04, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D05, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D06, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D07, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
#if LCD_USE_16BITS_BUS
    { PINID_LCD_D08, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D09, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D10, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D11, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D12, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D13, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D14, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_D15, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
#endif
    { PINID_LCD_RESET, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_RS, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_CS, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
    { PINID_LCD_WR, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 }
};

struct pin_group stmp378x_mpu_lcd_pins =
{
    .pins       = mpulcd_pin_desc,
    .nr_pins    = ARRAY_SIZE(mpulcd_pin_desc),
};

int init_pinmux(void)
{
    int ret = -EINVAL;

    lcd_pins = &stmp378x_mpu_lcd_pins;
    ret = stmp3xxx_request_pin_group(lcd_pins, "mpulcd_pin");

    return ret;
}

void uninit_pinmux(void)
{
    //lcd_pins = &stmp378x_mpu_lcd_pins;
    stmp3xxx_release_pin_group(lcd_pins, "mpulcd_pin");
}

void mpulcd_lcdif_SetDataSwizzle(hw_lcdif_DataSwizzle_t eDataSwizzle)
{
    hw_lcdif_ctrl_t reg;
    reg.U = HW_LCDIF_CTRL_RD();
    reg.B.INPUT_DATA_SWIZZLE = eDataSwizzle;
    HW_LCDIF_CTRL_WR(reg.U);
}

void mpulcd_lcdif_SetDataShift(hw_lcdif_DataShiftDir_t dir, uint8_t num_bits)
{
    hw_lcdif_ctrl_t reg;
    reg.U = HW_LCDIF_CTRL_RD();
    reg.B.DATA_SHIFT_DIR = dir;
    reg.B.SHIFT_NUM_BITS = num_bits & 0x03;
    HW_LCDIF_CTRL_WR(reg.U);
}

void mpulcd_lcdif_SetWordLength(hw_lcdif_WordLength_t len )
{
    hw_lcdif_ctrl_t reg;
    reg.U = HW_LCDIF_CTRL_RD();
    reg.B.WORD_LENGTH = len;
    HW_LCDIF_CTRL_WR(reg.U);
}

void mpulcd_lcdif_SetBytePacking(uint8_t valid_bytes)
{
    hw_lcdif_ctrl1_t reg;
    reg.U = HW_LCDIF_CTRL1_RD();
    reg.B.BYTE_PACKING_FORMAT = valid_bytes & 0x0F;
    HW_LCDIF_CTRL1_WR(reg.U);
}

void mpulcd_write_buffer(int mode, void *buf, int count)
{
//    const unsigned int *u32buf = (const unsigned int *)buf;
    unsigned int *u32buf = (unsigned int *)buf;
    const unsigned int max_size = 0xffff * 2;
    unsigned int val;
    hw_lcdif_ctrl_t reg,treg;


    if(count == 0) return;

    reg.U = HW_LCDIF_CTRL_RD();
    treg=reg;
    reg.B.WORD_LENGTH = WORDLENGTH_24BITS;
    HW_LCDIF_CTRL_WR(reg.U);
    mpulcd_lcdif_SetBytePacking(0x07);

    val = HW_LCDIF_CTRL1_RD();
    val &= ~(0xf << 12);
    HW_LCDIF_CTRL1_WR(val);

    HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_LCDIF_MASTER);

    do {
        unsigned int  size = MIN(count, max_size);
        count -= size;

        if(size & 1)
            size++;

//        HW_LCDIF_TRANSFER_COUNT_SET(BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
//                                    BF_LCDIF_TRANSFER_COUNT_H_COUNT(size >> 1));
        HW_LCDIF_TRANSFER_COUNT_SET(BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) |
                                    BF_LCDIF_TRANSFER_COUNT_H_COUNT(size));
        if(mode == 0)
            HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_DATA_SELECT);
        else
            HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_DATA_SELECT);

        HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_RUN);

//        size = ((size + 3) & (~3)) >> 2;
        while(size--) {
            while(HW_LCDIF_STAT_RD() & BM_LCDIF_STAT_LFIFO_FULL);

            HW_LCDIF_DATA_WR(*u32buf);
            u32buf++;
        }

        while(HW_LCDIF_CTRL_RD() & BM_LCDIF_CTRL_RUN);
    } while(count);

    HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_LCDIF_MASTER);

    HW_LCDIF_CTRL_WR(treg.U);

    val = HW_LCDIF_CTRL1_RD();
    val |= (0xf << 12);
    HW_LCDIF_CTRL1_WR(val);
}





#if LCD_USE_16BITS_BUS
/* This function is used to write LCD config registers */
void mpulcd_setup_pannel_register(hw_lcdif_CommMode_t mode, uint16_t data)
{
    mpulcd_lcdif_SetDataSwizzle(NO_SWAP);
    mpulcd_lcdif_SetBytePacking(0x03);  // 16bits valid data
    HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_LCDIF_MASTER);

    HW_LCDIF_TRANSFER_COUNT_SET(BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) | BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

    if(mode == CMD_MODE)
    {
        HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_DATA_SELECT); // Command
    }
    else
    {
        HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_DATA_SELECT); // Data
    }

    HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_RUN);

    HW_LCDIF_DATA_WR(data);

    while(HW_LCDIF_CTRL_RD() & BM_LCDIF_CTRL_RUN);

//    mpulcd_lcdif_SetBytePacking(0x0f); // 32 bits valid data
    HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_LCDIF_MASTER);
}
#else
/* This function is used to write LCD config registers */
void mpulcd_setup_pannel_register(hw_lcdif_CommMode_t mode, uint16_t data)
{
int tmp,tmp1,tmp2;
//    mpulcd_lcdif_SetDataSwizzle(NO_SWAP);
//    mpulcd_lcdif_SetBytePacking(0x03);  // 16bits valid data
//    mpulcd_lcdif_SetBytePacking(0x01);  // 16bits valid data

    tmp=__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
    tmp1=__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CTRL);
    tmp2=__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

    HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_RUN);

    HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_LCDIF_MASTER);
    HW_LCDIF_TRANSFER_COUNT_SET(BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) | BF_LCDIF_TRANSFER_COUNT_H_COUNT(1));

    if(mode == CMD_MODE)
    {
        HW_LCDIF_CTRL_CLR_WR(BM_LCDIF_CTRL_DATA_SELECT); // Command
    }
    else
    {
        HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_DATA_SELECT); // Data
    }

    HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_RUN);

    HW_LCDIF_DATA_WR(data);

    while(HW_LCDIF_CTRL_RD() & BM_LCDIF_CTRL_RUN);

//    HW_LCDIF_CTRL_SET_WR(BM_LCDIF_CTRL_LCDIF_MASTER);

    __raw_writel(tmp,REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
    __raw_writel(tmp1,REGS_LCDIF_BASE + HW_LCDIF_CTRL);
    __raw_writel(tmp2,REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

//    mpulcd_lcdif_SetDataSwizzle(HWD_BYTE_SWAP);
//    mpulcd_lcdif_SetBytePacking(0x0f);  // 16bits valid data
}
#endif

void mpulcd_lcdif_reset(void)
{
    volatile hw_lcdif_ctrl_t reg;

    for(;;)
    {
        reg.U = HW_LCDIF_CTRL_RD();
        if( reg.B.CLKGATE == 0 )
        {
            break;
        }
        reg.B.CLKGATE = 0;
        HW_LCDIF_CTRL_WR(reg.U);
    }

    for(;;)
    {
        reg.U = HW_LCDIF_CTRL_RD();
        if( reg.B.SFTRST == 1 )
        {
            break;
        }
        reg.B.SFTRST = 1;
        HW_LCDIF_CTRL_WR(reg.U);
    }

    for(;;)
    {
        reg.U = HW_LCDIF_CTRL_RD();
        if( reg.B.CLKGATE == 0 )
        {
            break;
        }
        reg.B.CLKGATE = 0;
        HW_LCDIF_CTRL_WR(reg.U);
    }

    for(;;)
    {
        reg.U = HW_LCDIF_CTRL_RD();
        if( reg.B.SFTRST == 0 )
        {
            break;
        }
        reg.B.SFTRST = 0;
        HW_LCDIF_CTRL_WR(reg.U);
    }

    for(;;)
    {
        reg.U = HW_LCDIF_CTRL_RD();
        if( reg.B.CLKGATE == 0 )
        {
            break;
        }
        reg.B.CLKGATE = 0;
        HW_LCDIF_CTRL_WR(reg.U);
    }

    reg.U = HW_LCDIF_CTRL_RD();
    reg.U = HW_LCDIF_CTRL_RD();
    reg.U = HW_LCDIF_CTRL_RD();
    reg.U = HW_LCDIF_CTRL_RD();

    for(;;)
    {
        reg.U = HW_LCDIF_CTRL_RD();
        if( reg.B.CLKGATE == 0 )
        {
            break;
        }
        reg.B.CLKGATE = 0;
        HW_LCDIF_CTRL_WR(reg.U);
    }
}

static struct clk *lcd_clk;

int mpulcd_init_panel(struct device *dev, dma_addr_t phys, int memsize,
        struct stmp3xxx_platform_fb_entry *pentry)
{
    int ret = 0;

    lcd_clk = clk_get(dev, "lcdif");
    if (IS_ERR(lcd_clk))
    {
        ret = PTR_ERR(lcd_clk);
        goto out_1;
    }

    ret = clk_enable(lcd_clk);
    if (ret)
    {
        clk_put(lcd_clk);
        goto out_1;
    }

//    ret = clk_set_rate(lcd_clk, 24000);//60 fps
    ret = clk_set_rate(lcd_clk, 10000);//25 fps
//    ret = clk_set_rate(lcd_clk, 5000);//12 fps
    if (ret)
    {
        clk_disable(lcd_clk);
        clk_put(lcd_clk);
        goto out_1;
    }

    mpulcd_init_lcdif();
    init_pinmux();
    stmp3xxx_lcdif_dma_init(dev, phys, memsize, 1);
    mpulcd_init_panel_hw();
    stmp3xxx_lcd_set_bl_pdata(pentry->bl_data);
    stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_INIT, pentry);
    return 0;

out_1:
    uninit_pinmux();
    return ret;
}

void mpulcd_release_panel(struct device *dev,
        struct stmp3xxx_platform_fb_entry *pentry)
{
    stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_RELEASE, pentry);
    mpulcd_display_off();
    uninit_pinmux();
    stmp3xxx_lcdif_dma_release();
    clk_disable(lcd_clk);
    clk_put(lcd_clk);
}

