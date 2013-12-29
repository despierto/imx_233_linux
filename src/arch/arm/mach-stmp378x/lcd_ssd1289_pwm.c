
static struct clk *pwm_clk;
static int init_bl(struct stmp3xxx_platform_bl_data *data)
{
    int ret = 0;

    pwm_clk = clk_get(NULL, "pwm");
    if (IS_ERR(pwm_clk))
    {
        ret = PTR_ERR(pwm_clk);
        goto out;
    }

    clk_enable(pwm_clk);
    stmp3xxx_reset_block(REGS_PWM_BASE, 1);

    ret = stmp3xxx_request_pin(PINID_PWM2, PIN_FUN1, "lcd_ssd1289");
    if (ret)
        goto out_mux;

    stmp3xxx_pin_voltage(PINID_PWM2, PIN_16MA, "lcd_ssd1289");
    stmp3xxx_pin_strength(PINID_PWM2, PIN_3_3V, "lcd_ssd1289");

    __raw_writel(BF(0, PWM_ACTIVEn_INACTIVE) |
                BF(0, PWM_ACTIVEn_ACTIVE),
                REGS_PWM_BASE + HW_PWM_ACTIVEn(2));

    __raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
            BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
            BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
            BF(80, PWM_PERIODn_PERIOD), REGS_PWM_BASE + HW_PWM_PERIODn(2));
    stmp3xxx_setl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
    printk("\r\n --init_bl");

#if 0
    __raw_writel(BF(80, PWM_ACTIVEn_INACTIVE) |
        BF(0, PWM_ACTIVEn_ACTIVE), REGS_PWM_BASE + HW_PWM_ACTIVEn(2));

    __raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
        BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
        BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
        BF(80, PWM_PERIODn_PERIOD),
        REGS_PWM_BASE + HW_PWM_PERIODn(2));
#endif
    return 0;

out_mux:
    clk_put(pwm_clk);
out:
    return ret;
}

static void free_bl(struct stmp3xxx_platform_bl_data *data)
{
    __raw_writel(BF(0, PWM_ACTIVEn_INACTIVE) |
                BF(0, PWM_ACTIVEn_ACTIVE),
                REGS_PWM_BASE + HW_PWM_ACTIVEn(2));

    __raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
        BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
        BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
        BF(80, PWM_PERIODn_PERIOD),
        REGS_PWM_BASE + HW_PWM_PERIODn(2));

    stmp3xxx_clearl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
    stmp3xxx_pin_voltage(PINID_PWM2, PIN_4MA, "lcd_ssd1289");
    stmp3xxx_pin_strength(PINID_PWM2, PIN_1_8V, "lcd_ssd1289");

    stmp3xxx_release_pin(PINID_PWM2, "lcd_ssd1289");
    clk_disable(pwm_clk);
    clk_put(pwm_clk);
}

static int values[] = { 6, 9, 12, 15, 19, 24, 30, 40, 55, 75, 100 };
static int power[] =
{
    0, 1500, 3600, 6100, 10300,
    15500, 74200, 114200, 155200,
    190100, 191000
};

static int  bl_to_power(int br)
{
    int base;
    int rem;

    if (br > 100)
        br = 100;

    base = power[br/10];
    rem = br % 10;

    if (!rem)
        return base;
    else
        return base + (rem * (power[br/10 + 1]) - base) / 10;
}

static int set_bl_intensity(struct stmp3xxx_platform_bl_data *data,
            struct backlight_device *bd, int suspended)
{
    int intensity = bd->props.brightness;
    int scaled_int;

    if (bd->props.power != FB_BLANK_UNBLANK)
        intensity = 0;
    if (bd->props.fb_blank != FB_BLANK_UNBLANK)
        intensity = 0;
    if (suspended)
        intensity = 0;

    /*
     * This is not too cool but what can we do?
     * Luminance changes non-linearly...
     */
    if (regulator_set_current_limit(data->regulator, bl_to_power(intensity), bl_to_power(intensity)))
        return -EBUSY;

    scaled_int = values[intensity/10];
    if (scaled_int < 100)
    {
        int rem = intensity - 10 * (intensity/10); /* r = i % 10;*/
        scaled_int += rem*(values[intensity/10 + 1] -
                  values[intensity/10])/10;
    }
    __raw_writel(BF(scaled_int, PWM_ACTIVEn_INACTIVE) |
        BF(0, PWM_ACTIVEn_ACTIVE), REGS_PWM_BASE + HW_PWM_ACTIVEn(2));

    __raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
        BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
        BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
        BF(80, PWM_PERIODn_PERIOD),
        REGS_PWM_BASE + HW_PWM_PERIODn(2));

    return 0;
}

static struct stmp3xxx_platform_bl_data bl_data = {
    .bl_max_intensity   = (BM_LRADC_CTRL2_BL_BRIGHTNESS >>
                    BP_LRADC_CTRL2_BL_BRIGHTNESS) + 1,
    .bl_default_intensity   = 0x1f,
    .init_bl        = init_bl,
    .free_bl        = free_bl,
    .set_bl_intensity   = set_bl_intensity,
};

