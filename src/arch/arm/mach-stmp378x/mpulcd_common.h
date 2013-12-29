#ifndef __MPULCD_COMMON_H
#define __MPULCD_COMMON_H

#define MIN(a, b)		((a) < (b)) ? (a) : (b)

typedef union
{
    u32  U;
    struct
    {
        unsigned RUN                   :  1;
        unsigned DATA_FORMAT_24_BIT    :  1;
        unsigned DATA_FORMAT_18_BIT    :  1;
        unsigned DATA_FORMAT_16_BIT    :  1;
        unsigned DMA_BURST_LENGTH      :  1;
        unsigned LCDIF_MASTER          :  1;
        unsigned ENABLE_PXP_HANDSHAKE  :  1;
        unsigned RGB_TO_YCBCR422_CSC   :  1;
        unsigned WORD_LENGTH           :  2;
        unsigned LCD_DATABUS_WIDTH     :  2;
        unsigned CSC_DATA_SWIZZLE      :  2;
        unsigned INPUT_DATA_SWIZZLE    :  2;
        unsigned DATA_SELECT           :  1;
        unsigned DOTCLK_MODE           :  1;
        unsigned VSYNC_MODE            :  1;
        unsigned BYPASS_COUNT          :  1;
        unsigned DVI_MODE              :  1;
        unsigned SHIFT_NUM_BITS        :  5;
        unsigned DATA_SHIFT_DIR        :  1;
        unsigned WAIT_FOR_VSYNC_EDGE   :  1;
        unsigned RSRVD0                :  1;
        unsigned YCBCR422_INPUT        :  1;
        unsigned CLKGATE               :  1;
        unsigned SFTRST                :  1;
    } B;
} hw_lcdif_ctrl_t;

typedef union
{
    u32  U;
    struct
    {
        unsigned RESET                              :  1;
        unsigned MODE86                             :  1;
        unsigned BUSY_ENABLE                        :  1;
        unsigned LCD_CS_CTRL                        :  1;
        unsigned PAUSE_TRANSFER_IRQ                 :  1;
        unsigned PAUSE_TRANSFER_IRQ_EN              :  1;
        unsigned PAUSE_TRANSFER                     :  1;
        unsigned RSRVD0                             :  1;
        unsigned VSYNC_EDGE_IRQ                     :  1;
        unsigned CUR_FRAME_DONE_IRQ                 :  1;
        unsigned UNDERFLOW_IRQ                      :  1;
        unsigned OVERFLOW_IRQ                       :  1;
        unsigned VSYNC_EDGE_IRQ_EN                  :  1;
        unsigned CUR_FRAME_DONE_IRQ_EN              :  1;
        unsigned UNDERFLOW_IRQ_EN                   :  1;
        unsigned OVERFLOW_IRQ_EN                    :  1;
        unsigned BYTE_PACKING_FORMAT                :  4;
        unsigned IRQ_ON_ALTERNATE_FIELDS            :  1;
        unsigned FIFO_CLEAR                         :  1;
        unsigned START_INTERLACE_FROM_SECOND_FIELD  :  1;
        unsigned INTERLACE_FIELDS                   :  1;
        unsigned RECOVER_ON_UNDERFLOW               :  1;
        unsigned BM_ERROR_IRQ                       :  1;
        unsigned BM_ERROR_IRQ_EN                    :  1;
        unsigned RSRVD1                             :  5;
    } B;
} hw_lcdif_ctrl1_t;

typedef union
{
    struct
    {
        //! Data bus setup time in XCLK cycles.
        //! Also the time that the data strobe is asserted in a cycle
        u8 m_u8DataSetup;
        //! Data bus hold time in XCLK cycles.
        //! Also the time that the data strobe is de-asserted in a cycle
        u8 m_u8DataHold;
        //! Number of XCLK cycles that DCn is active before CEn is asserted
        u8 m_u8CmdSetup;
        //! Number of XCLK cycles that DCn is active after CEn is de-asserted
        u8 m_u8CmdHold;
    };
    u32  U;
} hw_lcdif_Timing_t;

#define HW_LCDIF_CTRL_RD()     	((*(volatile hw_lcdif_ctrl_t *) HW_LCDIF_CTRL_ADDR).U)
#define HW_LCDIF_CTRL_WR(v)    	((*(volatile hw_lcdif_ctrl_t *) HW_LCDIF_CTRL_ADDR).U = (v))
#define HW_LCDIF_CTRL_CLR_WR(v) ((*(volatile hw_lcdif_ctrl_t *) HW_LCDIF_CTRL_CLR_ADDR).U = (v))
#define HW_LCDIF_CTRL_SET_WR(v) ((*(volatile hw_lcdif_ctrl_t *) HW_LCDIF_CTRL_SET_ADDR).U = (v))

#define HW_LCDIF_STAT_RD() 	((*(volatile unsigned int *) HW_LCDIF_STAT_ADDR))

#define HW_LCDIF_DATA_WR(v)  	((*(volatile unsigned int *) HW_LCDIF_DATA_ADDR) = (v))

#define HW_LCDIF_TRANSFER_COUNT_SET(v)  ((*(volatile unsigned int *) HW_LCDIF_TRANSFER_COUNT_ADDR) = (v))

#define HW_LCDIF_CTRL1_RD()     ((*(volatile hw_lcdif_ctrl1_t *) HW_LCDIF_CTRL1_ADDR).U)
#define HW_LCDIF_CTRL1_WR(v)    ((*(volatile hw_lcdif_ctrl1_t *) HW_LCDIF_CTRL1_ADDR).U = (v))
#define HW_LCDIF_CTRL1_CLR_WR(v) ((*(volatile hw_lcdif_ctrl1_t *) HW_LCDIF_CTRL1_CLR_ADDR).U = (v))
#define HW_LCDIF_CTRL1_SET_WR(v) ((*(volatile hw_lcdif_ctrl1_t *) HW_LCDIF_CTRL1_SET_ADDR).U = (v))

#define HW_LCDIF_TIMING_RD()    ((*(volatile hw_lcdif_Timing_t *) HW_LCDIF_TIMING_ADDR).U)
#define HW_LCDIF_TIMING_WR(v)    ((*(volatile hw_lcdif_Timing_t *) HW_LCDIF_TIMING_ADDR).U = (v))

typedef enum _hw_lcdif_DataSwizzle_t
{
    //! \brief No data swap, same as LITTLE_ENDIAN
    NO_SWAP=0,
    //! \brief No data swap, same as NO_SWAP
    LITTLE_ENDIAN=0,
    //! \brief Swap bytes 0,3 and 1,2, same as SWAP_ALL_BYTES
    BIG_ENDIAN_SWAP=1,
    //! \brief Swap bytes 0,3 and 1,2, same as BIG_ENDIAN_SWAP
    SWAP_ALL_BYTES=1,
    //! \brief Swap half-words
    HWD_SWAP=2,
    //! \brief Swap bytes within each half-word
    HWD_BYTE_SWAP=3,
} hw_lcdif_DataSwizzle_t;

typedef enum _hw_lcdif_DataShiftDir_t {
    //! \brief Shift all bytes to the left.
    DATA_SHIFT_LEFT = 0,
    //! \brief Shift all bytes to the right.
    DATA_SHIFT_RIGHT
} hw_lcdif_DataShiftDir_t;

//! Selects command or data mode for the buffer to be sent
//! See hw_lcdif_SetupDirectWrite() and hw_lcdif_SetDmaCommMode()
typedef enum _hw_lcdif_CommMode_t
{
    //! \brief Sets up bus for command mode, DCn signal low
    CMD_MODE = 0,
    //! \brief Sets up bus for data mode, DCn signal high
    DATA_MODE = 1
} hw_lcdif_CommMode_t;

typedef enum _hw_lcdif_Reset_t
{
    //! \brief LCD_RESET output signal is low
    LCDRESET_LOW=0,
    //! \brief LCD_RESET output signal is high
    LCDRESET_HIGH=1
} hw_lcdif_Reset_t;

//! Selects bus format.  
//! See hw_lcdif_SetBusMode() and hw_lcdif_GetBusMode()
typedef enum _hw_lcdif_BusMode_t
{
    //! \brief Data strobe is active low
    BUSMODE_8080=0,
    //! \brief Data strobe is active high
    BUSMODE_6800=1
} hw_lcdif_BusMode_t;

//! Selects LCDIF bus width.
//! See hw_lcdif_SetWordLength() and hw_lcdif_GetWordLength()
typedef enum _hw_lcdif_WordLength_t
{
    //! \brief Sets up 16 bit bus
    WORDLENGTH_16BITS = 0,
    //! \brief Sets up 8 bit bus
    WORDLENGTH_8BITS,
    WORDLENGTH_18BITS,
    WORDLENGTH_24BITS,
} hw_lcdif_WordLength_t;

typedef enum _hw_lcdif_BusWidth_t {
    //! \brief The data bus is 16-bits wide.
    HW_LCDIF_BUS_WIDTH_16BIT = 0,
    //! \brief The data bus is 8-bits wide.
    HW_LCDIF_BUS_WIDTH_8BIT,
    //! \brief The data bus is 18-bits wide.
    HW_LCDIF_BUS_WIDTH_18BIT,
    //! \brief The data bus is 24-bits wide.
    HW_LCDIF_BUS_WIDTH_24BIT,
} hw_lcdif_BusWidth_t;

void mpulcd_lcdif_reset(void);
void mpulcd_write_buffer(int mode, void *buf, int count);
void mpulcd_setup_pannel_register(hw_lcdif_CommMode_t mode, uint16_t data);
int mpulcd_init_panel(struct device *dev, dma_addr_t phys, int memsize,
        struct stmp3xxx_platform_fb_entry *pentry);
void mpulcd_release_panel(struct device *dev,
        struct stmp3xxx_platform_fb_entry *pentry);
void mpulcd_display_on( void );
void mpulcd_display_off( void );
void mpulcd_init_lcdif(void);
int mpulcd_blank_panel(int blank);
void mpulcd_init_panel_hw(void);

#endif

