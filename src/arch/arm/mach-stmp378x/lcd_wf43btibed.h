#ifndef __LCD_WF43_H
#define __LCD_WF43_H

//! Entry mode register of the SSD1289
typedef union _EntryMode
{
    uint16_t V;
    struct {
        uint8_t LG:3;
        uint8_t AM:1;
        uint8_t ID:2;
        uint8_t TY:2;
        uint8_t DMode:2;
        uint8_t WMode:1;
        uint8_t OEDef:1;
        uint8_t TRANS:1;
        uint8_t DFM:2;
        uint8_t VSMode:1;
    } B;
} SSD1289EntryMode;

//! Display control register of the SSD1289
typedef union _SSD1289DisplayControl
{
    uint16_t V;
    struct {
        uint8_t D0:1;
        uint8_t D1:1;
        uint8_t Reserved:1;
        uint8_t CM:1;
        uint8_t DTE:1;
        uint8_t GON:1;
        uint8_t Reserved2:2;
        uint8_t SPT:1;
        uint8_t VLE:2;
        uint8_t PT:2;
        uint8_t Reserved3:3;
    } B;
} SSD1289DisplayControl;

typedef union _SSD1289LCDDrivingWaveformControl
{
    uint16_t V;
    struct {
        uint8_t NW:8;
        uint8_t WSMD:1;
        uint8_t EOR:1;
        uint8_t BC:1;
        uint8_t ENWS:1;
        uint8_t FLD:1;
        uint8_t Reserved:3;
    } B;
} SSD1289LCDDrivingWaveformControl;

//! Frame cycle control register of the SSD1289
typedef union _FrameCycleControl
{
    uint16_t V;
    struct {
        uint8_t RTN:4;
        uint8_t SRTN:1;
        uint8_t SDIV:1;
        uint8_t DIV:2;
        uint8_t EQ:3;
        uint8_t Reserved:1;
        uint8_t SDT:2;
        uint8_t NO:2;
    } B;
} SSD1289FrameCycleControl;

typedef union _DriverOutputControl
{
    uint16_t V;
    struct {
        uint16_t MUX:9;
        uint8_t TB:1;
        uint8_t SM:1;
        uint8_t BGR:1;
        uint8_t CAD:1;
        uint8_t REV:1;
        uint8_t RL:1;
        uint8_t Reserved:1;
    } B;
} SSD1289DriverOutputControl;

typedef struct SSD1289Region {
    uint16_t SourceWin;
    uint16_t SourceAddress;

    uint16_t GateWinStart;
    uint16_t GateWinStop;
    uint16_t GateAddress;
} SSD1289Region_t;

#endif

