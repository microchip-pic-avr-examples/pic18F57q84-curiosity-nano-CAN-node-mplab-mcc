/**
  TU16A Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tu16a.c

  @Summary
    This is the generated driver implementation file for the TU16A driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides implementations for driver APIs for TU16A.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.3
        Device            :  PIC18F57Q84
        Driver Version    :  3.0.0
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.20 and above or later
        MPLAB             :  MPLAB X 5.40
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "tu16a.h"
#include "can1.h"

void (*TU16A_InterruptHandler)(void);

void TU16A_Initialize(void)
{
    //Stop Timer
    TU16ACON0bits.ON = 0;
    //OSEN disabled; CLR disabled; LIMIT disabled; CAPT disabled; PRIF disabled; ZIF disabled; CIF disabled; 
    TU16ACON1 = 0x00;
    //EPOL non inverted; CSYNC disabled; START Continous mode - no start due to ERS, timer runs when ON = 1; RESET Timer counter resets at PR match; STOP ERS or PR do not stop the timer; 
    TU16AHLT = 0x0C;
    //CLK LFINTOSC; 
    TU16ACLK = 0x04;
    //ERS TUIN0PPS; 
    TU16AERS = 0x00;
    //PS 1; 
    TU16APS = 0x01;
    //PRH 60; 
    TU16APRH = 0x3C;
    //PRL 140; 
    TU16APRL = 0x8C;
    //TMRH 0; 
    TU16ATMRH = 0x00;
    //TMRL 0; 
    TU16ATMRL = 0x00;
    //CH16AB disabled; 
    TUCHAIN = 0x00;
    // Clearing IF flag before enabling the interrupt.
    TU16ACON1bits.PRIF = 0;
    TU16ACON1bits.ZIF = 0;
    TU16ACON1bits.CIF = 0;
    // Set Default Interrupt Handler
    TU16A_SetInterruptHandler(TU16A_DefaultInterruptHandler);
    // Enabled TUI TU16A interrupt
    PIE0bits.TU16AIE = 1;
    //ON enabled; CPOL falling edge; OM pulse mode; OPOL low; RDSEL read; PRIE enabled; ZIE disabled; CIE disabled; 
    TU16ACON0 = 0x84;
}

inline void TU16A_StartTimer(void)
{
    TU16ACON0bits.ON = 1;
}

inline void TU16A_StopTimer(void)
{
    TU16ACON0bits.ON = 0;
}

uint16_t TU16A_ReadCaptureValue(void)
{
    TU16ACON0bits.RDSEL = 0;
    return (uint16_t)(((uint16_t)TU16ACRH<< 8) | TU16ACRL);
}

uint16_t TU16A_CaptureOnCommand(void)
{
    TU16ACON1bits.CAPT = 1;
    while(TU16ACON1bits.CAPT == 1);
    /* 
       The above while loop is blocking code.
       If CSYNC=1 and ON=0 (or freeze=1), this bit will not clear unless there's
       clock provided. User must be careful here.
       Also when CSYNC=1 and timer clock is very slow, it will take forever
       to clear this bit because it takes 3 timer clocks to synchronize.
    */
    return TU16A_ReadCaptureValue();
}

uint16_t TU16A_ReadTimer(void)
{
    return (uint16_t)(((uint16_t)TU16ATMRH << 8) | (TU16ATMRL));
}

void TU16A_WriteTimer(uint16_t timerVal)
{
    TU16ATMRH = (uint8_t) (timerVal >> 8);
    TU16ATMRL = (uint8_t) (timerVal & 0xFF);
}

inline void TU16A_ClearCounter(void)
{
    TU16ACON1bits.CLR = 1;
    while(TU16ACON1bits.CLR == 1);
    /* 
       The above while loop is blocking code.
       If CSYNC=1 and ON=0 (or freeze=1), this bit will not clear unless there's
       clock provided. User must be careful here.
       Also when CSYNC=1 and timer clock is very slow, it will take forever
       to clear this bit because it takes 3 timer clocks to synchronize.
    */
}

void TU16A_SetPeriodValue(uint16_t prVal)
{
    TU16APRH = (uint8_t)((prVal >> 8) & 0xFF);
    TU16APRL = (uint8_t)(prVal & 0xFF);
}

inline void TU16A_EnablePRMatchInterrupt(void)
{
    TU16ACON0bits.PRIE = 1;
}

inline void TU16A_DisablePRMatchInterrupt(void)
{
    TU16ACON0bits.PRIE = 0;
}

inline void TU16A_EnableZeroInterrupt(void)
{
    TU16ACON0bits.ZIE = 1;
}

inline void TU16A_DisableZeroInterrupt(void)
{
    TU16ACON0bits.ZIE = 0;
}

inline void TU16A_EnableCaptureInterrupt(void)
{
    TU16ACON0bits.CIE = 1;
}

inline void TU16A_DisableCaptureInterrupt(void)
{
    TU16ACON0bits.CIE = 0;
}

inline bool TU16A_HasPRMatchOccured(void)
{
    return TU16ACON1bits.PRIF;
}

inline bool TU16A_HasResetOccured(void)
{
    return TU16ACON1bits.ZIF;
}

inline bool TU16A_HasCaptureOccured(void)
{
    return TU16ACON1bits.CIF;
}

inline bool TU16A_IsTimerRunning(void)
{
    return TU16ACON1bits.RUN;
}

inline void TU16A_EnableInterrupt(void)
{
    PIE0bits.TU16AIE = 1;
}

inline void TU16A_DisableInterrupt(void)
{
    PIE0bits.TU16AIE = 0;
}

inline bool TU16A_IsInterruptEnabled(void)
{
    return PIE0bits.TU16AIE;
}

inline void TU16A_ClearInterruptFlags(void)
{
    TU16ACON1bits.PRIF = 0;
    TU16ACON1bits.ZIF = 0;
    TU16ACON1bits.CIF = 0;
}

void TU16A_ISR()
{
    if(TU16ACON1bits.PRIF == 1)
    {
        TU16ACON1bits.PRIF = 0;
    }

    if(TU16A_InterruptHandler)
    {
        TU16A_InterruptHandler();
    }

    // add your TU16A interrupt custom code
}

void TU16A_SetInterruptHandler(void (* InterruptHandler)(void)){
    TU16A_InterruptHandler = InterruptHandler;
}

void TU16A_DefaultInterruptHandler(void){
    CAN_MSG_OBJ Transmission;
    uint8_t Transmit_Data[8]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
    Transmission.field.brs=CAN_BRS_MODE;
    Transmission.field.dlc=DLC_8;
    Transmission.field.formatType=CAN_FD_FORMAT;
    Transmission.field.frameType=CAN_FRAME_DATA;
    Transmission.field.idType=CAN_FRAME_STD;
    Transmission.msgId=0x585;
    Transmission.data=Transmit_Data;
    if(CAN_TX_FIFO_AVAILABLE == (CAN1_TransmitFIFOStatusGet(TXQ) & CAN_TX_FIFO_AVAILABLE))
                {
                    CAN1_Transmit(TXQ, &Transmission);
                }
    
}

