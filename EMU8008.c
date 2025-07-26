/*!
 * Copyright (c) 2025 by Gazelle
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *-----------------------------------------------------------------------
 * EMU8008 ROM, RAM, UART and inturrupt controler emulation firmware
 *
 * Target: EMU8008 with 8008 and PIC18F47Q42/83/84
 * IDE: MPLAB X v6.25
 * Compiler: MPLAB XC8 v3.0
 * 
 * Repsitory https://github.com/Gazelle8087/EMU8008
 *
 * 2025/07/26 Rev. 1.00 Initial release
 */

// CONFIG1
#pragma config FEXTOSC = OFF	// External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF	// Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON		// PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON		// Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON		// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#ifndef _18F47Q43
#pragma config JTAGEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF
#endif

// CONFIG3
#pragma config MCLRE = EXTMCLR	// MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON		// Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON		// IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF	// Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9	// Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF		// ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF	// PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF		// WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC		// WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF		// Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF		// Storage Area Flash enable bit (SAF disabled)
#ifdef _18F47Q43
#pragma config DEBUG = OFF		// Background Debugger (Background Debugger disabled)	
#endif

// CONFIG8
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF		// SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF		// Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF		 // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#ifndef _18F47Q43
// CONFIG9
#pragma config BOOTPINSEL = RC5 // CRC on boot output pin selection (CRC on boot output pin is RC5)
#pragma config BPEN = OFF       // CRC on boot output pin enable bit (CRC on boot output pin disabled)
#pragma config ODCON = OFF      // CRC on boot output pin open drain bit (Pin drives both high-going and low-going signals)

// CONFIG11
#pragma config BOOTSCEN = OFF   // CRC on boot scan enable for boot area (CRC on boot will not include the boot area of program memory in its calculation)
#pragma config BOOTCOE = HALT   // CRC on boot Continue on Error for boot areas bit (CRC on boot will stop device if error is detected in boot areas)
#pragma config APPSCEN = OFF    // CRC on boot application code scan enable (CRC on boot will not include the application area of program memory in its calculation)
#pragma config SAFSCEN = OFF    // CRC on boot SAF area scan enable (CRC on boot will not include the SAF area of program memory in its calculation)
#pragma config DATASCEN = OFF   // CRC on boot Data EEPROM scan enable (CRC on boot will not include data EEPROM in its calculation)
#pragma config CFGSCEN = OFF    // CRC on boot Config fuses scan enable (CRC on boot will not include the configuration fuses in its calculation)
#pragma config COE = HALT       // CRC on boot Continue on Error for non-boot areas bit (CRC on boot will stop device if error is detected in non-boot areas)
#pragma config BOOTPOR = OFF    // Boot on CRC Enable bit (CRC on boot will not run)

// CONFIG12
#pragma config BCRCPOLT = hFF   // Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of BCRCPOL are 0xFF)

// CONFIG13
#pragma config BCRCPOLU = hFF   // Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of BCRCPOL are 0xFF)

// CONFIG14
#pragma config BCRCPOLH = hFF   // Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of BCRCPOL are 0xFF)

// CONFIG15
#pragma config BCRCPOLL = hFF   // Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of BCRCPOL are 0xFF)

// CONFIG16
#pragma config BCRCSEEDT = hFF  // Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of BCRCSEED are 0xFF)

// CONFIG17
#pragma config BCRCSEEDU = hFF  // Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of BCRCSEED are 0xFF)

// CONFIG18
#pragma config BCRCSEEDH = hFF  // Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of BCRCSEED are 0xFF)

// CONFIG19
#pragma config BCRCSEEDL = hFF  // Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of BCRCSEED are 0xFF)

// CONFIG20
#pragma config BCRCEREST = hFF  // Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of BCRCERES are 0xFF)

// CONFIG21
#pragma config BCRCERESU = hFF  // Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of BCRCERES are 0xFF)

// CONFIG22
#pragma config BCRCERESH = hFF  // Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of BCRCERES are 0xFF)

// CONFIG23
#pragma config BCRCERESL = hFF  // Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of BCRCERES are 0xFF)

// CONFIG24
#pragma config CRCPOLT = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of CRCPOL are 0xFF)

// CONFIG25
#pragma config CRCPOLU = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of CRCPOL are 0xFF)

// CONFIG26
#pragma config CRCPOLH = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of CRCPOL are 0xFF)

// CONFIG27
#pragma config CRCPOLL = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of CRCPOL are 0xFF)

// CONFIG28
#pragma config CRCSEEDT = hFF   // Non-Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of CRCSEED are 0xFF)

// CONFIG29
#pragma config CRCSEEDU = hFF   // Non-Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of CRCSEED are 0xFF)

// CONFIG30
#pragma config CRCSEEDH = hFF   // Non-Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of CRCSEED are 0xFF)

// CONFIG31
#pragma config CRCSEEDL = hFF   // Non-Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of CRCSEED are 0xFF)

// CONFIG32
#pragma config CRCEREST = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of CRCERES are 0xFF)

// CONFIG33
#pragma config CRCERESU = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of CRCERES are 0xFF)

// CONFIG34
#pragma config CRCERESH = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of CRCERES are 0xFF)

// CONFIG35
#pragma config CRCERESL = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of CRCERES are 0xFF)
#endif

#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 64000000UL

// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

// UART3 Transmit
void putch(char c) {
	while(!U3TXIF);		// Wait or Tx interrupt flag set
	U3TXB = c;			// Write data
}

// UART3 Recive
int getch(void) {
	while(!U3RXIF);		// Wait for Rx interrupt flag set
	return U3RXB;		// Read data
}
// ======================  wiring define  ======================================

#define		S0		RA0
#define		S1		RA1
#define		S2		RA2
#define		READY	RA5
#define		SYNC	RB3
#define		PHI2	RB4
#define		PHI1	RB5
#define		INT		RC5

// ======================  Global ram and rom  =================================
//8008 RAM equivalent
#ifdef _18F47Q43
#define RAM_SIZE	0x1F00
#else
#define RAM_SIZE	0x3000
#endif

#define	INT_PTR		0x41					// ram[0x41]:initial value of INT pointer
#define	INTLOG_PTR	0x42					// ram[0x42]:initial value of INTLOG pointer

#define	RAM_start	6
unsigned char		ram[RAM_SIZE] __at(RAM_start * 0x100);	// RAM
const unsigned char rom[] __at(0x10000);
unsigned char		T2_addr __at(0x500);	// Processor status during T2
unsigned char		INT_PC __at(0x501);		// Instruction pointer for INT controler
unsigned char		INTLOG __at(0x502);		// Inturrpt log pointer
unsigned int		i;
// ======================  main routine  =======================================
void main(void) {

	// System initialize
	OSCFRQ		= 0x08;		// 64MHz internal OSC

	ANSELA		= 0;		// Disable analog function
	LATA		= 0x20;
	TRISA		= 0xdf;		// Set as input
	WPUA		= 0xff;		// weak pull up

	ANSELB		= 0;		// Disable analog function
	TRISB		= 0xff;		// Set as input
	WPUB		= 0xff;		// weak pull up

	ANSELC		= 0;		// Disable analog function
	LATC		= 0;
	TRISC		= 0xdf;		// Set as input
	WPUC		= 0xff;		// weak pull up

	ANSELD		= 0;		// Disable analog function
	TRISD		= 0xff;		// Set as input
	WPUD		= 0xff;		// weak pull up

	ANSELE		= 0;		// Disable analog function
	TRISE		= 0xff;		// Set as input
	WPUE		= 0xff;		// weak pull up

//======== Generate 2 pahse CPU clock using PWM ================================
//	#define		Hispeed		// enable Hispeed for 8008-1

	ANSELB5		= 0;		// Disable analog function
	TRISB5		= 0;		// PWM output pin
	RB5PPS		= 0x18;		// PWM1S1P1_OUT => RB5 => PHI1

	ANSELB4		= 0;		// Disable analog function
	TRISB4		= 0;		// PWM output pin
	RB4PPS		= 0x1A;		// PWM2S1P1_OUT => RB4 => PHI2

	PWM1CON		= 0x00;		// EN=0, LD=0
	PWM2CON		= 0x00;		// EN=0, LD=0
	PWM1CLK		= 0x02;		// Clock source Fsoc
	PWM2CLK		= 0x02;		// Clock source Fsoc
	PWM1GIE		= 0x00;		// interrupt disable
	PWM2GIE		= 0x00;		// interrupt disable
	PWM1S1CFG	= 0x00;		// (POL1, POL2)= 0, PPEN = 0 MODE = 0 (Left Aligned mode)
	PWM2S1CFG	= 0x00;		// (POL1, POL2)= 0, PPEN = 0 MODE = 0 (Left Aligned mode)

#ifdef	Hispeed
	PWM1PR		= 81;		// period 1281.25 nsec (82*15.625) 780kHz
	PWM1S1P1	= 23;		// PHI1 width  359.375 nsec (23*15.625)
	PWM2S1P1	= 23;		// PHI2 width  359.375 nsec (23*15.625)
#else
	PWM1PR		= 127;		// period 2000 nsec (128*15.625) 500kHz
	PWM1S1P1	= 44;		// PHI1 width  687.5   nsec (44*15.625)
	PWM2S1P1	= 35;		// PHI2 width  546.875 nsec (35*15.625)
#endif

	PWM2PR		= PWM1PR;	// PWM2 period same as PWM1
	PWM1CON		= 0x84;		// EN=1, LD=1 it takes 125nsec(=2*62.5)
	NOP();					// 3 NOP(); takes 62.5nsec
	NOP();					// 4
	NOP();					// 5
	NOP();					// 6
	NOP();					// 7
	NOP();					// 8
	NOP();					// 9 Time difference 62.5*9 = 562.5nsec
#ifndef	Hispeed
	NOP();					// 10
	NOP();					// 11
	NOP();					// 12
	NOP();					// 13
	NOP();					// 14
	NOP();					// 15
	NOP();					// 16
	NOP();					// 17 Time difference 62.5*17 = 1062.5nsec
#endif
	PWM2CON		= 0x84;		// EN=1, LD=1
//==============================================================================

//========== CLC pin assign ===========
	CLCIN0PPS	= 0x00;		// RA0 <- S0
	CLCIN1PPS	= 0x01;		// RA1 <- S1
	CLCIN2PPS	= 0x0C;		// RB4 <- PHI2
	CLCIN3PPS	= 0x0D;		// RB5 <- PHI1
	CLCIN4PPS	= 0x02;		// RA2 <- S2
//	CLCIN5PPS	=
	CLCIN6PPS	= 0x0B;		// RB3 <- SYNC
//	CLCIN7PPS	=

//========== Latch T1 ==========
    CLCSELECT	= 0;		// select CLC1

	CLCnSEL0	= 6;		// CLCIN6PPS <- SYNC
	CLCnSEL1	= 2;		// CLCIN2PPS <- PHI2
	CLCnSEL2	= 53;		// CLC3(T1 decode)
	CLCnSEL3	= 127;		// NC

	CLCnGLS0	= 1 + 4;	// C <- PHI2 invert or SYNC invert
	CLCnGLS1	= 0x20;		// D <- CLC3 no invert
	CLCnGLS2	= 0;		// R <- 0
	CLCnGLS3	= 0;		// S <- 0

	CLCnPOL		= 0x00;		// Q no invert D no invert CLK no invert
	CLCnCON		= 0x94;		// D-FF, positive edge interrupt

//========== Latch T1I ==========
	CLCSELECT	= 1;		// select CLC2

	CLCnSEL0	= 6;		// CLCIN6PPS <- SYNC
	CLCnSEL1	= 2;		// CLCIN2PPS <- PHI2
	CLCnSEL2	= 54;		// CLC4(T1I decode)
	CLCnSEL3	= 127;		// NC

	CLCnGLS0	= 1 + 4;	// C <- PHI2 invert or SYNC invert
	CLCnGLS1	= 0x20;		// D <- CLC3 no invert
	CLCnGLS2	= 0;		// R <- 0
	CLCnGLS3	= 0;		// S <- 0

	CLCnPOL		= 0x00;		// Q no invert D no invert CLK no invert
	CLCnCON		= 0x94;		// D-FF, positive edge interrupt

//========== decode T1 S0=0 S1=1 S2=0 ==========
    CLCSELECT	= 2;		// select CLC3

	CLCnSEL0	= 0;		// CLCIN0PPS <- S0
	CLCnSEL1	= 1;		// CLCIN1PPS <- S1
	CLCnSEL2	= 4;		// CLCIN4PPS <- S2
	CLCnSEL3	= 127;		// NC

	CLCnGLS0	= 1;		// S0 invert
	CLCnGLS1	= 8;		// S1 no invert
	CLCnGLS2	= 0x10;		// S2 invert
	CLCnGLS3	= 0x40;		// 1(0 inv)

	CLCnPOL		= 0x00;		// Not inverted
	CLCnCON		= 0x82;		// 4 input AND, no interrupt

//========== decode T1I S0=0 S1=1 S2=1 ==========
	CLCSELECT	= 3;		// select CLC4

	CLCnSEL0	= 0;		// CLCIN0PPS <- S0
	CLCnSEL1	= 1;		// CLCIN1PPS <- S1
	CLCnSEL2	= 4;		// CLCIN4PPS <- S2
	CLCnSEL3	= 127;		// NC

	CLCnGLS0	= 1;		// S0 invert
	CLCnGLS1	= 8;		// S1 no invert
	CLCnGLS2	= 0x20;		// S2 no invert
	CLCnGLS3	= 0x40;		// 1(0 inv)

	CLCnPOL		= 0x00;		// Not inverted
	CLCnCON		= 0x82;		// 4 input AND, no interrupt

//=====================================================
	// UART3 initialize
	U3BRG		= 416;		// 9600bps @ 64MHz
	U3RXEN		= 1;		// Receiver enable
	U3TXEN		= 1;		// Transmitter enable

	// UART3 Receiver
	ANSELA7		= 0;		// Disable analog function
	TRISA7		= 1;		// RX set as input
	U3RXPPS		= 0x07;		// RA7->UART3:RX3;

	// UART3 Transmitter
	ANSELA6		= 0;		// Disable analog function
	LATA6		= 1;		// Default level
	TRISA6		= 0;		// TX set as output
	RA6PPS		= 0x26;		// RA6->UART3:TX3;

	U3ON		= 1;		// Serial port enable

	printf("\r\nEMU8008 clock speed %3.0fkHz\r\n",(float)64000/(PWM1PR+1));
	printf("PWM1 -> RB5 -> PHI1 width %3.0f nsec\r\n",(float)PWM1S1P1*15.625);
	printf("PWM2 -> RB4 -> PHI2 width %3.0f nsec\r\n\r\n",(float)PWM2S1P1*15.625);

	for(i = 0; i < RAM_SIZE; i++) {
		ram[i] = rom[i];
	}

    // Unlock IVT
    IVTLOCK	= 0x55;
    IVTLOCK	= 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x00;

    // Default IVT base address
    IVTBASE 	= 0x000008;

    // Lock IVT
    IVTLOCK	= 0x55;
    IVTLOCK	= 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x01;

    // CLC VI enable
    CLC1IF = 0;		// Clear the CLC1 interrupt flag
    CLC2IF = 0;		// Clear the CLC2 interrupt flag
    CLC3IF = 0;		// Clear the CLC3 interrupt flag
    CLC4IF = 0;		// Clear the CLC4 interrupt flag
    CLC5IF = 0;		// Clear the CLC5 interrupt flag
    CLC6IF = 0;		// Clear the CLC6 interrupt flag
    CLC7IF = 0;		// Clear the CLC7 interrupt flag
    CLC8IF = 0;		// Clear the CLC8 interrupt flag

    CLC1IE = 1;		// Enable CLC1 interrupt T1
    CLC2IE = 1;		// Enable CLC2 interrupt T1I
    CLC3IE = 0;		//
    CLC4IE = 0;		//
    CLC5IE = 0;		//
    CLC6IE = 0;		//
    CLC7IE = 0;		//
    CLC8IE = 0;		//

    GIE			= 1;	// Global interrupt enable
    BSR			= 0;	// BSR 0 fixed
    TBLPTRU 	= 1;	// TBLTPU always 1 fixed (8008ROM at 10000h)
    CLCSELECT	= 0;	// CLCSELECT usually 0

    // 8008 start
	INTLOG		= ram[INTLOG_PTR];	// initial value of INTLOG pointer
	INT_PC		= ram[INT_PTR];		// initial value of INT_PC counter
	INT	= 1;						// INT=H and start 8008
    while(1);
}
//==============================================================================
void __interrupt(irq(CLC1),base(8)) T1_ISR(){

	CLC1IF	= 0;							// clear CLC1 interrupt flag
	WREG	= PORTD;						// data bus shows lower byte of instruction code or data
	TBLPTRL	= WREG;							// Set TBLPTRL in advance
	FSR0L	= WREG;							// Set FSR0L in advance

	while (!SYNC);							// wait for T2
	while (SYNC);							// wait for T2 OUT DATA ready

	WREG	= PORTD;						// data bus shows higher byte of instruction code or data
	T2_addr	= WREG;
	TBLPTRH = WREG & 0x3f;					// mask upper 2bit(CPU status) and set TBLPTRH
	FSR0H	= TBLPTRH + RAM_start;			// mask upper 2bit(CPU status) and set FSR0H

	if (!(T2_addr & 0b01000000)){			// memory read
		if (TBLPTRH >= (RAM_SIZE/0x100)){	// address >= RAM_SIZE then ROM read
			asm("tblrd	*			\n"		// Table read
				"movff	TABLAT,LATD	\n");	// LATD = rom[TBLPTR]
		}
		else {								// RAM read
			asm("movff	indf0,LATD	\n"); 	// LATD = ram[FSR0]
		}

		while (!PHI1);						// wait for T3 DATA IN timing

		TRISD	= 0x00;						// port OUT
		while (SYNC);						// wait for SYNC = 0
		TRISD	= 0xff;						// port IN
	}
	else if (!(T2_addr & 0b10000000)){		// IO operation

		switch (T2_addr){					// data bus shows OP code its self qin case of I/O

			case (0x41):					// IN 00
				LATD	= PIR9;				// check status

				while (!PHI1);				// wait for T3 DATA IN timing

				TRISD	= 0x00;				// PORTD OUT
				while (SYNC);				// wait for SYNC = 0
				TRISD	= 0xff;				// PORTD IN
				break;

			case (0x43):					// IN 01
				LATD	= U3RXB;			// UART receive

				while (!PHI1);				// wait for T3 DATA IN timing

				TRISD	= 0x00;				// PORTD OUT
				while (SYNC);				// wait for SYNC = 0
				TRISD	= 0xff;				// PORTD IN
				break;

			case (0x51):					// OUT 08h
				INT		= 1;				// set INT signal and let CPU enter INT procedure
				INT_PC	= ram[INT_PTR];		// set INT_PC
				INTLOG	= ram[INTLOG_PTR];
				break;

			case (0x53):					// OUT 09h
				INT		= 0;				// reset INT signal
				break;

			case (0x61):					// OUT 10h
				U3TXB	= TBLPTRL;			// UART send
				break;

			default:
				break;
		}
	}
	else if (TBLPTRH < RAM_SIZE/0x100){		// RAM write

		while (!SYNC);						// wait for T3
		while (SYNC);						// wait for SYNC = 0

		asm("movff	PORTD,indf0	\n");		// ram[FSR0] = PORTD
	}
}
//==============================================================================
void __interrupt(irq(CLC2),base(8)) T1I_ISR(){

//	INT		= 0;							// INT signal should be reset by 8008 instruction OUT 09h
	CLC2IF		= 0;						// clear CLC2 interrupt flag
	WREG		= PORTD;					// data bus shows lower byte of instruction code or data
	TBLPTRL		= WREG;						// Set TBLPTRL in advance
	FSR0L		= WREG;						// Set FSR0L in advance
	ram[INTLOG]	= WREG;
	INTLOG++;

	while (!SYNC);							// wait for T2
	while (SYNC);							// wait for T2 OUT DATA ready

	WREG	= PORTD;						// data bus shows higher byte of instruction code or data
	T2_addr	= WREG;
	TBLPTRH = WREG & 0x3f;					// mask upper 2bit(CPU status) and set TBLPTRH
	FSR0H	= TBLPTRH + RAM_start;			// mask upper 2bit(CPU status) and set FSR0H

	if (!(T2_addr & 0b01000000)){			// memory read

		LATD	= ram[INT_PC];				// fetch INT instruction at ram[INT_PC]
		INT_PC++;							// increment INT PC

		while (!PHI1);						// wait for T3 DATA IN timing

		TRISD	= 0x00;						// port OUT
		ram[INTLOG]	= T2_addr;
		INTLOG++;
		while (SYNC);						// wait for SYNC = 0
		TRISD	= 0xff;						// port IN
	}
	else if (!(T2_addr & 0b10000000)){		// IO operation

		switch (T2_addr){

			case (0x41):					// IN 00
				LATD	= PIR9;				// check status

				while (!PHI1);				// wait for T3 DATA IN timing

				TRISD	= 0x00;				// PORTD OUT
				ram[INTLOG]	= T2_addr;
				INTLOG++;
				while (SYNC);				// wait for SYNC = 0
				TRISD	= 0xff;				// PORTD IN
				break;

			case (0x43):					// IN 01
				LATD	= U3RXB;			// UART receive

				while (!PHI1);				// wait for T3 DATA IN timing

				TRISD	= 0x00;				// PORTD OUT
				ram[INTLOG]	= T2_addr;
				INTLOG++;
				while (SYNC);				// wait for SYNC = 0
				TRISD	= 0xff;				// PORTD IN
				break;

			case (0x51):					// OUT 08h
				ram[INTLOG]	= T2_addr;
				INTLOG++;
				INT		= 1;				// set INT signal
				break;

			case (0x53):					// OUT 09h and exit INT procedure
				INT		= 0;				// reset INT signal
				ram[INTLOG]	= T2_addr;
				INTLOG	= ram[INTLOG_PTR];
				INT_PC	= ram[INT_PTR];		// set INT_PC
				break;

			case (0x61):					// OUT 10h
				U3TXB	= TBLPTRL;			// UART send
				ram[INTLOG]	= T2_addr;
				INTLOG++;
				break;

			default:
				ram[INTLOG]	= T2_addr;
				INTLOG++;
				break;
		}
	}
	else if (TBLPTRH < RAM_SIZE/0x100){		// RAM write

		while (!SYNC);						// wait for T3
		ram[INTLOG]	= T2_addr;
		INTLOG++;
		while (SYNC);						// wait for SYNC = 0

		asm("movff	PORTD,indf0	\n");		// ram[FSR0] = PORTD
	}
	else {
		ram[INTLOG]	= T2_addr;
		INTLOG++;
	}
}
//==============================================================================

const unsigned char rom[] __at(0x10000) = {

//	Simple loop back test
//	nop   in 0  ani   1     jz 0001           in 01 out16 jmp 0000
//	0xc0, 0x41, 0x24, 0x01, 0x68, 0x01, 0x00, 0x43, 0x61, 0x44, 0x00, 0x00,

#include "sc1.txt"	
};
