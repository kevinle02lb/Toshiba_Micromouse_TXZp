/**
 ***********************************************************
 * @file     main.c
 * @brief    Micromouse main firmware entry point
 * @version  V1.0.0
 * @date     05-05-2026
 *
 * @details
 *   Initializes system clock, GPIO, PWM motor control,
 *   ADC IR sensing, and A-ENC32 hardware encoders.
 *   Runs maze solving algorithm in main loop.
 *
 * ___________________________________________________________________________
 * Hardware:
 *   - MCU:     TMPM4KNF10AFG (ARM Cortex-M4, 5V, 120MHz)
 *   - Motors:  TB67H450AFNG x2 + Pololu #5211 N20 30:1
 *   - Sensors: IR emitter/receiver pairs x4
 *   - Debug:   External CMIS-DAP w/ Level Shifter
 *
 * ___________________________________________________________________________
 * Pin Assignment:
 *   - PA3, PA4      Left motor PWM (T32A ch0)
 *   - PC2, PC3      Right motor PWM (T32A ch3)
 * 
 *   - PN0, PN1      Left encoder A/B (A-ENC32 ch0)
 *   - PD3, PD4      Right encoder A/B (A-ENC32 ch2)
 * 
 *   - PL0, PL1 	 Left IR ADC inputs (AINA16, AINA15)
 * 	 - PJ1, PJ0 	 Right IR ADC inputs (AINC01, AINC00)
 * 
 *   - PU0-1, PG4-5  IR emitter MOSFET control
 * 
 * ___________________________________________________________________________
 * Aliasses:
 *  - PA3 = IN1, PA4 = IN2 (Left motor)    			T32A00OUTA, T32A00OUTB
 * 	- PC2 = IN1, PC3 = IN2 (Right motor)  			T32A30OUTA, T32A30OUTB	
 * 
 *  - PN0 = ENC0A, PN1 = ENC0B (Left encoder)
 *  - PD3 = ENC2A, PD4 = ENC2B (Right encoder)
 * 
 *  - PL0 = Far Left IR, PL1 = Left IR (Left ADC)
 *  - PJ1 = Right IR, PJ0 = Far Right IR (Right ADC)
 * 
 *  - PU1 = Far Left IR Emitter, PU0 = Left IR Emitter (Left IR Emitters)
 *  - PG5 = Right IR Emitter, PG4 = Far Right IR Emitter (Right IR Emitters)
 * 
 * ___________________________________________________________________________
 * 
 * @note
 *   MCU must be powered at 5V for ADC to function.
 *   See TMPM4KNF10AFG (TMPM4K Group(2)) datasheet section 7.2.
 * 	 
 *   Reference Documents (Toshiba):
 *   - Product Info:  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *   - TMPM4K Group(2) Datasheet: https://toshiba.semicon-storage.com/info/TMPM4KNF10AFG_datasheet_en_20250516.pdf?did=155921&prodName=TMPM4KNF10AFG
 *
 * Copyright (c) [Kevin Le] 2026
 ******************************************************************************
 */


/* Inclutions */
#include "TMPM4KyA.h"
#include "system_TMPM4KyA.h"
#include "modules/Timebase.h"
#include "modules/IrSensor.h"
#include "modules/Encoder.h"
#include "modules/Motor.h"

void ModuleInit(void);




int main()
{
	SystemInit();								/* Initialize system clock and peripherals */
	ModuleInit();
	

	while(1)
	{
		if ( TickCtrl_GetAndClear() ) 
		{
            /* control Logic to be implemented */
        }
	}

	return 0;
}



/**
 * @brief  Initialize all relevant Modules
 * @note   Must be called after SystemInit()
 */
void ModuleInit()
{
	Timebase_Init();
	IR_Init();
	Encoder_Init();
	Motor_Init();
}




