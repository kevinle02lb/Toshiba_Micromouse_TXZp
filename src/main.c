/**
 ******************************************************************************
 * @file        main.c
 * @brief       Micromouse main firmware entry point
 * @version     V1.0.0
 * @date        05-05-2026
 *
 * @details
 *   Do change for deployment: Configure IR sensor values, Odometry wheel distance (wheelbase)
 * 
 * 
 *   Initializes system clock and all modules, then runs the main control loop.
 *   Control logic (PID, maze solving) is called from the 1 kHz tick.
 *
 * ___________________________________________________________________________
 * Hardware:
 *   - MCU:     TMPM4KNF10AFG (ARM Cortex-M4, 5V, 120MHz)
 *   - Motors:  TB67H450AFNG x2 + Pololu #5211 N20 30:1
 *   - Sensors: IR emitter/receiver pairs x4
 *   - Debug:   External CMSIS-DAP w/ Level Shifter
 *
 * ___________________________________________________________________________
 * Pin Assignment:
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │  PORT A  │ PA3  = 	 T32A00OUTA  (Left PWM IN1)            │
 *   │          │ PA4  = 	 T32A00OUTB  (Left PWM IN2)            │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT C  │ PC2  = 	 T32A30OUTA  (Right PWM IN1)           │
 *   │          │ PC3  = 	 T32A30OUTB  (Right PWM IN2)           │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT N  │ PN0  = 	 ENC0A  (Left encoder A)               │
 *   │          │ PN1  = 	 ENC0B  (Left encoder B)               │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT D  │ PD3  = 	 ENC2A  (Right encoder A)              │
 *   │          │ PD4  = 	 ENC2B  (Right encoder B)              │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT L  │ PL0  = 	 AINA16  (Far Left IR)                 │
 *   │          │ PL1  = 	 AINA15  (Left IR)                     │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT J  │ PJ0  =     AINC00  (Far Right IR)                │
 *   │          │ PJ1  = 	 AINC01  (Right IR)                    │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT U  │ PU0  =     Left IR Emitter                       │
 *   │          │ PU1  =     Far Left IR Emitter                   │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT G  │ PG4  =     Far Right IR Emitter                  │
 *   │          │ PG5  =     Right IR Emitter                      │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * ___________________________________________________________________________
 * Aliases (for quick reference):
 *   - PA3 = IN1, PA4 = IN2  (Left motor, T32A00OUTA/B)
 *   - PC2 = IN1, PC3 = IN2  (Right motor, T32A30OUTA/B)
 *   - PN0 = ENC0A, PN1 = ENC0B  (Left encoder)
 *   - PD3 = ENC2A, PD4 = ENC2B  (Right encoder)
 *   - PL0 = AINA16 (Far Left IR), PL1 = AINA15 (Left IR)
 *   - PJ1 = AINC01 (Right IR), PJ0 = AINC00 (Far Right IR)
 *   - PU1 = Far Left IR Emitter, PU0 = Left IR Emitter
 *   - PG5 = Right IR Emitter, PG4 = Far Right IR Emitter
 *
 * ___________________________________________________________________________
 * Notes:
 *   - MCU must be powered at 5V for ADC to function.
 *   - See TMPM4KNF10AFG datasheet section 7.2.
 *
 * ___________________________________________________________________________
 * References:
 *   - Product Info:  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *   - Datasheet:     https://toshiba.semicon-storage.com/info/TMPM4KNF10AFG_datasheet_en_20250516.pdf?did=155921&prodName=TMPM4KNF10AFG
 *
 * Copyright (c) [Kevin Le] 2026
 ******************************************************************************
 */


/* Inclutions */
#include "TMPM4KyA.h"
#include "system_TMPM4KyA.h"
#include "modules/Timebase.h"
#include "modules/Encoder.h"
#include "modules/Odometry.h"
#include "modules/Motion.h"
#include "modules/IrSensor.h"
#include "modules/FloodFill.h"
#include "modules/Navigator.h"


void ModuleInit(void);




int main()
{
	SystemInit();								/* Initialize system clock and peripherals */
	ModuleInit();
	

	while(1)
	{
		if ( Timebase_GetAndClear() ) 
		{
			Encoder_Update();
            Odometry_Update();
            Motion_Update();
            Navigator_Update();
        }
	}

	return 0;
}



/**
 * @brief  Initialize all modules in dependency order.
 * @note   Call once after SystemInit().
 *
 *   Each driver configures its own GPIO port internally, so no separate
 *   GPIO_Init() pass is needed. SysTick needs no init (stateless).
 *
 *   Order:
 *   1. Motion   -> Encoder + Motor (PWM T32A0/3, encoder ports) up first
 *   2. Odometry -> caches encoder position, so must follow Motion
 *   3. IR       -> DMAC + ADC + emitter ports; ADC uses SysTick_us (stateless)
 *   4. FloodFill-> planner; samples IR on first Plan(), so IR must precede it
 *   5. Navigator-> depends on FloodFill + Motion
 *   6. Timebase -> starts 1 kHz tick LAST
 */
void ModuleInit(void)
{
    Motion_Init();      // -> Encoder_Init() + Motor_Init()
    Odometry_Init();    // reads Encoder position
    IR_Init();          // -> DMAC_Init() + ADC_Init() + emitter ports
    FloodFill_Init();
    Navigator_Init();
    Timebase_Init();    // start tick last
}



