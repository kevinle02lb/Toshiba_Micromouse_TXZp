/**
 * @file        Motor.c
 * @brief       Motor control implementation for TB67H450AFNG
 * @version     V1.0.0
 * @date        08-06-2026
 *
 * @details
 *   See motor.h for wiring and truth table details.
 *
 *   Direction-change policy (RM-T32A-C §4.6.1):
 *   OUTCRA1 / OUTCRB1 may only be written while RUNA = 0.
 *
 *   ┌─────────────────────────┬─────────────────────────────┐
 *   │  Scenario               │  Action                     │
 *   ├─────────────────────────┼─────────────────────────────┤
 *   │  Same dir, moving       │  Live duty write only       │
 *   │  Same dir, BRAKE/STOP   │  No-op                      │
 *   │  Any direction change   │  Stop → reconfigure → Start │
 *   └─────────────────────────┴─────────────────────────────┘
 *
 *   Output mapping (IN1=TimerA/OUTA, IN2=TimerB/OUTB):
 *   ─────────────────────────────────────────────────────────
 *   FORWARD : IN1=LOW,  IN2=PWM  (datasheet calls this reverse)
 *   REVERSE : IN1=PWM,  IN2=LOW  (datasheet calls this forward)
 *   BRAKE   : IN1=HIGH, IN2=HIGH (motor shorted, fast stop)
 *   STOP    : IN1=LOW,  IN2=LOW  (standby, freewheel)
 *
 *  @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Motor.h"
#include "drivers/timer32A.h"

#define MAX_SPEED   100U
#define PERIOD_L    ((uint16_t)T32A_CH0_PERIOD)
#define PERIOD_R    ((uint16_t)T32A_CH3_PERIOD)

static direction_t motor_left_dir  = STOP;
static direction_t motor_right_dir = STOP;

/* ==========================================================================
 *   Private helper
 * ========================================================================== */

/**
 * @brief  Convert speed % to timer compare value.
 * @param  speed   0–100 (%)
 * @param  period  Timer period in counts
 * @return Duty value in [0, period-1]
 */
static inline uint16_t Speed_ToDuty(uint8_t speed, uint16_t period)
{
    if (speed >= MAX_SPEED)
    {
        return (uint16_t)(period - 1U);
    }

    return (uint16_t)(period - 1U - ((uint32_t)speed * (uint32_t)(period - 1U) / (uint32_t)MAX_SPEED));     /* Inverted: Period - duty ticks */
}

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

void Motor_Init(void)
{
    T32A0_Init(T32A_CH0_PERIOD);   /* Left motor */
    T32A3_Init(T32A_CH3_PERIOD);   /* Right motor */

    /* Safe initial state: both inputs LOW (standby) */
    T32A0_SetOutCRA1(T32A_OUTPUT_LOW);
    T32A0_SetOutCRB1(T32A_OUTPUT_LOW);
    T32A3_SetOutCRA1(T32A_OUTPUT_LOW);
    T32A3_SetOutCRB1(T32A_OUTPUT_LOW);

    motor_left_dir  = STOP;
    motor_right_dir = STOP;
}

/* ==========================================================================
 *   Start / Stop
 * ========================================================================== */

void Motor_Start(void)
{
    T32A0_Start();
    T32A3_Start();
}

void Motor_Stop(void)
{
    T32A0_Stop();
    T32A3_Stop();

    motor_left_dir  = STOP;
    motor_right_dir = STOP;
}

/* ==========================================================================
 *   Left motor
 * ========================================================================== */

void Motor_SetLeft(direction_t dir, uint8_t speed)
{
    uint16_t duty = Speed_ToDuty(speed, PERIOD_L);

    /* Fast path: same direction, just update duty */
    if (dir == motor_left_dir)
    {
        if (dir == FORWARD)      /* IN2 duty */
        {
            T32A0_SetTimerB0(duty);
        }
        else if (dir == REVERSE) /* IN1 duty */
        {
            T32A0_SetTimerA0(duty);
        }
        /* BRAKE / STOP: nothing to update */
        return;
    }

    /* Full path: stop, reconfigure, restart */
    T32A0_Stop();

    switch (dir)
    {
        /* IN2=PWM, IN1=LOW → OUT2=H, OUT1=L → current OUT2→OUT1 → M1(+) positive */
        case FORWARD:
            T32A0_SetOutCRA1(T32A_OUTPUT_LOW);       /* IN1 = LOW  */
            T32A0_SetOutCRB1(T32A_OUTPUT_PPG);       /* IN2 = PWM  */
            T32A0_SetTimerA0(0U);
            T32A0_SetTimerB0(duty);
            break;

        /* IN1=PWM, IN2=LOW → OUT1=H, OUT2=L → current OUT1→OUT2 → M1(+) negative */
        case REVERSE:
            T32A0_SetOutCRA1(T32A_OUTPUT_PPG);       /* IN1 = PWM  */
            T32A0_SetOutCRB1(T32A_OUTPUT_LOW);       /* IN2 = LOW  */
            T32A0_SetTimerA0(duty);
            T32A0_SetTimerB0(0U);
            break;

        /* IN1=HIGH, IN2=HIGH → both outputs L → motor shorted */
        case BRAKE:
            T32A0_SetOutCRA1(T32A_OUTPUT_HIGH);
            T32A0_SetOutCRB1(T32A_OUTPUT_HIGH);
            T32A0_SetTimerA0(0U);    /* Clean compare regs */
            T32A0_SetTimerB0(0U);
            break;

        /* IN1=LOW, IN2=LOW → Hi-Z → standby */
        case STOP:
            T32A0_SetOutCRA1(T32A_OUTPUT_LOW);
            T32A0_SetOutCRB1(T32A_OUTPUT_LOW);
            T32A0_SetTimerA0(0U);
            T32A0_SetTimerB0(0U);
            break;
    }

    motor_left_dir = dir;
    T32A0_Start();
}

/* ==========================================================================
 *   Right motor
 * ========================================================================== */

void Motor_SetRight(direction_t dir, uint8_t speed)
{
    uint16_t duty = Speed_ToDuty(speed, PERIOD_R);

    /* Fast path */
    if (dir == motor_right_dir)
    {
        if (dir == FORWARD)
        {
            T32A3_SetTimerB0(duty);
        }
        else if (dir == REVERSE)
        {
            T32A3_SetTimerA0(duty);
        }
        return;
    }

    T32A3_Stop();

    switch (dir)
    {
        case FORWARD:
            T32A3_SetOutCRA1(T32A_OUTPUT_LOW);
            T32A3_SetOutCRB1(T32A_OUTPUT_PPG);
            T32A3_SetTimerA0(0U);
            T32A3_SetTimerB0(duty);
            break;

        case REVERSE:
            T32A3_SetOutCRA1(T32A_OUTPUT_PPG);
            T32A3_SetOutCRB1(T32A_OUTPUT_LOW);
            T32A3_SetTimerA0(duty);
            T32A3_SetTimerB0(0U);
            break;

        case BRAKE:
            T32A3_SetOutCRA1(T32A_OUTPUT_HIGH);
            T32A3_SetOutCRB1(T32A_OUTPUT_HIGH);
            T32A3_SetTimerA0(0U);
            T32A3_SetTimerB0(0U);
            break;

        case STOP:
            T32A3_SetOutCRA1(T32A_OUTPUT_LOW);
            T32A3_SetOutCRB1(T32A_OUTPUT_LOW);
            T32A3_SetTimerA0(0U);
            T32A3_SetTimerB0(0U);
            break;
    }

    motor_right_dir = dir;
    T32A3_Start();
}

/* ==========================================================================
 *   Motor set
 * ========================================================================== */

void Motor_Set(motor_t motor, direction_t dir, uint8_t speed)
{
    switch (motor)
    {
        case MOTOR_LEFT:   Motor_SetLeft(dir, speed);   break;
        case MOTOR_RIGHT:  Motor_SetRight(dir, speed);  break;
        default:                                        break;
    }
}