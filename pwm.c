#include "driverlib.h"

#define TIMER_PERIOD 512  // fpwm = f*(1/TACRR0+1) = 48 KHz
#define DUTY_CYCLE1  384  //75%
#define DUTY_CYCLE2  128  //25%

    void main(void)
    {
        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        //Pin configuration
        GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P2,
            GPIO_PIN0  //P2.0
            );

        GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P1,
            GPIO_PIN4  //P1.4
            );

        //Start timer A
        Timer_A_initUpModeParam initUpParam = {0};
        initUpParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK; //SMCLK solo activo en algunos LPM, sino ACLK
        initUpParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1; //1,2,4,8
        initUpParam.timerPeriod = TIMER_PERIOD; // Periodo 48 KHz
        initUpParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
        initUpParam.captureCompareInterruptEnable_CCR0_CCIE =
            TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
        initUpParam.timerClear = TIMER_A_DO_CLEAR;
        initUpParam.startTimer = false;
        Timer_A_initUpMode(TIMER_A1_BASE, &initUpParam);

        Timer_A_startCounter(TIMER_A1_BASE,
                             TIMER_A_UP_MODE
                             );

        //Initialize compare mode to generate PWM1
        Timer_A_initCompareModeParam initComp1Param = {0}; //CCR1
        initComp1Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
        initComp1Param.compareInterruptEnable =
            TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
        initComp1Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
        initComp1Param.compareValue = DUTY_CYCLE1;
        Timer_A_initCompareMode(TIMER_A1_BASE, &initComp1Param); //Timer TA1.1

        //Initialize compare mode to generate PWM2
        Timer_A_initCompareModeParam initComp3Param = {0}; //CCR3
        initComp3Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
        initComp3Param.compareInterruptEnable =
            TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
        initComp3Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
        initComp3Param.compareValue = DUTY_CYCLE2;
        Timer_A_initCompareMode(TIMER_A0_BASE, &initComp3Param); //Timer TA0.3

        //Enter LPM0
        __bis_SR_register(LPM0_bits);

        //For debugger
        __no_operation();
    }
