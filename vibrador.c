
// Control Vibrador por PWM en el PIN 2.0
// Recomendado un DC de mas del 80%


#include "driverlib.h"

#define TIMER_PERIOD 120  // fpwm = f*(1/TACRR0+1) = 100 Hz
#define DUTY_CYCLE1  100   // 100%

void main(void){
            //Stop WDT
           WDT_A_hold(WDT_A_BASE);

           //Pin configuration
           GPIO_setAsPeripheralModuleFunctionOutputPin(
               GPIO_PORT_P2,
               GPIO_PIN0  //P2.0
               );

           //Start timer A
           Timer_A_initUpModeParam initUpParam = {0};
           initUpParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK; // ACLK 12KHz
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
           initComp1Param.compareValue = (DUTY_CYCLE1/100)*TIMER_PERIOD;
           Timer_A_initCompareMode(TIMER_A1_BASE, &initComp1Param); //Timer TA1.1

           //Enter LPM0
           __bis_SR_register(LPM0_bits);

           //For debugger
           __no_operation();
       }
