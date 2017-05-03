/* INCLUDE */
#include "driverlib.h"

/* DEFINE */
#define DEBUG 1

/* GLOBAL VARIABLES */
bool flag_fall = false;
bool flag_fuel = false;
bool flag_max_speed = false;
bool flag_turn = false;
bool flag_temp_ready = false;
bool flag_pulse_ready = false;
bool flag_test = false;

typedef struct data {
    int32_t temperature;
    int16_t temp_corp;
    //ToDo: Tipo de pulso
    bool fall;
} data;

/* FUNCTIONS */

/* DEFINITION: Configuration of the pins.
 *
 * INPUTS: None
 *
 * OUTPUTS: None
 *
 */
void configPins(void) {
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1);
}

/* DEFINITION: Include all the individual functions to initializate each peripheral modules: ADC, SPI, I2C, PWM, Bluetooth, etc.
 *
 * INPUTS: None
 *
 * OUTPUTS: None
 *
 */
void initHardware(void) {
    configPins();
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
}

/* DEFINITION: Calculate the pulse rate with the obtain samples.
 *
 * INPUTS: vector of samples
 *
 * OUTPUTS: pulse rate
 *
 */
void calcPulse(voi){

}

/* DEFINITION: obtain the data of pulse from MAX sensor
 *
 * INPUTS: None
 *
 * OUTPUTS: one sample for the vector
 *
 *  */
void getPulse(void) {

}

/* DEFINITION: obtain the corporal temperature from MAX sensor
 *
 * INPUTS: None
 *
 * OUTPUTS: int16_t tempCorp
 *
 *  */
int16_t getTempCorp(void) {
    return 0;
}

/* DEFINITION: put on/off the Rx Module
 *
 * INPUTS: None
 *
 * OUTPUTS: None
 *
 *  */
void toggleRx(void){

}

/* DEFINITION: read the data receive from the Rx and activate the necessary flag. Return the speed and the distance cover with the fuel available received.
 *
 * INPUTS: uint16_t * maxSpeed, uint16_t * kmFuel
 *
 * OUTPUTS: None
 *
 *  */
void rcvDataRx(uint16_t * speed, uint16_t * kmFuel){

}

/* DEFINITION: Read temperature from sensor connected to SPI interface
 *
 * INPUTS: None
 *
 * OUTPUTS: int32_t temperature
 *
 *  */
int32_t readTemperature(void){

    return 0;
}

/* DEFINITION: Enable or disable actuators depending on the active flags
 *
 * INPUTS: None
 *
 * OUTPUTS: None
 *
 *  */
void manageActuators(void){

}

/* DEFINITION: Wake-up Bluetooth IC, send data and get it to sleep again
 *
 * INPUTS: data txData
 *
 * OUTPUTS: None
 *
 *  */
void sendDataTx(data txData){

}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
    if(GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN1) == GPIO_PIN1){
        flag_test = true;
        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    }
 }

void main(void) {

    initHardware();
    __bis_SR_register(LPM3_bits + GIE);

    while(1){
        if(flag_test){
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            flag_test = false;
        }
    }
    //Go to sleep
}
