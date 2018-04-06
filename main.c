
//#include "inc/tm4c123gh6pm.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "inc/hw_memmap.h"   // defines the memory address locations for various modules and peripherals
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"
#include "driverlib/mpu.h"
#include "driverlib/adc.h"
#include "inc/hw_adc.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/hw_mpu9150.h"
#include "PID.c"
#include "driverlib/timer.h"


/**********************************************
 *              SYSTEM VARIABLES
 **********************************************/

// UART Functionality Variables
#define SERIAL_BAUD_RATE 115200

char input[50];
char inputUART1[50];
char dest[50];  // Dummy string for copying and shortening
int inputIndex = 0;
int inputIndexUART1 = 0;
char incomingByteUART0;
char incomingByteUART1;
bool foundEOT;
bool ready = false;

// ADC Functionality Variables
uint32_t ADC0Value[4];
volatile uint32_t ADC0Average;
float pressureResistor = 159.28;
float pressure_PSI;

#define pressureReadCount 5

// Gyroscope Functionality Variables
volatile bool gyroDone = false;
// float Accel[3], Gyro[3], Magneto[3];
tI2CMInstance i2c1;
tMPU9150 gyro1;
float Gyro;

// RTI Functionality Variables
 uint32_t RTIPeriod;


 // Desired Values
 float dpressure_PSI;

/**********************************************
 *       FLOAT COMMUNICATION PROTOCOL
 **********************************************/

// Float receiving union
typedef union floatData {
    float f;
    long l;
}floatConvert;

// When defining identifier characters, note that identifier characters for sending floats from the Ti to the Java program
// will be used in the file name. This means that the identifier has to be a valid filename character.
// Describe as either received, sent or both.

#define PRESSURE_RESISTOR 0x61      // Received     'a'
#define PRESSURE_ADC 0x62           // Sent         'b'
#define PRESSURE_PSI 0x63           // Sent         'c'
#define DPRESSURE_PSI 0x64          // Both         'd'
#define DEPTH_PID_OUTPUT 0x65       // Sent         'e'
#define DEPTH_P 0x66                // Both         'f'
#define DEPTH_I 0x67                // Both         'g'
#define DEPTH_D 0x68                // Both         'h'


/**********************************************
 *       FLOAT COMMUNICATION PROTOCOL
 **********************************************/
PID depthPID;


/**********************************************
 *      MOTOR CONTROL VARIABLES
 **********************************************/

long motor[8] = {PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3, PWM_OUT_4, PWM_OUT_5, PWM_OUT_6, PWM_OUT_7};

uint32_t PWM_L;
uint32_t PWM_R;
uint32_t PWM_FL;
uint32_t PWM_FR;
uint32_t PWM_BL;
uint32_t PWM_BR;

float PWM_Vertical; // temporary variable for receiving a PID value

// Motor description definitions, used for ease in programming for the setMotor function
#define MOTOR1 0x01
#define MOTOR2 0x02
#define MOTOR3 0x04
#define MOTOR4 0x08
#define MOTOR5 0x10
#define MOTOR6 0x20
#define MOTOR7 0x40
#define MOTOR8 0x80

// Multiply your desired PWM width in ms by this constant to get the appropriate pulse width
#define PulseConvert 0.781718

/**********************************************
 *   Code Implementation for Configurations
 **********************************************/

void InitUART0(void){
    // Enables the UART0 Module and the GPIO Port A Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pins for receiver and the transmitter
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Connect the pins to the GPIO_PORTA_BASE
    // Set bits 0 and 1 of GPIO_PORTA's AFSEL register to turn on alternate functions
    // Set bits 0 and 1 of GPIO_PORTA's DEN register to enable digital values
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART Connection to have a baud rate of 115200, 8 bit words, one stop char, and no parity character
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), SERIAL_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable(); // enable processor interrupts
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);  // only enable RX and TX interrupts

    // Configure UART using UARTStdio Library to enable the use of UARTStdio functions
    UARTStdioConfig(0, SERIAL_BAUD_RATE, SysCtlClockGet());
}

void InitUART1(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), SERIAL_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable();
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);


}

void InitPWM(void){
    // Activate clock for the PWM module
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable the PWM Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enabled for PE4 & PE5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enabled for PC4 & PC5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enabled for PB4,5,6,7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);


    // Configure the specified pins for PWM functionality
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);

    // Set PWM functionality on GPIO module for each of the specified pins
    // Sets the AFSEL bits for each port and the respective pins
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    // Configure PWM settings on each PWM generator. Set for countdown timing and asynchronous updates
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //set the period to 1565 which somehow creates a PWM period of roughly 2000us. Someone should crunch the math on this
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1562); // Sets the period for PWM_OUT_0/1
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 1562); // Sets the period for PWM_OUT_2/3
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 1562); // Sets the period for PWM_OUT_4/5
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 1562); // Sets the period for PWM_OUT_6/7

    //set pulse width to 1500us. Multiply desired width in us by 0.781718 to obtain proper clock values
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 1500*0.78125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 1500*0.78125);

    //enable pwm generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    //enable pwm module to start modifying pins for pwm output
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT , true);

}

void InitRGB(void){
    // RGB Pins are located at Port F pins 1, 2, and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Port F is connected to the PWM module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Configure pins for PWM functionality
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    // Sets alternate function register for each pin
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    // Configures the PWM generator for specified configurations
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Gen 0 drives pin 4/5
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Gen 1 drives pin 6/7

    // Sets period for PWM
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 255); // Sets the period for PWM_OUT_4/5
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 255); // Sets the period for PWM_OUT_6/7

    // Sets original width to be 0
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 150);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 150);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 150);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}

void InitI2C0(void){

    // enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    // HWREG(I2C0_BASE + I2C_O_FIFODATA) = 80008000;
}

void InitPressureADC(){
    // Enable ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Choose to use sequencer 1. Sequencer 1 takes 4 samples and each sample requires configuration
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

    // Configure samples / steps
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    // Enable ADC Sequencer 1
    ADCSequenceEnable(ADC0_BASE, 1);
}

void GyroCallBack(void *pvCallbackData, uint_fast8_t ui8Status){
    // Reference TivaWare Sensor Library

    // See if an error occurred
    if (ui8Status != I2CM_STATUS_SUCCESS){
        // Error Handle
        UARTprintf("[MCU] An error occurred in the I2C initialization for the gyroscope\n");
    }

    // Indicate that the MPUTransaction has been complete
    gyroDone = true;
}

void InitGyro(){
    SetRBG(0,100,100);

    // Initialize I2C Master Instance
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}

    //enable GPIO peripheral that contains I2C0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable I2C0_BASE
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    // Set the slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, 0b110100, true);

    // Store the value to be sent
    I2CMasterDataPut(I2C0_BASE, 0x3B);

    // Send a single value
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    //
    // Delay until transmission completes
    //
    while(I2CMasterBusBusy(I2C0_BASE))
    {
    }

    /*
    MPU9150Init(&gyro1, &i2c1, 0x68, GyroCallBack, 0);

    // wait until the gyroscope has been configured
    while(!gyroDone){}
    */

    SetRBG(100,100,100);

}

void InitRTI(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // TimerLoadSet(TIMER0_BASE, TIMER_A, RTIPeriod);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/10) - 1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
}


/**********************************************
 *          Function Implementations
***********************************************/


//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        while(!I2CMasterBusy(I2C0_BASE));
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        while(!I2CMasterBusy(I2C0_BASE));
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        uint8_t i;
        for(i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            while(!I2CMasterBusy(I2C0_BASE));
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        while(!I2CMasterBusy(I2C0_BASE));
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        //"close" variable args list
        va_end(vargs);
    }
}

//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}

void setMotors(unsigned char motorSel, uint32_t PWM){
    if (motorSel & MOTOR1)        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWM*PulseConvert);
    if ((motorSel & MOTOR2) >> 1) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM*PulseConvert);
    if ((motorSel & MOTOR3) >> 2) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWM*PulseConvert);
    if ((motorSel & MOTOR4) >> 3) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWM*PulseConvert);
    if ((motorSel & MOTOR5) >> 4) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWM*PulseConvert);
    if ((motorSel & MOTOR6) >> 5) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PWM*PulseConvert);
    if ((motorSel & MOTOR7) >> 6) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM*PulseConvert);
    if ((motorSel & MOTOR8) >> 7) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWM*PulseConvert);
}

void killMotors(){
    setMotors(0xFF, 1500);
}

uint32_t readADC(){
    // Clear the interrupt register bit
    ADCIntClear(ADC0_BASE, 1);

    // Start ADC Sample Process
    ADCProcessorTrigger(ADC0_BASE, 1);

    // Wait until sampling is complete
    while(!ADCIntStatus(ADC0_BASE, 1, false)){}

    ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);

    // Return the average of all values
    return (ADC0Value[0] + ADC0Value[1] + ADC0Value[2] + ADC0Value[3] + 2) /4;
}

uint32_t multiReadADC(int count){
    // Saturate input between 1 - 20
    if (count > 20) count = 20;
    if (count < 1) count = 1;
    uint32_t ADCValue;
    int i;
    for (i = 0; i < count; i++){
        ADCValue  += readADC();
    }
    return ADCValue / count;
}

float pressureADC(uint32_t ADCvalue){
    float voltage = (float) ADCvalue;
    pressure_PSI = voltage/25 - 24.195;
    return pressure_PSI;

}

float pressureToDepth(float pressure){
    float depth = pressure;
    return depth;
}

void SetRBG(int R, int B, int G){
    // Check if received values exceed 0 - 255 range
    if (R > 255) R = 255;
    if (B > 255) B = 255;
    if (G > 255) G = 255;

    // Check if pins should be on or off (even at a pulse width of 1, there is light. At a pulse width of 0, there is max light)
    // correct for a color value of 0 by turning off the pin. Else set the color
    if (R == 0){
        PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);
    }else{
        if (R > 0){
            PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, R);
        }
    }

    if (B == 0){
        PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
    }else{
        if (B > 0){
            PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, B);
        }
    }

    if (G == 0){
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
    }else{
        if (G > 0){
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, G);
        }
    }
}

void floatSend(char identifier, float value){
    floatConvert floatSender;
    floatSender.f = value;
    unsigned char UARTsend[5];
    UARTsend[0] = identifier;
    UARTsend[1] = (floatSender.l >> 24) & 0xFF;
    UARTsend[2] = (floatSender.l >> 16) & 0xFF;
    UARTsend[3] = (floatSender.l >> 8) & 0xFF;
    UARTsend[4] = (floatSender.l) & 0xFF;
    int i;
    if ((floatSender.l >> 23) & 0x1FF){


    }
    for (i = 0; i<5; i++){
        UARTCharPut(UART0_BASE, UARTsend[i]);
    }
    UARTprintf("*&~");
}

void resetInput(){
    // reset the input string
    int i;
    for (i = 0 ; i < inputIndex ; i++){
        input[i] = '\0';
    }
    inputIndex = 0;
    input[0] = '\0';

    foundEOT = false;
}

void manualMotor(){

}

void pressureTest(){
    float ADCvalue;
    resetInput();
    UARTprintf("[MCU] Pressure Read-out Test\n");
    while(!foundEOT){
        ADCvalue = multiReadADC(pressureReadCount);
        floatSend(PRESSURE_ADC, (float)ADCvalue);
        floatSend(PRESSURE_PSI, (float)(pressureADC(ADCvalue)));
        SysCtlDelay(2000000);
    }
    UARTprintf("[MCU] Exiting pressure read-out test~\n");
}

void LEDtest(){
    resetInput();
    int R = 0;
    int B = 0;
    int G = 0;
    int cCombo = 0; // cCombo variable provides switch table selections for color combination
    SetRBG(R,B,G); // turn off all LEDs initially

    UARTprintf("[MCU] LED Test~\n");
    while (!foundEOT){

        // Throttle Red
        for ( R = 0 ; R<255 ; R = R + 5){
            SetRBG(R,B,G);
            UARTprintf("R: %u, B: %u, G: %u\n", R, B, G);
            SysCtlDelay(500000);
            if (foundEOT) break;
        }
        R = 0;

        // Throttle Blue
        for ( B = 0 ; B < 255 ; B = B + 5){
            SetRBG(R,B,G);
            UARTprintf("R: %u, B: %u, G: %u\n", R, B, G);
            SysCtlDelay(500000);
            if (foundEOT) break;
        }
        B = 0;

        // Throttle Green
        for ( G = 0 ; G < 255 ; G = G + 5){
            SetRBG(R,B,G);
            UARTprintf("R: %u, B: %u, G: %u\n", R, B, G);
            SysCtlDelay(500000);
            if (foundEOT) break;
        }
        G = 0;

        // Different Color Combination Throttles
        // 0. R Throttle, B 50%, G 0%
        // 1. R Throttle, B 0% , G 50%
        // 2. R Throttle, B 50%, G 50%
        // 3. R 50%, B Throttle, G 0%
        // 4. R 0% , B Throttle, G 50%
        // 5. R 50%, B Throttle, G 50%
        // 6. R 50%, B 0% , G Throttle
        // 7. R 0% , B 50%, G Throttle
        // 8. R 50%, B 50%, G Throttle
        for (cCombo = 0 ; cCombo < 9; cCombo++){
            switch (cCombo){
            case 0:
                B = 130;
                G = 0;
                break;
            case 1:
                B = 0;
                G = 130;
                break;
            case 2:
                B = 130;
                G = 130;
                break;
            case 3:
                R = 0;
                G = 130;
                break;
            case 4:
                R = 130;
                G = 0;
                break;
            case 5:
                R = 130;
                G = 130;
                break;
            case 6:
                R = 130;
                B = 0;
                break;
            case 7:
                R = 0;
                B = 130;
                break;
            case 8:
                R = 130;
                B = 130;
            }

            switch (cCombo){
            case 0:
            case 1:
            case 2:
                // Throttle Red with current BG settings
                for ( R = 0 ; R<255 ; R = R + 5){
                    SetRBG(R,B,G);
                    UARTprintf("R: %u, B: %u, G: %u\n", R, B, G);
                    SysCtlDelay(500000);
                    if (foundEOT) break;
                }
                break;
            case 3:
            case 4:
            case 5:
                // Throttle Blue
                for ( B = 0 ; B < 255 ; B = B + 5){
                    SetRBG(R,B,G);
                    UARTprintf("R: %u, B: %u, G: %u\n", R, B, G);
                    SysCtlDelay(500000);
                    if (foundEOT) break;
                }
                break;
            case 6:
            case 7:
            case 8:
                for ( G = 0 ; G < 255 ; G = G + 5){
                    SetRBG(R,B,G);
                    UARTprintf("R: %u, B: %u, G: %u\n", R, B, G);
                    SysCtlDelay(500000);
                    if (foundEOT) break;
                }
            }
            if (foundEOT) break;
        }

    }
    UARTprintf("[MCU] Exit LED Test");

}

void PIDTest(){
    resetInput();
    UARTprintf("[MCU] PID Test~\n");
    depthPID.myInput = &pressure_PSI;
    depthPID.myOutput = &PWM_Vertical;
    depthPID.mySetpoint = &dpressure_PSI;
    depthPID.pOnE = 1;
    depthPID.inAuto = AUTOMATIC;
    depthPID.controllerDirection = DIRECT;
    depthPID.outputSum = 1500;
    depthPID.outMin = 1100;
    depthPID.outMax = 1900;
    InitRTI();
    while(!foundEOT){}
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    killMotors();
    UARTprintf("[MCU] Exiting PID Test~\n");
}

void motorTest(){
    resetInput();
    UARTprintf("[MCU]  Motor Run-through Test\n");
    int i,j=1;
    char printMotor[11];
    for (i = 0 ; i < 8 ; i++){
        switch (i){
        case 0:
            strcpy(printMotor,"Left");
            break;
        case 1:
            strcpy(printMotor,"Right");
            break;
        case 2:
            strcpy(printMotor,"Front Left");
            break;
        case 3:
            strcpy(printMotor,"Front Right");
            break;
        case 4:
            strcpy(printMotor,"Back Left");
            break;
        case 5:
            strcpy(printMotor,"Back Right");
            break;
        case 6:
            strcpy(printMotor,"Other 1");
            break;
        case 7:
            strcpy(printMotor,"Other 2");
        }
        UARTprintf("[MCU]  Throttle %s Forward~\n", printMotor);
        setMotors(j, 1600);
        while(!foundEOT);
        resetInput();
        UARTprintf("[MCU]  Throttle %s Backward~\n", printMotor);
        setMotors(j, 1400);
        while(!foundEOT);
        setMotors(j, 1500);
        resetInput();
        strcpy(printMotor,"");
        j = j << 1;
    }
    killMotors();
}

void systemTest(){
    resetInput();
    bool systemTest = true;
    int menuSel;

    UARTprintf("[MCU System Test]\nSelect a system test:\n0. Exit Test\n1. Motor Run-through\n2. Pressure Read-out\n3. Battery Voltage Read-out\n4. LED Test\n5. Gyroscope Read-out \n6. PID Test");
    UARTprintf("~");
    while(systemTest){
        if(foundEOT){
            UARTprintf("[MCU_CMD]: \"%s\"\n",input);  // ECHO received command
            input[inputIndex-1] = '\0'; // remove the end of transmission character

            // check if too many characters has been entered
            if ((inputIndex-1)> 1){
                UARTprintf("[MCU] Incorrect string length. The menu selection only accepts 1 character strings");
                UARTprintf("[MCU] String length is %u ~", inputIndex);
            }else{
                // check if the entered menu option is valid
                menuSel = (int)input[0] - 48; // subtract 48 to turn the character into the proper integer
                if (menuSel > 6 || menuSel < 0){
                    UARTprintf("[MCU] Incorrect menu selection. The menu selection only accepts values between 0 - 6");
                    UARTprintf("[MCU] Entered value is %u ~", menuSel);
                }else{
                    // Enter different test mode functions
                    switch (menuSel){
                    case 0:
                        // menu selection 0 exits the system test function
                        systemTest = false;
                        break;
                    case 1:
                        // menu selection 1 calls the motor run-through test
                        SetRBG(0,130,130);
                        motorTest();
                        SetRBG(10,10,10);
                        break;
                    case 2:
                        // menu selection 2 calls the pressure read-out test
                        SetRBG(130,0,130);
                        pressureTest();
                        SetRBG(10,10,10);
                        break;
                    case 3:
                        // menu selection 3 calls the battery voltage read-out test
                        SetRBG(10,10,10);

                        SetRBG(10,10,10);
                        break;
                    case 4:
                        // menu selection 4 calls the LED test
                        LEDtest();
                        break;
                    case 5:
                        // menu selection 5 calls the gyroscope read-out test
                        SetRBG(130,0,10);
                        // I2CSend(0x20,2,0x09,0xFF);

                        SetRBG(10,10,10);
                        break;
                    case 6:
                        // menu selection 6 calls the PID test
                        SetRBG(10,0,130);
                        PIDTest();
                        SetRBG(0,0,0);
                    }

                }
            }
            resetInput();
            if (menuSel != 0){
                UARTprintf("[MCU System Test]\nSelect a system test:\n0. Exit Test\n1. Motor Run-through\n2. Pressure Read-out\n3. Battery Voltage Read-out\n4. LED Test\n5. Gyroscope Read-out \n6. PID Test");
                UARTprintf("~");
            }
        }

    }
    UARTprintf("[MCU] Exiting System Test~\n");
}

int main (void){
    // Set the clock rate to be 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    InitUART0();
    InitUART1();
    InitRGB();
    InitPWM();
    InitPressureADC();
    InitI2C0();

    depthPID.inAuto = MANUAL;
    dpressure_PSI = 3;
    depthPID.kp = 20;

    UARTprintf("[MCU]: MCU Started~\n");
    SetRBG(255,255,255);

    // MAIN PROGRAM LOOP
    while(1){

        if (foundEOT){

            UARTprintf("[MCU_CMD]: \"%s\"  \n",input);  // ECHO received command

            if (strcmp(input, "help") == 0){
                UARTprintf("[MCU Help Menu]\n");
                UARTprintf("   Command   Description\n");
                UARTprintf("   help      prints the help dialogue with the list of available commands\n");
                UARTprintf("   config    prints the current MCU setting and configurations\n");
                UARTprintf("   mm        enters manual mode where the user can manually alter thruster values\n");
                UARTprintf("   mtr       begins a motor test where the user manually edit thruster values\n");
                UARTprintf("   test      begins a pressure test that reads out voltage readings from PE3\n");

                UARTprintf("~");

            }else if (strcmp(input, "config") == 0){
                char buffer[50];
                UARTprintf("[MCU Configurations]\n");
                UARTprintf("          Clock Rate: %u\n", SysCtlClockGet());
                UARTprintf("   PWM Period M0Gen0: %u\n", PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0));
                UARTprintf("   PWM Period M0Gen1: %u\n", PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
                UARTprintf("   PWM Period M0Gen2: %u\n", PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2));
                UARTprintf("   PWM Period M0Gen3: %u\n", PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3));
                UARTprintf("[Depth Hold Variables]\n");
                UARTprintf("        Depth P-Gain: ");
                sprintf(buffer, "%0.3f",depthPID.kp);
                UARTprintf("%s\n",buffer);
                UARTprintf("        Depth I-Gain: ");
                sprintf(buffer, "%0.3f",depthPID.ki);
                UARTprintf("%s\n",buffer);
                UARTprintf("        Depth D-Gain: ");
                sprintf(buffer, "%0.3f",depthPID.kd);
                UARTprintf("%s\n",buffer);
                UARTprintf("Depth Setpoint(pssr): ");
                sprintf(buffer, "%0.3f",dpressure_PSI);
                UARTprintf("%s\n",buffer);
                UARTprintf("~");

            }else if (strcmp(input, "mm") == 0){
                UARTprintf("[MCU] Manual Mode has not been written~\n");

            }else if (strcmp(input, "mtr") == 0){
                SetRBG(100,0,0);
                manualMotor(); // Enters manual mode loop and only exits when the exit status is received
                SetRBG(100,100,100);
            }else if (strcmp(input, "test") == 0){
                SetRBG(10,10,10);
                systemTest();
                SetRBG(100,100,100);
            }else{
                UARTprintf("[MCU] Unrecognized command. Type \"help\" for all possible commands~\n");
            }

            resetInput();
        }
    }
}

/**********************************************
 *         Interrupt Implementation
 **********************************************/

void UART0IntHandler(void){
    // the UART interrupt is triggered whenever a character is received in the UART FIFO

    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true); // get our interrupt status
    UARTIntClear(UART0_BASE, ui32Status);         // clear the asserted interrupts

    // Loop through the UART FIFO whenever there are characters available in the UART FIFO
    while(UARTCharsAvail(UART0_BASE)){
        incomingByteUART0 = UARTCharGetNonBlocking(UART0_BASE); // Store the first character in the UART FIFO
        input[inputIndex++] = incomingByteUART0; // Append the character to the input char array

        // UARTCharPut(UART0_BASE, incomingByteUART0); // echo character

        // Look out for the EoT and signal the flag when an EoT is found, signifying for the MCU to parse the command
        if (incomingByteUART0 == '~'){
            input[strlen(input)-1] = '\0';
            if (input[0] == '&' && input[1] == '*'){
                // UARTprintf("[MCU] Received float is: %s \n", input);
                long receivedLong = ((long) input[3] << 24) | ((long) input[4] << 16) | ((long) input[5] << 8) | ((long) input[6]);
                floatConvert fValue;
                fValue.l = receivedLong;
                switch (input[2]){
                case (PRESSURE_RESISTOR):
                        pressureResistor = fValue.f;
                        UARTprintf("[MCU]  Pressure Resistor Updated to: ");
                        break;
                case (DPRESSURE_PSI):
                        dpressure_PSI = fValue.f;
                        UARTprintf("[MCU]  Desired Pressure Updated to: ");
                        break;
                case (DEPTH_P):
                        depthPID.kp = fValue.f;
                        UARTprintf("[MCU]  Depth Proportional Gain Updated to: ");
                        break;
                case (DEPTH_I):
                        depthPID.ki = fValue.f;
                        UARTprintf("[MCU]  Depth Integral Gain Updated to: ");
                        break;
                case (DEPTH_D):
                        depthPID.kd = fValue.f;
                        UARTprintf("[MCU]  Depth Derivative Gain Updated to: ");
                        break;
                default:
                }
                char buffer[50];
                sprintf(buffer, "%0.3f",fValue.f);
                UARTprintf(buffer);
                UARTprintf("~\n");
                resetInput();
            }else{
                foundEOT = true;
            }

        }
    }
}

void UART1IntHandler(void){
    uint32_t ui32Status;
    char stringValue[50];
    ui32Status = UARTIntStatus(UART1_BASE, true); // get our interrupt status
    UARTIntClear(UART1_BASE, ui32Status);         // clear the asserted interrupts

    // Loop through the UART FIFO whenever there are characters available in the UART FIFO
    while(UARTCharsAvail(UART1_BASE)){
        incomingByteUART1 = UARTCharGetNonBlocking(UART1_BASE); // Store the first character in the UART FIFO
        inputUART1[inputIndexUART1++] = incomingByteUART1; // Append the character to the input char array
        if(incomingByteUART1 == '~'){
            strncpy(stringValue, inputUART1+1, 7);
            UARTprintf("[MCU] Gyro Received: %s\n", inputUART1);
            inputUART1[inputIndexUART1] = '\0';
            // Gyro = atof(inputUART1);
            int i;
            for (i = 0 ; i < inputIndexUART1 ; i++){
                inputUART1[i] = '\0';
            }
            inputIndexUART1 = 0;
            inputUART1[0] = '\0';
        }
    }
}

void Timer0IntHandler(void){
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Read the current state of the GPIO pin and
    // write back the opposite state
    uint32_t PWM_sent;
    float printPressure;
    printPressure = pressureADC(multiReadADC(pressureReadCount));
    PID_Compute(&depthPID);

    if (PWM_Vertical > 1500){
        PWM_Vertical += 50;
    }else{
        PWM_Vertical -= 50;
    }
    floatSend(DEPTH_PID_OUTPUT, PWM_Vertical);
    floatSend(PRESSURE_PSI, printPressure);

    setMotors(MOTOR1 | MOTOR2, (unsigned int) PWM_Vertical);
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)){
        SetRBG(0,0,0);
    } else {
        SetRBG(10,0,130);
    }
}


