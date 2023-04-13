#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
//#include "math.h"

// I2C configuration        
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

// initialising ports, protocols, etc
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?
    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only
    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3
                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

// Initializing port H for stepper motor 
void PortH_Init(void){
    //Use PortM pins for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};   // allow time for clock to stabilize
    GPIO_PORTH_DIR_R |= 0xFF;                                       // make PN0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;                                  // disable alt funct on PN0
  GPIO_PORTH_DEN_R |= 0xFF;                                     // enable digital I/O on PN0
                                                                                                    // configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0xFF;                                  // disable analog functionality on PN0      
    return;
}

// Initialising for configuring ports for onboard assigned LED
void PortF0F4_Init(void) { // F4 - measurement status, F0 - Additional status
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {};//allow time for clock to stabilize
    GPIO_PORTF_DIR_R = 0b00010001; //Make PF0 and PF4 outputs, to turn on LED's
    GPIO_PORTF_DEN_R = 0b00010001;
    return;
}

// Initiasiling Port M for buttons - start/stop spinning and start/stop recording
void PortM0M1M2M3_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0) {}; //allow time for clock to stabilize
    GPIO_PORTM_DIR_R = 0b00000000; // Make PM0:PM3 inputs, reading if the button is pressed or not
    GPIO_PORTM_DEN_R = 0b00001111; // Enable PM0:PM3
    return;
}

// Give clock to Port J and initalize as input GPIO-- 0 and 1 for now 
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;	 								//? Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//??Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor
}
// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;       // initialize counter

	
		GPIO_PORTJ_IS_R = 0;    	// (Step 1) PJ1 is edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;   	//     			PJ1 is not both edges 
		GPIO_PORTJ_IEV_R = 0;    	//     			PJ1 falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;     // 					clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;    	// 					arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;           // (Step 2) enable interrupt 51 in NVIC
	
		NVIC_PRI12_R = 0xA0000000; 				// (Step 4) set interrupt priority 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}
//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention
//	 				in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
void GPIOJ_IRQHandler(void){
  FallingEdges = FallingEdges + 1;	// Increase the global counter variable ;Observe in Debug Watch Window
	GPIO_PORTJ_ICR_R = 0x02;     					// acknowledge flag by setting proper bit in ICR register
  if ((GPIO_PORTJ_DATA_R & 0b00000010) == 0){
 SysTick_Wait10ms(10);
				return;
             }
      if ((GPIO_PORTJ_DATA_R & 0b00000001) == 0){
while(sensorState==0){
			status = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
      FlashLED2();
		}
          }
										//Flash the LED D2 one time
													//Flash the LED D2 one time
	GPIO_PORTJ_ICR_R = 0x03;     					// Acknowledge flag by setting proper bit in ICR register
}

// init function calls
void FlashLED(int flash_count);
void spin_cw(uint8_t dataReady, uint16_t Distance);
void spin_ccw(uint8_t dataReady, uint16_t Distance);
void take_measurement(uint8_t dataReady, uint16_t Distance);
void bus_pulse();
/* *********************************************************************************************/
// ******************************* !! MAIN CODE BEGINS !! *********************************** //
/* ********************************************************************************************/

/*
This program interfaces with a Time of Flight sensor using I2C and takes measurements after rotating the sensor on a motor.
The motor is programmed to spin clockwise then return home. 
An onboard button PJ1 sets an interrupt routine in the program to allow for measurements to be taken.
Run this after running the respective python file to allow for measurements to be recorded.
*/ 

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
int totalsteps = 0;
// flags for measure and spin button toggle
int startstop_measure = 0;
int startstop_spin = 0;

// call for bus speed measurement 
void bus_pulse(){
  while(1){
		GPIO_PORTN_DATA_R=0b00001111;
		SysTick_Wait(20000);
		GPIO_PORTN_DATA_R=0b00000000;
		SysTick_Wait(20000);
	}
}

// Flashes D3 - takes parameter for number of flashes
void FlashLED(int flash_count){
    for (int i = 0; i < flash_count; i++) {
    GPIO_PORTF_DATA_R ^= 0b10000; // F4
    SysTick_Wait10ms(1);
    GPIO_PORTF_DATA_R ^= 0b10000;
    SysTick_Wait10ms(1);
    }
}

// spinning clockwise and measuring
void spin_cw(uint8_t dataReady, uint16_t Distance){
    int delay = 4000; // minimum delay between states for given clock
    int angle = 64; // 512/8 = 64
    int measure_point = 16; // 64/4 = 16 
		for (int j = 0; j < angle; j++){ 
      // 64 steps --> 45 deg (360 / 8): 4 state changes 45/4 = 11.25 deg
		  GPIO_PORTH_DATA_R = 0b00001001;
      SysTick_Wait(delay);
      GPIO_PORTH_DATA_R = 0b00000011;
      SysTick_Wait(delay);
      GPIO_PORTH_DATA_R = 0b00000110;
      SysTick_Wait(delay);
      GPIO_PORTH_DATA_R = 0b00001100;
      SysTick_Wait(delay);
      totalsteps++;
		}
    // stop button
    if((GPIO_PORTJ_DATA_R & 0b00000001) == 0){
      SysTick_Wait10ms(10);
				return;
    }
		if(totalsteps % measure_point == 0){ // 16 (64/4) increments is 11.25 deg (45/4)
      FlashLED(3);
      take_measurement(dataReady, Distance);
		}     
}

// spinning counter clockwise and measuring 
void spin_ccw(uint8_t dataReady, uint16_t Distance){
    int delay = 4000; // minimum delay between states for given clock
    int angle = 64; // 512/8 = 64
    int measure_point = 16; // 64/4 = 16 
		for (int j = 0; j < angle; j++){ 
      // 64 steps --> 45 deg (360 / 8): 4 state changes 45/4 = 11.25 deg
      GPIO_PORTH_DATA_R = 0b00001100;		
      SysTick_Wait(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
      SysTick_Wait(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
      SysTick_Wait(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
      SysTick_Wait(delay);
      totalsteps++;
		}
    // stop button PJ1 
    if((GPIO_PORTJ_DATA_R & 0b00000001) == 0){
      SysTick_Wait10ms(10);
				return;
    }
		if(totalsteps % measure_point == 0){ // 16 (64/4) increments is 11.25 deg (45/4)
      FlashLED(3);
      take_measurement(dataReady, Distance);
		}     
}

void take_measurement(uint8_t dataReady, uint16_t Distance){
  
  //wait until the ToF sensor's data is ready
  while (dataReady == 0){
    status = VL53L1X_CheckForDataReady(dev, &dataReady);
    FlashLED3(1);
    VL53L1_WaitMs(dev, 5);
  }
  dataReady = 0;

  // store received data
  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
  // Print the resulted readings to UART
  sprintf(printf_buffer,"%u\n",Distance);
  UART_printf(printf_buffer);
  SysTick_Wait10ms(50);
  }

int main(void) {
  // varaibles to store tof data 
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialisation 
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
  // init ports
  PortM0M1M2M3_Init();
  PortF0F4_Init();
  PortH_Init();
  PortJ_Init();
	PortJ_Interrupt_Init();	


  // 	status = VL53L1X_GetSensorId(dev, &wordData);
  // 	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
  // 	UART_printf(printf_buffer);


  // uncomment and connect probes to ad2 to check bus speed of system 
  //bus_pulse();
  // Booting ToF chip

  // getting sensor state 
  while(sensorState==0){
  status = VL53L1X_BootState(dev, &sensorState);
  SysTick_Wait10ms(10);
  }
  //FlashAllLEDs();
  //UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

  /* Initialize sensor in default mode - ranges at 10 Hz in Long distance mode */
  status = VL53L1X_SensorInit(dev);
  Status_Check("SensorInit", status);

    
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
  //  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  //  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
  //  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

  // main loop - ranging enabled and ready to measure 
	while(1){
		//wait for interupt from button to start
		WaitForInt();
		// Booting ToF chip
		while(sensorState==0){
			status = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
		}

		status = VL53L1X_ClearInterrupt(dev); 
		
		/* This function must to be called to initialize the sensor with the default setting  */
		status = VL53L1X_SensorInit(dev);
		Status_Check("SensorInit", status);
		status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

  
  for(int i = 0; i < 8; i++) { // 64*8 = 512
    // Wait until the ToF sensor's data is ready
    while (dataReady == 0){
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        VL53L1_WaitMs(dev, 5);
    }
    dataReady = 0;
    
    // Read the data values from ToF sensor
    status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
    status = VL53L1X_GetDistance(dev, &Distance); // The Measured Distance value
    status = VL53L1X_GetSignalRate(dev, &SignalRate);
    status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
    status = VL53L1X_GetSpadNb(dev, &SpadNum);
    
    status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
    
    // Print the resulted readings to UART
    sprintf(printf_buffer,"%u\r\n", Distance);
    UART_printf(printf_buffer);
    
    // Flash LED and delay before next measurement
    GPIO_PORTF_DATA_R = 0b00000001;
    SysTick_Wait10ms(30);
    GPIO_PORTF_DATA_R = 0b00000000;
    spin_cw();
    SysTick_Wait10ms(150);
}
// Spin motor in opposite direction to complete a full rotation
spin_ccw();
// Stop ranging after taking all measurements
VL53L1X_StopRanging(dev);

// removing temporarily
int temp = 0
if(temp){ 
// Loop through 8 times to spin the motor in opposite directions to take additional measurements
for(int i = 0; i < 8; i++) { // 360 / 32 = 11.25
    int dir = 0;
    if(!dir){
        // Check for peripheral button press to stop
        if((GPIO_PORTM_DATA_R&0b00000001)==0){
            SysTick_Wait10ms(10);
            break;
        }
        // Spin motor clockwise and take distance measurements
        spin_cw(dataReady, Distance);	
        totalsteps = 0;
    }else if (dir){
        // Check for peripheral button press to stop
        if((GPIO_PORTM_DATA_R&0b00000001)==0){
            SysTick_Wait10ms(10);
            break;
        }
        // Spin motor counterclockwise and take distance measurements
        spin_ccw(dataReady, Distance); 
        totalsteps = 0;
    }
    // toggle to switch direction again
    dir ^= 1;

    // Stop ranging after taking all measurements
    VL53L1X_StopRanging(dev);
    // Wait indefinitely
   // while(1) {}
  }
}
