// 2DX4 Final Project
//	
//  Name: Shiv Thakar 
//  Student No: 400247588
// 	MACID: thakas4
//
//	Assigned Clock-Speed and LED
//			Clock-Speed -> 30 MHz 
//			Onboard LED -> PN1 (LED D1) 


#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

// The VL53L1X uses a slightly different way to define the default address of 0x29
// The I2C protocol defintion states that a 7-bit address is used for the device
// The 7-bit address is stored in bit 7:1 of the address register.  Bit 0 is a binary
// value that indicates if a write or read is to occur.  The manufacturer lists the 
// default address as 0x52 (0101 0010).  This is 0x29 (010 1001) with the read/write bit
// alread set to 0.
//uint16_t	dev = 0x29;

//Global Variables Declared/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t	dev=0x52;

int status=0;
volatile int IntCount;

int loop = 0; //loop flag 
int motorComplete = 0; //motor flag
int pushButtonLoop = 0;


uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
uint16_t wordData;
uint8_t ToFSensor = 1; 																																// 0=Left, 1=Center(default), 2=Right
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady;

#define isInterrupt 1 

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[100];

#define I2C_MCS_ACK             0x00000008  																					// Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  																					// Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  																					// Acknowledge Address
#define I2C_MCS_STOP            0x00000004  																					// Generate STOP
#define I2C_MCS_START           0x00000002  																					// Generate START
#define I2C_MCS_ERROR           0x00000002																						// Error
#define I2C_MCS_RUN             0x00000001  																					// I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  																					// I2C Busy
#define I2C_MCR_MFE             0x00000010  																					// I2C Master Function Enable

#define MAXRETRIES              5           																					// number of receive attempts before giving up

//I2C Init Method///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_Init(void){
		SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           																	// activate I2C0
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          																// activate port B
 
		while((SYSCTL_PRGPIO_R&0x0002) == 0){};																						// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;          																						  // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																						// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																						// 5) enable digital I/O on PB2,3

																																											// 6) configure PB2,3 as I2C
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    								//TED
    I2C0_MCR_R = I2C_MCR_MFE;                      																		// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                  // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
        
		// 20*(TPR+1)*20ns = 10us, with TPR=24
    // TED 100 KHz
    //     CLK_PRD = 8.3ns
    //    TIMER_PRD = 1
    //    SCL_LP = 6
    //    SCL_HP = 4
    //    10us = 2 * (1 + TIMER_PRD) * (SCL_LP + SCL_HP) * CLK_PRD
    //    10us = 2 * (1+TIMER+PRD) * 10 * 8.3ns
    //  TIMER_PRD = 59 (0x3B)
    //
    // TIMER_PRD is a 6-bit value.  This 0-127
    //    @0: 2 * (1+ 0) * 10 * 8.3ns --> .1667us or 6.0MHz
    //  @127: 2 * (1+ 127) * 10 * 8.3ns --> 47kHz
    
    
}

//Port G Init Method for XSHUT//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port G
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//Port E Button Scanning////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PortE_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		  						// activate the clock for Port E
	
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	  			// allow time for clock to stabilize
	
	GPIO_PORTE_DEN_R= 0b00000001;															// enable PE0
	GPIO_PORTE_DIR_R |= 0b00000001;                           // make PE0 output  
	GPIO_PORTE_DATA_R=0b00000000;                             // setting state to zero to drive the row, one to disable 
	
	return;
}

//Port M Button Scanning////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // activate the clock for Port M
	
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // allow time for clock to stabilize 
			
	GPIO_PORTM_DIR_R |= 0b00000000;       			  						// make PM0 an input, PM0 is reading the column 
  GPIO_PORTM_DEN_R |= 0b00000001;														// enable PM0
	
	return;
}
//Port H Stepper Motor Initialization/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void stepperMotor_Init(void){
		//Use PortH pins for output
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;								// activate clock for Port H
	
		while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};				// allow time for clock to stabilize
			
		GPIO_PORTH_DIR_R |= 0xFF;        												
		GPIO_PORTH_AFSEL_R &= ~0xFF;     												
		GPIO_PORTH_DEN_R |= 0xFF;       															
		GPIO_PORTH_AMSEL_R &= ~0xFF;    
			
		return;
}

//VK53K1X_XSHUT/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

//motorToF Method///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ccwStepper(){

			for(int i=0; i<512;i++){
	
					GPIO_PORTH_DATA_R = 0b00001001;
					SystickMotorReverse(1);
					
					GPIO_PORTH_DATA_R = 0b00000011;
					SystickMotorReverse(1);
					
					GPIO_PORTH_DATA_R = 0b0000110;
					SystickMotorReverse(1);
					
					GPIO_PORTH_DATA_R = 0b00001100;
					SystickMotorReverse(1);
		
			}

}


void motorToF(void){
	
			int c = 512; //one full rotation is 512 repetitions for full step 

				//maxCount/8 * counter 
	
			for(int i=1; i<513; i++){
				
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait10ms(1);	//wait 10ms between steps
					
					GPIO_PORTH_DATA_R = 0b0000110;
					SysTick_Wait10ms(1);  //wait 10ms between steps
					
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait10ms(1);	//wait 10ms between steps
					
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait10ms(1);	//wait 10ms between steps
				
				
					 //32 Flashes, 11.25 degrees
					//edit the times to make more accurate
					if(i==c/32||i==2*c/32||i==3*c/32||i==4*c/32||i==5*c/32||i==6*c/32||i==7*c/32||i==8*c/32||i==9*c/32||i==10*c/32||i==11*c/32||i==12*c/32||i==13*c/32||i==14*c/32||i==15*c/32||i==16*c/32||i==17*c/32||i==18*c/32||i==19*c/32||i==20*c/32||i==21*c/32||i==22*c/32||i==23*c/32||i==24*c/32||i==25*c/32||i==26*c/32||i==27*c/32||i==28*c/32||i==29*c/32||i==30*c/32||i==31*c/32||i==c){
								
									//SysTick_Wait10ms(50);
					
					
									while (dataReady == 0){
											status = VL53L1X_CheckForDataReady(dev, &dataReady);
											VL53L1_WaitMs(dev, 5);
									}

									dataReady = 0;
									status = VL53L1X_GetDistance(dev, &Distance);						//get distance measurment from VL53L1X

									debugArray[i] = Distance;
			
									status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
									sprintf(printf_buffer,"%u\r\n", Distance); //send to python code 
									UART_printf(printf_buffer);
													
					}
				
			}
			
			return; 
	
}

//Button Scanning//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void buttonScan(void){
	
		while(1){//keep checking if the button is pressed 
 
					//Checks if Button 1 is pressed, if pressed D2 lights up	
				if(((GPIO_PORTM_DATA_R&0b00000001)==0&&(GPIO_PORTE_DATA_R&0b000000001)==0)){	//button pushed
							FlashLED1(1);
							//Motor has run at least once first
							if(motorComplete == 1){	//checks if motorComplete flag is True
										
										//if the button is pushed again after the motor
										//completes one rotation, it can be reset
									
										//resets the motor
										GPIO_PORTH_DATA_R = 0b00000000;		//resets GPIO PORT H 
								
							}
					
					break;	//break out of infinite loop, and return to main()
							
				}else{
						GPIO_PORTN_DATA_R=0b00000000;
				}
		}
		
		
		motorComplete = 1;
		return;
}

//Python Output//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pythonOutput(){
						UART_printf("Program Begins\r\n");
						sprintf(printf_buffer,"Shiv Thakar 2DX4 Final Project \r\n");
						UART_printf(printf_buffer);
							
					/* Those basic I2C read functions can be used to check your own I2C functions */
						status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
						myByteArray[i++] = byteData;

						status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
						myByteArray[i++] = byteData;
						
						status = VL53L1_RdWord(dev, 0x010F, &wordData);
						status = VL53L1X_GetSensorId(dev, &wordData);

						sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
						UART_printf(printf_buffer);

						// Booting ToF chip
						while(sensorState==0){
							status = VL53L1X_BootState(dev, &sensorState);
							SysTick_Wait10ms(10);
						}
				//		FlashAllLEDs();
						UART_printf("ToF Chip Booted!\r\n");
						UART_printf("One moment...\r\n");
						
						status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

						
						/* This function must to be called to initialize the sensor with the default setting  */
						status = VL53L1X_SensorInit(dev);
						Status_Check("SensorInit", status);

						status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
						Status_Check("StartRanging", status);
}

//Main Method///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

			loop = 1; 				//to enter loop
			int counter = 0;
	
			while(loop==1){		
						loop = 0; 
						motorComplete = 0;
				
						//Initialization 
						PLL_Init();	
						SysTick_Init();
						PortE_Init();
						PortM_Init();
						onboardLEDs_Init();
						I2C_Init();
						UART_Init();
						stepperMotor_Init();
			
						//Start Program
				
						if(counter == 0){
								buttonScan();			//scan for a button to start program
																	//this will only run once because a button pushed is asked at the end of loop 
																	//to reset
						}
										
						pythonOutput();					//call pythonOutput method

						motorToF();							//start motor
																	//take measurements every 11.25 degrees
										
						ccwStepper();
								
						if(counter < 9){				//allows program to take 10 measurements 
								buttonScan();				//asks for button input, and resets motor
								loop = 1; 					//loop flag is set to 1
								sprintf(printf_buffer,"%d\r\n",loop);	//this will allow the python code to loop within its module
								UART_printf(printf_buffer);
						}else{
								loop = 0;							//if 10 measurements have been taken, stop
								sprintf(printf_buffer,"%d\r\n",loop); //stop loop within python code
								UART_printf(printf_buffer);		
						}
						

						
						counter++;						//increment counter
												
			}
			
				VL53L1X_StopRanging(dev); 		//stop ranging
}

