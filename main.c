/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    CSE325_Project7.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
int red = 0;
int green = 0;
int blue = 0;
int button_flag = 0;
int flag;
unsigned short cal_v = 0;
unsigned char light_val = 0;
int color = 0;
int setColor = 100;
int skipColor = 100;
int lrFlag = 100;
/*
 * @brief   Application entry point.
 */

void adc(void){
	if(flag == 0){
    	ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
    	flag = 1;

	}else{
    	ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
    	flag = 0;
	}
	while(!(ADC0->SC1[0] & 0x80)){ }
	//delay(1000);
	light_val = ADC0->R[0]; // Resets COCO
	//printf("%d\n", light_val);
	if(light_val >= 240) {
		if(flag == 1){
			TPM2->CONTROLS[0].CnV = 0 ;
		}else{
			TPM2->CONTROLS[1].CnV = 0;
		}
	}
	else {
		if(button_flag == 1){
			if(flag == 1){
				TPM2->CONTROLS[0].CnV = 5000;
			}else{
				TPM2->CONTROLS[1].CnV = 5000;

			}
		}



	}
}

void turn_right(){
	GPIOB->PDOR |= (1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR |= (1<<2);
	for(int i=0;i<2500000;i++){
        __asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void turn_left(){
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR |= (1<<1);
	GPIOC->PDOR |= (1<<1);
	GPIOC->PDOR &= ~(1<<2);
	for(int i=0;i<2500000;i++){
        __asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void adjustLeft(){
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR |= (1<<1);
	GPIOC->PDOR |= (1<<1);
	GPIOC->PDOR &= ~(1<<2);
	for(int i=0;i<999999;i++){
		__asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}
void adjustLeft2(){
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR |= (1<<1);
	GPIOC->PDOR |= (1<<1);
	GPIOC->PDOR &= ~(1<<2);
	for(int i=0;i<1299998;i++){
		__asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void adjustRight2(){
	GPIOB->PDOR |= (1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR |= (1<<2);
	for(int i=0;i<1299998;i++){
		__asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void adjustRight(){
	GPIOB->PDOR |= (1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR |= (1<<2);
	for(int i=0;i<999999;i++){
		__asm volatile ("nop");
	}
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}

void stop(void){
	GPIOB->PDOR &= ~(1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<1);
	GPIOC->PDOR &= ~(1<<2);
}
void straight(void){
	TPM2->CONTROLS[0].CnV = 5000;
	TPM2->CONTROLS[1].CnV = 5000;
	GPIOB->PDOR |= (1<<0);
	GPIOB->PDOR &= ~(1<<1);
	GPIOC->PDOR |= (1<<1);
	GPIOC->PDOR &= ~(1<<2);
}
void delay_ms(unsigned short delay){
	SIM->SCGC6 |= (1<<24);//Enable Clock for TPM0
	SIM->SOPT2 |= (0x2 << 24); //Set TPMSRC to OSCERCLK
	TPM0->CONF |= (0x1 << 17); //Stop on Overflow
	TPM0->SC = (1<<7) | (0x07);//Reset Timer Overflow Flag, set prescaler 128
	TPM0->MOD = delay * 61 + delay/2;

	TPM0->SC |= 0x01 << 3; //Starts the clock

	while(!(TPM0->SC & 0x80)){
	}
}

void delay(unsigned short delay){
	for(int i=0;i<delay;i++){
		delay_ms(1000);
	}
}

void initI2C(){
    //Enable Clock Gating for I2C module and Port
	SIM->SCGC4 |= (1<<7) | (1<<6);
    SIM->SCGC5 |= (1<<11);

    //PTC9
    PORTC->PCR[9] &= ~0x700;
    PORTC->PCR[9] |= 0x200; //I2C SCL

    //PTC8
    PORTC->PCR[8] &= ~0x700;
    PORTC->PCR[8] |= 0x200; //I2C SDA

    //Write 0 to all I2C Registers
    I2C0->A1 = 0;
    I2C0->C1 = 0;
    I2C0->F = 0;
    I2C0->S = 0;
    I2C0->D = 0;
    I2C0->C2 = 0;
    I2C0->FLT = 0;
    I2C0->RA = 0;
    I2C0->SMB = 0;
    I2C0->A2 = 0;
    I2C0->SLTH = 0;
    I2C0->SLTL = 0;

    //Write 0x50 to FLT registers
    I2C0->FLT = 0x50;

    //Clear Status Flags
    I2C0->S |= (1<<1) | (1<<4);

    //Set I2C Divider Register
    //I2C Baudrate = bus speed / (mul * SCL Divider)
    I2C0->F |= (1<<7); //Set MULT register to 4;
    I2C0->F |= 0x1C;

    //Enable I2C module
    I2C0->C1 |= (1<<7);
}

void clearStatusFlags(){
	//Clear STOPF and Undocumented STARTF bit 4 in filter register
	I2C0->FLT |= (1<<6); //Clear STOPF
	I2C0->FLT &= ~(1<<4); //Clear bit 4

	//Clear ARBL and IICIF bits in Status Register
	I2C0->S |= (1<<1);//Clear Interrupt Flag
	I2C0->S |= (1<<4);//Clear ARBL bit
}

void TCFWait(){
	//Wait for TCF bit to set in Status Register
	while(!(I2C0->S & 0x80)){
		flag+=1;
	}

}

void IICIFWAIT(){
	//Wait for IICF bit to set in Status Register
	while(!(I2C0->S & 0x02)){
		//printf("IN WAIT LOOP\n");

	}
	//Reset Interupt Flag
//	I2C0->FLT |= (1<<6);
//	I2C0->S &= ~(1<<1);

}

void SendStart(){
	//Set MST and TX bits in Control 1 Register
	I2C0->C1 |= (1<<4) | (1<<5); //Sets Transmit mode to transmit and Master mode select to master mode
}

void RepeatStart(){
	I2C0->C1 |= (1<<4) | (1<<5) |(1<<2); //Sets Transmit mode to transmit, Master mode select to master mode and set RSTA
	//Todo: Wait 6 cycles
	volatile int cycles = 6;
	while(cycles > 0){
		cycles--;
	}
}

void SendStop(){
	//Clear MST, TX and TXAK bits in Control 1 Register
	I2C0->C1 &= ~(1<<4);
	I2C0->C1 &= ~(1<<5);
	I2C0->C1 &= ~(1<<2);

	//Wait for BUSY bit to go low in status registers
	while(I2C0->S & 0x20){}
}

void clearIICIF(){
	//Clear IICIF bit in status register
	I2C0->S |= (1<<1);
}

int RxAK(){
	if(!(I2C0->S & 0x01)){
		return 1;
	}
}

void I2C_WriteByte(int register_address, int Data){
	clearStatusFlags();
	TCFWait();
	SendStart();

	//printf("In I2C_WriteByte\n");

	//Write Device Address/ R/W = 0 to Data Register
	I2C0->D = ((0x29<<1) | (0x00));

	IICIFWAIT();
	//printf("Got through WAIT\n");
	if(!RxAK()){
		printf("No Response - Address: %d\n", 0x29);
		SendStop();
		return;
	}

	clearIICIF();

	//Todo: Write Register address to Data Register
	I2C0->D = register_address;

	IICIFWAIT();
	if(!RxAK()){
		printf("No Response - Register: %d\n", register_address);
		SendStop();
		return;
	}

	TCFWait();
	clearIICIF();

	//Todo: Write Data Byte to Data Register
	I2C0->D = Data;

	IICIFWAIT();
	if(!RxAK()){
		printf("Incorrect ACK - Data: %d\n",Data);
	}

	clearIICIF();
	SendStop();

	//printf("Leaving I2C_Write Byte\n");
}

void ReadBlock(int register_address, int destination_data_array[], unsigned int length){
	unsigned char dummy = 0;
	clearStatusFlags();
	TCFWait();
	SendStart();

	dummy++;

	//printf("In Read Block\n");

	//TODO: Write Device Address, R/W = 0 to Data Register
	I2C0->D = ((0x29<<1) | 0x00);

	IICIFWAIT();
	if(!RxAK()){
		printf("No Response - Address: %d\n", 0x29);
		SendStop();
		return;
	}

	clearIICIF();

	//Write Register Address to Data Register
	I2C0->D = register_address;

	IICIFWAIT();
	if(!RxAK()){
		printf("No Response - Register: %d\n", register_address);
		SendStop();
		return;
	}

	clearIICIF();
	RepeatStart();

	//Write Device address again, R/W = 1 to Data Register
	I2C0->D = ((0x29<<1) | (0x01));
	//I2C0->D |= (1<<0);

	IICIFWAIT();
	if(!RxAK()){
		//printf("No Response - Restart\n");
		SendStop();
		return;
	}

	TCFWait();
	clearIICIF();

	//Switch to RX by clearing TX and TXAK bits in Control 1 Register
	I2C0->C1 &= ~(1<<3); //TX
	I2C0->C1 &= ~(1<<4);//TXAK

	if(length==1){
		//Set TXAK to NACK in Control 1 - No more DATA!
		I2C0->C1 |= (1<<3);
	}

	dummy = I2C0->D;
	//printf("Dummy: %d\n",dummy);
	for(int index = 0; index<length; index++){
		IICIFWAIT();
		clearIICIF();

		if(index == length-2){
			I2C0->C1 |= (1<<3);
		}

		if(index == length-1){
			SendStop();
		}
		//Read Byte from Data Register into Array
		destination_data_array[index] = I2C0->D;
		//printf("%d\n",I2C0->D);
	}
}

//int checkBlue(){
//	if((red > 1000 && red <2300) && ((blue > 2000 && blue<3400)) && ((green > 3000 && green < 4400))){
//		return 1;
//	}else{
//		return 0;
//	}
//}

int checkBlue(){
	if((red > 1000 && red <5000) && ((blue > 2000 && blue<6000)) && ((green > 3000 && green < 6000))){
		//GPIOD->PDOR |= (1<<5);
		return 1;
	}else{
		//GPIOD->PDOR &= ~(1<<5);
		return 0;
	}
}
/*int checkGreen(){ Come back later if using cable
	if((red > 3300 && red <4500) && ((blue > 5000 && blue<6000)) && ((green > 2900 && green < 3700))){
		return 1;
	}else{
		return 0;
	}
}*/

int checkGreen(){
	if((red > 4000 && red <8000) && ((blue > 6000 && blue<9000)) && ((green > 2500 && green < 5000))){
		GPIOD->PDOR |= (1<<5);
		return 1;
	}else{
		GPIOD->PDOR &= ~(1<<5);
		return 0;
	}
}

//int checkYellow(){
//	if((red > 6000 && red <12000) && ((blue > 5000 && blue<9000)) && ((green > 2000 && green < 6000))){
//		return 1;
//	}else{
//		return 0;
//	}
//}

int checkYellow(){
	if((red > 8000 && red <12000) && ((blue > 7000 && blue<10000)) && ((green > 3000 && green < 6000))){
		//GPIOD->PDOR |= (1<<5);
		return 1;
	}else{
		//GPIOD->PDOR &= ~(1<<5);
		return 0;
	}
}
//int checkRed(){
//	if((red > 4000 && red <7000) && ((blue > 1100 && blue<2900)) && ((green > 1000 && green < 2450))){
//		GPIOD->PDOR |= (1<<5);
//		return 1;
//	}else{
//		GPIOD->PDOR &= ~(1<<5);
//		return 0;
//	}
//}
int checkRed(){
	if((red > 5000 && red <9000) && ((blue > 1100 && blue<4500)) && ((green > 1000 && green < 4000))){
		//GPIOD->PDOR |= (1<<5);
		return 1;
	}else{
		//GPIOD->PDOR &= ~(1<<5);
		return 0;
	}
}

void find_color(){
	if(checkRed()){
		color = 1;
		//printf("Red\n");
		return;
	}
	if(checkGreen()){
		color = 3;
		//printf("Green\n");
		return;
	}
	if(checkYellow()){
		color = 2;
		//printf("Yellow\n");
		return;
	}
//	if(checkGreen()){
//		color = 3;
//		//printf("Green\n");
//		return;
//	}
	if(checkBlue()){
		color = 4;
		//printf("Blue\n");
		return;
	}
	//printf("Black or white\n");
}


int main(void) {
	/* Init board hardware. */
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    int destination_arr[6] = {0};

    SIM->SCGC5 |= (1<<13) | (1<<9) | (1<<10) | (1<<11) | (1<<12);
   	SIM->SCGC6 |= (1<<27); //Enables ADC0
    //Motor stuff
    	//Left Motor Setup
	PORTB->PCR[0] &= ~0x700; //In1
	PORTB->PCR[0] |= 0x700 & (1<<8);

	PORTB->PCR[1] &= ~0x700;//In2
	PORTB->PCR[1] |= 0x700 & (1<<8);

	//Right Motor Setup
	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[1] |= 0x700 & (1<<8);

	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[2] |= 0x700 & (1<<8);

	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[2] |= 0x300;//Drive as PWM

	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[3] |= 0x300;//Drive as PWM

	GPIOB->PDDR |= (1<<0) | (1<<1);
	GPIOC->PDDR |= (1<<1)| (1<<2);
	GPIOC->PDDR &= ~(1<<3);
	GPIOC->PDDR &= ~(1<<12);

	//Setup for PWM on PTB2 (PWMA) Left Motor
	SIM->SCGC6 |= (1<<26);
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); // Toggle Output on Match
	//Setup for PWM on PTB3 (PWMB) Right Motor
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4); // Toggle Output on Match
	TPM2->MOD = 7999;
	TPM2->CONTROLS[0].CnV = 7000;
	TPM2->CONTROLS[1].CnV = 7000;

	TPM2->SC |= 0x01 << 3;

	//SW1
	PORTC->PCR[3] &= ~0x703;
	PORTC->PCR[3] |= 0x703 & ((1<<8) | 0x03); //Set MUX bits, enable pullups.

	//SW2
	PORTC->PCR[12] &= ~0x703;
	PORTC->PCR[12] |= 0x703 & ((1<<8) | 0x03); //Set MUX bits, enable pullups.


    //SETUP ADC0
    ADC0->CFG1 = 0; //Setup ADC clock
    ADC0->SC3 = 0x07; //Enables Maximum hardware averaging
    ADC0->SC3 |= 0x80; //Start calibration

    while(!(ADC0->SC1[0] & 0x80)){	}//Waits for calibration

    //Write calibration registers
    cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
    cal_v = cal_v >> 1 | 0x8000;
    ADC0->PG = cal_v;

    cal_v = 0;
    cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
    cal_v = cal_v >> 1 | 0x8000;
    ADC0->MG = cal_v;

    ADC0->SC3 = 0; // Turn off Hardware Averaging

    //Green led
   PORTD->PCR[5] &= ~0x700;
   PORTD->PCR[5] |= 0x700 & (1 <<8 );
   GPIOD->PDDR |= (1<<5);



   //RED led
   PORTE->PCR[29] &= ~ 0x700;
   PORTE->PCR[29] |= 0x700 & (1<<8);
   GPIOE->PDDR |= (1<<29);
    //Address for color sensor is 0x29;
    while(1) {
    	//printf("IN");
//    	straight();
    	TPM2->CONTROLS[0].CnV = 5000;
    	TPM2->CONTROLS[1].CnV = 5000;
        initI2C();
    	I2C_WriteByte(0x80,0x03);
    	ReadBlock(0xB6, destination_arr ,6); //Sets read address to 0x16 and sets Type to autoincrement
    	red = (destination_arr[1] << 8) | (destination_arr[0]);
    	blue = (destination_arr[3]<<8) | (destination_arr[2]);
    	green = (destination_arr[5]<<8) | (destination_arr[4]);
    	adc();
    	find_color();
    	//printf("Should go straight\n");
    	//straight();

    	//checks if robot has reached desired destination
    	if(color == setColor){
    		delay_ms(150);
    		if(lrFlag == 1){
    			adjustLeft();
    		}else{
    			adjustRight2();
    			adjustRight2();
    			straight();
    			delay(1);
    			delay_ms(100);
    		}
    		stop();
    		setColor = 100;
    	}
    	//Checks if robot has reached a color it has to skip over.
    	if(color == skipColor){
    		if(lrFlag == 1){
        		adjustLeft2();
    		}else{
    			adjustRight2();
    		}
    		straight();
    		skipColor = 100;
    	}
//    	if(green>3000){
//			printf("Turned off green\n");
//    		GPIOD->PDOR |= (1<<5);
//    	}else{
//    		GPIOD->PDOR &= ~(1<<5);
//    	}
//
//    	if(green<6000){
//    		printf("Turned off red\n");
//    		GPIOE->PDOR |= (1<<29);
//    	}else{
//    		GPIOE->PDOR &= ~(1<<5);
//    	}
    	//straight();
    	//printf("IN 2.0");
    	if(checkGreen()){
    		if(!(GPIOC->PDIR & 0x8)){ //SW1 Pressed(Outer Circle)
				lrFlag = 1;
    			button_flag = 1;
				straight();
				adc();
				setColor = 4;
				skipColor = 2;
    		}else if(!(GPIOC->PDIR & 0x1000)){
				lrFlag = 2;
				button_flag = 1;
				straight();
				adc();
				setColor = 4;
				skipColor = 1;
			}else{
				//printf("Button no work\n");
			}

        }

    	if(checkYellow()){
    		if(!(GPIOC->PDIR & 0x8)){ //SW1 Pressed (Outer Circle)
    			lrFlag = 1;
    			button_flag = 1;
    			straight();
    			adc();
    			setColor = 1;
    			skipColor = 4;

    		}else if(!(GPIOC->PDIR & 0x1000)){
    			lrFlag = 2;
    			button_flag = 1;
    			straight();
    			adc();
    			setColor = 1;
    			skipColor = 3;
    		}
    	}

    	if(checkRed()){
    		if(!(GPIOC->PDIR & 0x8)){ //SW1 Pressed (Outer Circle)
    			lrFlag = 1;
    			button_flag = 1;
    			straight();
    			adc();
    			setColor = 3;

    		}else if(!(GPIOC->PDIR & 0x1000)){
    			lrFlag = 2;
    			button_flag = 1;
    			straight();
    			adc();
    			setColor = 3;
    		}
    	}

    	if(checkBlue()){
    		if(!(GPIOC->PDIR & 0x8)){ //SW1 Pressed (Outer Circle)
    			lrFlag = 1;
    			button_flag = 1;
    			straight();
    			adc();
    			setColor = 1;

    		}else if(!(GPIOC->PDIR & 0x1000)){
    			lrFlag = 2;
    			button_flag = 1;
    			straight();
    			adc();
    			setColor = 1;
    		}
    	}


    	//printf("Red Low: %d Red High: %d\n",destination_arr[0],destination_arr[1]);
    	//printf("Blue Low: %d Blue High: %d\n",destination_arr[2],destination_arr[3]);
    	//printf("Green Low: %d Green High %d\n",destination_arr[4],destination_arr[5]);
    	//printf("HELLO");
    	printf("Red: %d\n", red);
    	printf("Blue: %d\n", blue);
    	printf("Green: %d\n", green);


    }
    return 0 ;
}
