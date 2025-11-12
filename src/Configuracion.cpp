#include <stm32f4xx.h>
#include "Configuracion.h"
#include <stdlib.h> // Est?ndar de librer?as 
#include <stdio.h> // Est?ndar de entradas y salidas
#include "math.h"


double Fac;
int ARR;
long APB;


// Definici�n de funciones apuntadoras a interrupci�n EXTI //
FunInt FunExti0=0,FunExti1=0,FunExti2=0,FunExti3=0,
FunExti4=0,FunExti5=0,FunExti6=0,FunExti7=0,FunExti8=0,
FunExti9=0,FunExti10=0,FunExti11=0,FunExti12=0,
FunExti13=0,FunExti14=0,FunExti15=0; 

 // Apuntadores a funciones de interrupci�n 
FunInt FunTim2=0,FunTim3=0,FunTim4=0,FunTim5=0,
FunTim6=0,FunTim7=0,FunTim9=0,FunTim10=0,
FunTim11=0,FunTim12=0,FunTim13=0,FunTim14=0;

FunInt FunUsart1=0, FunUsart2=0, FunUsart3=0, FunUsart4=0,
FunUsart5=0, FunUsart6=0, FunUsart7=0, FunUsart8=0;

// Vectores de Interrupciones //
extern "C"{
 // Vector interrupci�n EXTI0 y limpieza de bandera
 void EXTI0_IRQHandler(void){if(FunExti0!=0)FunExti0();EXTI->PR|=(1UL<<0);}
 // Vector interrupci�n EXTI1 y limpieza de bandera
 void EXTI1_IRQHandler(void){if(FunExti1!=0)FunExti1();EXTI->PR|=(1UL<<1);}
 // Vector interrupci�n EXTI2 y limpieza de bandera
 void EXTI2_IRQHandler(void){if(FunExti2!=0)FunExti2();EXTI->PR|=(1UL<<2);}
 // Vector interrupci�n EXTI3 y limpieza de bandera
 void EXTI3_IRQHandler(void){if(FunExti3!=0)FunExti3();EXTI->PR|=(1UL<<3);}
 // Vector interrupci�n EXTI4 y limpieza de bandera
 void EXTI4_IRQHandler(void){if(FunExti4!=0)FunExti4();EXTI->PR|=(1UL<<4);}
 // Vector interrupci�n EXTI 5 al 9 y limpieza de bandera
 void EXTI9_5_IRQHandler(void){
  // Se eval�a bandera activa, ejecuta rutina y limpia banderas
  if(EXTI->PR&(1UL<<5)){if(FunExti5!=0)FunExti5();EXTI->PR|=(1UL<<5);}
  if(EXTI->PR&(1UL<<6)){if(FunExti6!=0)FunExti6();EXTI->PR|=(1UL<<6);}
  if(EXTI->PR&(1UL<<7)){if(FunExti7!=0)FunExti7();EXTI->PR|=(1UL<<7);}
  if(EXTI->PR&(1UL<<8)){if(FunExti8!=0)FunExti8();EXTI->PR|=(1UL<<8);}
  if(EXTI->PR&(1UL<<9)){if(FunExti9!=0)FunExti9();EXTI->PR|=(1UL<<9);}
 }
 
 // Vector interrupci�n EXTI 10 al 15 y limpieza de bandera
 void EXTI15_10_IRQHandler(void){
  // Se eval�a bandera activa, ejecuta rutina y limpia banderas
  if(EXTI->PR&(1UL<<10)){if(FunExti10!=0)FunExti10();EXTI->PR|=(1UL<<10);}
  if(EXTI->PR&(1UL<<11)){if(FunExti11!=0)FunExti11();EXTI->PR|=(1UL<<11);}
  if(EXTI->PR&(1UL<<12)){if(FunExti12!=0)FunExti12();EXTI->PR|=(1UL<<12);}
  if(EXTI->PR&(1UL<<13)){if(FunExti13!=0)FunExti13();EXTI->PR|=(1UL<<13);}
  if(EXTI->PR&(1UL<<14)){if(FunExti14!=0)FunExti14();EXTI->PR|=(1UL<<14);}
  if(EXTI->PR&(1UL<<15)){if(FunExti15!=0)FunExti15();EXTI->PR|=(1UL<<15);}
 }
   // Vectores de interrupci�n, limpieza de banderas y ejecuci�n de rutina
 void TIM2_IRQHandler(void){TIM2->SR&=(~TIM_SR_UIF);if(FunTim2!=0)FunTim2();}
 void TIM3_IRQHandler(void){TIM3->SR&=(~TIM_SR_UIF);if(FunTim3!=0)FunTim3();}
 void TIM4_IRQHandler(void){TIM4->SR&=(~TIM_SR_UIF);if(FunTim4!=0)FunTim4();}
 void TIM5_IRQHandler(void){TIM5->SR&=(~TIM_SR_UIF);if(FunTim5!=0)FunTim5();}
 void TIM1_BRK_TIM9_IRQHandler(void){
  TIM9->SR&=(~TIM_SR_UIF);if(FunTim9!=0)FunTim9();}
 void TIM1_UP_TIM10_IRQHandler(void){
  TIM10->SR&=(~TIM_SR_UIF);if(FunTim10!=0)FunTim10();}
 void TIM1_TRG_COM_TIM11_IRQHandler(void){
  TIM11->SR&=(~TIM_SR_UIF);if(FunTim11!=0)FunTim11();}

    // Vector interrupci�n USART1
 void USART1_IRQHandler(void){
  if(!(USART1->SR&USART_SR_RXNE))return;
   // Se ejecuta interrupci�n
  if(FunUsart1!=0)FunUsart1();
  (void)USART1->SR; // Se limpian banderas
  (void)USART1->DR;
 }
 
 // Vector interrupci�n USART2
 void USART2_IRQHandler(void){
  if(!(USART2->SR&USART_SR_RXNE))return;
   // Se ejecuta interrupci�n
  if(FunUsart2!=0)FunUsart2();
  (void)USART2->SR; // Se limpian banderas
  (void)USART2->DR;
 }
 
  // Vector interrupci�n USART3
 void USART6_IRQHandler(void){
  if(!(USART6->SR&USART_SR_RXNE))return;
     // Se ejecuta interrupci�n   
  if(FunUsart6!=0)FunUsart6();
  (void)USART6->SR; // Se limpian banderas
  (void)USART6->DR;
 }
}

void Pines::ModoPin(char Pin,int Modo){
	int M=0,N=0,Corre=0;
	int pin=Pin&0x0F; // Numero de pin
	 switch(Pin&0xF0){
     case 0x00:Port=GPIOA;RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;break;
     case 0x10:Port=GPIOB;RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;break;
     case 0x20:Port=GPIOC;RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;break;
	 }
	Corre=pin+pin;
	if(Modo==0){M=0;N=0;}else if(Modo==1){M=0;N=1;}else if(Modo==2){M=1;N=0;}else if(Modo==3){M=1;N=1;} // 0 Input, 1 Output, 2 Alternate
	Port ->MODER |= (M<<(Corre+1))|(N<<Corre);
	 mask1=(1UL<<pin); // Asignaci�n de la m�scara 1
     mask0=(1UL<<(pin+16)); // Asignaci�n de la m�scara 0
}
int Pines::EntradaPin(char Pin, int Lectura){
	int Corra=0,I=0;
	int pin=Pin&0x0F; // Numero de pin
    mask1=(1UL<<pin); // Asignaci�n de la m�scara 1
	Corra=pin+pin;
     switch(Pin&0xF0){
     case 0x00:Port=GPIOA;;break;
     case 0x10:Port=GPIOB;;break;
     case 0x20:Port=GPIOC;;break;
	 }
	if(Lectura==0){Port->PUPDR |= (0<<(Corra+1))|(1<<Corra);} else if(Lectura==1){Port->PUPDR |= (1<<(Corra+1))|(0<<Corra);}
    if(Port->IDR&mask1){return I=1;}
    return I=0;
    }
void Pines::SalidaPin(char Pin, int Salida){
	int pin=Pin&0x0F; // Numero de pin
	mask1=(1UL<<pin); // Asignaci�n de la m�scara 1
    mask0=(1UL<<(pin+16)); // Asignaci�n de la m�scara 0
	switch(Pin&0xF0){
     case 0x00:Port=GPIOA;;break;
     case 0x10:Port=GPIOB;;break;
     case 0x20:Port=GPIOC;;break;
	 }
    if(Salida)Port->BSRR|=mask1;
    else Port->BSRR|=mask0;
}
void Pines::Interrupcion(FunInt Funcion,char Pin, int Lectura){
	//void EXTI#_IRQHandler(void){} Interrupcion
	int Corra=0;
	int pin=Pin&0x0F; // Numero de pin
	unsigned char prt=(pin>>4)&0x0F; // Numero de puerto
	Corra=pin+pin;
	ModoPin(Pin,0);
	RCC->APB2ENR |=RCC_APB2ENR_SYSCFGEN;
	 EXTI->RTSR&=~mask1; // Apaga flanco de subida
     EXTI->FTSR&=~mask1; // Apaga flanco de bajada
     EXTI->IMR&=~mask1; // apaga interrupci�n
	 EXTI->IMR |= (1<<pin);
	if(Lectura==0){
	EXTI->FTSR |= (1<<pin);
	Port->PUPDR |= (0<<(Corra+1))|(1<<Corra);
	}else if(Lectura==1){
	EXTI->RTSR |= (1<<pin);
	Port->PUPDR |= (1<<(Corra+1))|(0<<Corra);
	}else if(Lectura==2){
	EXTI->RTSR |= (1<<pin);
	EXTI->FTSR |= (1<<pin);}
   switch(pin&=0x0F){ // Eval�a pin fuente
  case 0x00:SYSCFG->EXTICR[0]&=0xFFF0; // Borra puerto
   SYSCFG->EXTICR[0]|=(prt<<0); // Asigna puerto 
   FunExti0=Funcion; // Asigna vector de interrupci�n 
   NVIC_EnableIRQ(EXTI0_IRQn); // Activa interrupci�n
   break;
  case 0x01:SYSCFG->EXTICR[0]&=0xFF0F; // Borra puerto
   SYSCFG->EXTICR[0]|=(prt<<4); // Asigna puerto 
   FunExti1=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI1_IRQn); // Activa interrupci�n
   break;
  case 0x02:SYSCFG->EXTICR[0]&=0xF0FF; // Borra puerto
   SYSCFG->EXTICR[0]|=(prt<<8); // Asigna puerto
   FunExti2=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI2_IRQn); // Activa interrupci�n
   break;
  case 0x03:SYSCFG->EXTICR[0]&=0x0FFF; // Borra puerto
   SYSCFG->EXTICR[0]|=(prt<<12); // Asigna puerto
   FunExti3=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI3_IRQn); // Activa interrupci�n
   break;
  case 0x04:SYSCFG->EXTICR[1]&=0xFFF0; // Borra puerto
   SYSCFG->EXTICR[1]|=(prt<<0); // Asigna puerto
   FunExti4=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI4_IRQn); // Activa interrupci�n
   break;
  case 0x05:SYSCFG->EXTICR[1]&=0xFF0F; // Borra puerto
   SYSCFG->EXTICR[1]|=(prt<<4); // Asigna puerto
   FunExti5=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI9_5_IRQn); // Activa interrupci�n
   break;
  case 0x06:SYSCFG->EXTICR[1]&=0xF0FF; // Borra puerto
   SYSCFG->EXTICR[1]|=(prt<<8); // Asigna puerto
   FunExti6=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI9_5_IRQn); // Activa interrupci�n
   break;
  case 0x07:SYSCFG->EXTICR[1]&=0x0FFF; // Borra puerto
   SYSCFG->EXTICR[1]|=(prt<<12); // Asigna puerto
   FunExti7=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI9_5_IRQn); // Activa interrupci�n
   break;
  case 0x08:SYSCFG->EXTICR[2]&=0xFFF0; // Borra puerto
   SYSCFG->EXTICR[2]|=(prt<<0); // Asigna puerto
   FunExti8=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI9_5_IRQn); // Activa interrupci�n
   break;
  case 0x09:SYSCFG->EXTICR[2]&=0xFF0F; // Borra puerto
   SYSCFG->EXTICR[2]|=(prt<<4); // Asigna puerto
   FunExti9=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI9_5_IRQn); // Activa interrupci�n
   break;
  case 0x0A:SYSCFG->EXTICR[2]&=0xF0FF; // Borra puerto
   SYSCFG->EXTICR[2]|=(prt<<8); // Asigna puerto
   FunExti10=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Activa interrupci�n
   break;
  case 0x0B:SYSCFG->EXTICR[2]&=0x0FFF; // Borra puerto
   SYSCFG->EXTICR[2]|=(prt<<12); // Asigna puerto
   FunExti11=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Activa interrupci�n
   break;
  case 0x0C:SYSCFG->EXTICR[3]&=0xFFF0; // Borra puerto
   SYSCFG->EXTICR[3]|=(prt<<0); // Asigna puerto
   FunExti12=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Activa interrupci�n
   break;
  case 0x0D:SYSCFG->EXTICR[3]&=0xFF0F; // Borra puerto
   SYSCFG->EXTICR[3]|=(prt<<4); // Asigna puerto 
   FunExti13=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Activa interrupci�n  
   break;
  case 0x0E:SYSCFG->EXTICR[3]&=0xF0FF; // Borra puerto
   SYSCFG->EXTICR[3]|=(prt<<8); // Asigna puerto 
   FunExti14=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Activa interrupci�n
   break;
  case 0x0F:SYSCFG->EXTICR[3]&=0x0FFF; // Borra puerto
   SYSCFG->EXTICR[3]|=(prt<<12); // Asigna puerto
   FunExti15=Funcion; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Activa interrupci�n
   break; 
 }
}

void Analogo::Conversion(char Pin){
	int Canal=0;
    int pin=Pin&0x0F; // Numero de pin
	RCC -> APB2ENR |= 0X100; //HABILITAR RELOJ DEL ADC 1
	ADC1 -> CR1 |= 0X0; //RESOLUCION DE 12 BITS (POR DEFECTO)
	ADC1 -> CR2 |= 0X1; //ENCENDER EL ADC
    ModoPin(Pin,3);
     switch(Pin&0xF0){
     case 0x00:  ADC1 -> SQR3 |= pin;break;
     case 0x10:  if(pin==0){ADC1 -> SQR3 = 8;}break;
     case 0x20:  if(pin<=5){Canal=10+pin; ADC1 -> SQR3 |= Canal;}break; //CANAL 0 DEL ADC PIN PA0
	 }	
}
void Analogo :: IniciarADC(){
	ADC1 -> CR2 |= (1UL<<30); //0X40000000; //INICIAR LA CONVERSION EN CANALES REGULARES
	while((ADC1->SR & 0X2)==1);   //Espera por el fin de conversi�n
 }

void USART::Comunicacion(int USART,char Puerto,int Velocidad){
	//void USART1_IRQHandler(void){               while(!(USART#->SR&USART_SR_RXNE));}  Interrupcion
	RCC -> AHB1ENR |= 0XF;
	int BRR=0;
	unsigned long PC=0;
	if(USART==1){
	USA=USART1;
	PC=Re.APB2();
	 RCC->APB2ENR |= (1UL<<4); // Enable clock for USART1
   if(Puerto=='A'){
		 GPIOA->MODER |=0X280000; // USART PA9TX Y PA10RX
		 GPIOA->AFR[1] |= 0x770;
	 }else if(Puerto=='B'){
		 GPIOB->MODER |=0XA000; // USART PB6TX Y PB7RX
		 GPIOB->AFR[0] |= 0x77000000;
	 }
	}else if(USART==6){
	 USA=USART6;
	 RCC->APB2ENR |= (1UL<<5); // Enable clock for USART6
	 PC=Re.APB2();
		 if(Puerto=='A'){
		 GPIOA->MODER |=0X2800000; // USART PA11TX Y PA12RX
		 GPIOA->AFR[1] |= 0x88000;
	 }else if(Puerto=='C'){
		 GPIOC->MODER =0XA000; // USART PC6TX Y PC7RX
		 GPIOC->AFR[0] |= 0x88000000;
	 }
	}else if(USART==2){
		USA=USART2;
		PC=Re.PCLK1();
		RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
		GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
		GPIOA->AFR[0] |= (7UL<<GPIO_AFRL_AFSEL2_Pos);
		GPIOA->AFR[0] |= (7UL<<GPIO_AFRL_AFSEL3_Pos);
	}
	BRR=PC/Velocidad;
	USA->BRR |=BRR;
	USA->CR1 |= (USART_CR1_RE|USART_CR1_TE);
	USA->CR1 |= USART_CR1_UE;
}
void USART::TransmitirDato(unsigned char d){
	 while(!(USA->SR&USART_SR_TXE)){}; // Se espera buffer disponible
   USA->DR=d; // Se transmite dato 
}
void USART::TransmitirDatos(char *d){
	 int n=0;
  // Transmite datos hasta car�cter nulo  
  while(d[n]!=0){TransmitirDato(d[n++]);};  
}
void USART ::Interrupcion(FunInt fun,int usa){// M�todo para asignar interrupci�n de Rx
 switch(usa){ // Eval�a puerto
  case 1:FunUsart1=fun; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(USART1_IRQn); // Activa interrupci�n
   USA=USART1;
  break;
  case 2:FunUsart2=fun; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(USART2_IRQn); // Activa interrupci�n
   USA=USART2;
  break;
  case 6:FunUsart6=fun; // Asigna vector de interrupci�n
   NVIC_EnableIRQ(USART6_IRQn); // Activa interrupci�n
   USA=USART6;
  break;
 }
 // Activa interrupci�n en puerto
 USA->CR1|=USART_CR1_RXNEIE;
}

void Timers::PWM(int Timerr,char Puerto,int CH1,int CH2, int CH3,int CH4,double Periodo){
	RCC -> AHB1ENR |= 0XF;
	if(Timerr==1){
		Timer(1,Periodo);
		TIM1->EGR |= 1;
		if(CH1==1){TIM1->CCMR1 |= 0x78;TIM1->CCER|= 0x1;GPIOA->MODER |= 0X20000;GPIOA->AFR[1] |= 0X1;}
		if(CH2==1){TIM1->CCMR1 |= 0x7800;TIM1->CCER|= 0x10;GPIOA->MODER |= 0X80000;GPIOA->AFR[1] |= 0X10;}
		if(CH3==1){TIM1->CCMR2 |= 0x78;TIM1->CCER|= 0x100;GPIOA->MODER |= 0X200000;GPIOA->AFR[1] |= 0X100;}
		if(CH4==1){TIM1->CCMR2 |= 0x7800;TIM1->CCER|= 0x1000;GPIOA->MODER |= 0X800000;GPIOA->AFR[1] |= 0X1000;}
		TIM1->BDTR=0X8000;
	}else if(Timerr==2){
		Timer(2,Periodo);
		TIM2->EGR |= 1;
		if(CH1==1){TIM2->CCMR1 |= 0x60;TIM2->CCER|= 0x1;GPIOA->MODER |= 0X2;GPIOA->AFR[0] |= 0X1;TIM2->CCER^=TIM_CCER_CC1P;}
		if(CH2==1){TIM2->CCMR1 |= 0x6000;TIM2->CCER|= 0x10;GPIOA->MODER |= 0X8;GPIOA->AFR[0] |= 0X10;TIM2->CCER^=TIM_CCER_CC2P;}
		if(CH3==1){TIM2->CCMR2 |= 0x60;TIM2->CCER|= 0x100;GPIOA->MODER |= 0X20;GPIOA->AFR[0] |= 0X100;TIM2->CCER^=TIM_CCER_CC3P;}
		if(CH4==1){TIM2->CCMR2 |= 0x6000;TIM2->CCER|= 0x1000;GPIOA->MODER |= 0X80;GPIOA->AFR[0] |= 0X1000;TIM2->CCER^=TIM_CCER_CC4P;}	
	}else if(Timerr==3){
	  Timer(3,Periodo);
		TIM3->EGR |= 1;
		if(CH1==1){TIM3->CCMR1 |= 0x78;TIM3->CCER |= 0x1;
		if(Puerto=='A'){GPIOA->MODER |= 0X2000;GPIOA->AFR[0] |= 0X2000000;}else if(Puerto=='B'){GPIOB->MODER |= 0X200;GPIOB->AFR[0] |= 0X20000;}else if(Puerto=='C'){GPIOC->MODER |= 0X2000;GPIOC->AFR[0] |= 0X2000000;}
		}
		if(CH2==1){TIM3->CCMR1 |= 0x7800;TIM3->CCER|= 0x10;
		if(Puerto=='A'){GPIOA->MODER |= 0X8000;GPIOA->AFR[0] |= 0X20000000;}else if(Puerto=='B'){GPIOB->MODER |= 0X800;GPIOB->AFR[0] |= 0X200000;}else if(Puerto=='C'){GPIOC->MODER |= 0X8000;GPIOC->AFR[0] |= 0X20000000;}
		}
		if(CH3==1){TIM3->CCMR2 |= 0x78;TIM3->CCER|= 0x100;
		if(Puerto=='B'){GPIOB->MODER |= 0X2;GPIOB->AFR[0] |= 0X2;}else if(Puerto=='C'){GPIOC->MODER |= 0X20000;GPIOC->AFR[1] |= 0X2;}
		}
		if(CH4==1){TIM3->CCMR2 |= 0x7800;TIM3->CCER|= 0x1000;
		if(Puerto=='B'){GPIOB->MODER |= 0X8;GPIOB->AFR[0] |= 0X20;}else if(Puerto=='C'){GPIOC->MODER |= 0X80000;GPIOC->AFR[1] |= 0X20;}
		}	
	}else if(Timerr==4){
		Timer(4,Periodo);
		TIM4->EGR |= 1;         // Up Counter
		if(CH1==1){TIM4->CCMR1 |= 0x78;TIM4->CCER|= 0x1;GPIOB->MODER |= 0X2000;GPIOB->AFR[0] |= 0X2000000;}
		if(CH2==1){TIM4->CCMR1 |= 0x7800;TIM4->CCER|= 0x10;GPIOB->MODER |= 0X8000;GPIOB->AFR[0] |= 0X20000000;}
		if(CH3==1){TIM4->CCMR2 |= 0x78;TIM4->CCER|= 0x100;GPIOB->MODER |= 0X20000;GPIOB->AFR[1] |= 0X2;}
		if(CH4==1){TIM4->CCMR2 |= 0x7800;TIM4->CCER|= 0x1000;GPIOB->MODER |= 0X80000;GPIOB->AFR[1] |= 0X20;}	
	}else if(Timerr==5){
		Timer(5,Periodo);
		TIM5->EGR |= 1;         // Up Counter
		if(CH1==1){TIM5->CCMR1 |= 0x78;TIM5->CCER|= 0x1;GPIOA->MODER |= 0X2;GPIOA->AFR[0] |= 0X2;}
		if(CH2==1){TIM5->CCMR1 |= 0x7800;TIM5->CCER|= 0x10;GPIOA->MODER |= 0X8;GPIOA->AFR[0] |= 0X20;}
		if(CH3==1){TIM5->CCMR2 |= 0x78;TIM5->CCER|= 0x100;GPIOA->MODER |= 0X20;GPIOA->AFR[0] |= 0X200;}
		if(CH4==1){TIM5->CCMR2 |= 0x7800;TIM5->CCER|= 0x1000;GPIOA->MODER |= 0X80;GPIOA->AFR[0] |= 0X2000;}	
	}else if(Timerr==9){
	  Timer(9,Periodo);
		TIM9->EGR |= 1;                // Up Counter
		if(CH1==1){TIM9->CCMR1 |= 0x78;TIM9->CCER = 0x1;  GPIOA->MODER |= 0X20;GPIOA->AFR[0] |= 0X300;}
		if(CH2==1){TIM9->CCMR1 |= 0x7800;TIM9->CCER|= 0x10;GPIOA->MODER |= 0X80;GPIOA->AFR[0] |= 0X3000;}
		
	}
	Tim->CR1|=TIM_CR1_ARPE; // Recarga activa
    Tim->CR1|=TIM_CR1_CEN; // Activo modulo comparador
    Tim->BDTR|=TIM_BDTR_MOE; // Outputs activos
}
void Timers::CicloUtil(int Timer,int CH,double Ciclo){
	unsigned int CCR; // Variable de calculo
	if(Timer==1){Tim=TIM1;}else if(Timer==2){Tim=TIM2;}
  else if(Timer==3){Tim=TIM3;}else if(Timer==4){Tim=TIM4;}
  else if(Timer==5){Tim=TIM5;}else if(Timer==9){Tim=TIM9;}
	CCR = Tim->ARR*(100.0-Ciclo)/100.0;
	if     (CH==1){Tim->CCR1=CCR;}
	else if(CH==2){Tim->CCR2=CCR;}
	else if(CH==3){Tim->CCR3=CCR;}
	else if(CH==4){Tim->CCR4=CCR;}
}
void Timers::Timer(int Time,double Periodo){
	unsigned int PSC_ARR;
	switch(Time){
  case 1:Tim=TIM1; // Asigna estructura TIM1
   RCC->APB2ENR|=RCC_APB2ENR_TIM1EN; // Activa reloj TIM
   APB=APB2(); // Lee fuente APB
  break;
  case 2:Tim=TIM2; // Asigna estructura TIM2
   RCC->APB1ENR|=RCC_APB1ENR_TIM2EN; // Activa reloj TIM
   APB=APB1(); // Lee fuente APB
  break;
  case 3:Tim=TIM3; // Asigna estructura TIM3
   RCC->APB1ENR|=RCC_APB1ENR_TIM3EN; // Activa reloj TIM
   APB=APB1(); // Lee fuente APB
  break;
  case 4:Tim=TIM4; // Asigna estructura TIM4
   RCC->APB1ENR|=RCC_APB1ENR_TIM4EN; // Activa reloj TIM
   APB=APB1(); // Lee fuente APB
  break;
  case 5:Tim=TIM5; // Asigna estructura TIM5
   RCC->APB1ENR|=RCC_APB1ENR_TIM5EN; // Activa reloj TIM
   APB=APB1(); // Lee fuente APB
  break;
  case 9:Tim=TIM9; // Asigna estructura TIM9
   RCC->APB2ENR|=RCC_APB2ENR_TIM9EN; // Activa reloj TIM
   APB=APB2(); // Lee fuente APB
  break;
  case 10:Tim=TIM10; // Asigna estructura TIM10
   RCC->APB2ENR|=RCC_APB2ENR_TIM10EN; // Activa reloj TIM
   APB=APB2(); // Lee fuente APB
  break;
  case 11:Tim=TIM11; // Asigna estructura TIM11
   RCC->APB2ENR|=RCC_APB2ENR_TIM11EN; // Activa reloj TIM
   APB=APB2(); // Lee fuente APB
  break;
 }
 Tim->CNT=0xFFFFFFFF; // Valor inicial del contador
 Tim->CR1|=TIM_CR1_CEN; // Encendido del TIM
 PSC_ARR=sqrtf(Periodo*APB)-1;
 Tim->ARR=PSC_ARR;
 Tim->PSC=PSC_ARR;
 Tim->CNT=0xFFFFFFFF;
}
void Timers::Interrupcion(int Time,FunInt Funcion){
 switch(Time){ // Eval�a n�mero del TIM
  case 2:FunTim2=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM2_IRQn); // Activa interrupci�n
  break;
  case 3:FunTim3=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM3_IRQn); // Activa interrupci�n
  break;
  case 4:FunTim4=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM4_IRQn); // Activa interrupci�n
  break;
  case 5:FunTim5=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM5_IRQn); // Activa interrupci�n
  break;
  case 9:FunTim9=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn); // Activa interrupci�n
  break;
  case 10:FunTim10=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // Activa interrupci�n
  break;
  case 11:FunTim11=Funcion; // Asigna apuntador de funci�n
   NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn); // Activa interrupci�n
  break;
  default: return;
 }
  if(Time==1){Tim=TIM1;}else if(Time==2){Tim=TIM2;}
  else if(Time==3){Tim=TIM3;}else if(Time==4){Tim=TIM4;}
  else if(Time==5){Tim=TIM5;}else if(Time==9){Tim=TIM9;}
 Tim->DIER|=TIM_DIER_UIE; // Enciende interrupci�n TIM
}
double Timers::LeerPeriodo(void){
	return (Tim->PSC+1)*(Tim->ARR+1)/(double)APB;
}
void Timers::CapturarPulso(int Time,double Periodo,char Puerto,int Pin){
    Timer(Time,Periodo); // Asigna Timer
    Fac=LeerPeriodo()/((double)(Tim->ARR)); // Calcula el factor de tiempo
    Tim->CCMR1|=TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_1; // Activa entrada TI1 con CCR1 y CCR2
    Tim->SMCR|=TIM_SMCR_TS_2|TIM_SMCR_TS_0|TIM_SMCR_SMS_2; 
    Tim->CCER|=TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC2P;// Activa m�dulo de captura y sus flancos
    Tim->CNT=0xFFFFFFFF;; // Valor inicial del contador
}
void Timers::IniciarEncoder(int Tmr,char CanalA,char CanalB){
    int Corra1=0,Corra2=0;
    int pinA=CanalA&0x0F; // Numero de pin
    int pinB=CanalB&0x0F; // Numero de pin
	Corra1=pinA+pinA;
	Corra2=pinB+pinB;
	P.ModoPin(CanalA,2);
    switch(CanalA&0xF0){
     case 0x00:Puerto=GPIOA;;break;
     case 0x10:Puerto=GPIOB;;break;
     case 0x20:Puerto=GPIOC;;break;
	 }
    Puerto->PUPDR |= (0<<(Corra1+1))|(1<<Corra1);
	if(pinA<=7){Puerto->AFR[0]|=0X1<<pinA*4;}else{Puerto->AFR[1]|=0X1<<(pinA-8)*4;}
    P.ModoPin(CanalB,2);
    switch(CanalB&0xF0){
     case 0x00:Puerto=GPIOA;;break;
     case 0x10:Puerto=GPIOB;;break;
     case 0x20:Puerto=GPIOC;;break;
	 }
    Puerto->PUPDR |= (0<<(Corra2+1))|(1<<Corra2);
	if(pinB<=7){Puerto->AFR[0]|=0X1<<pinB*4;}else{Puerto->AFR[1]|=0X1<<(pinB-8)*4;}
   switch(Tmr){
   case 1:Tim=TIM1; // Asigna estructura TIM1
   RCC->APB2ENR|=RCC_APB2ENR_TIM1EN; // Activa reloj TIM
   APB=APB2(); // Lee fuente APB
   break;
   case 2:Tim=TIM2; // Asigna estructura TIM2
   RCC->APB1ENR|=RCC_APB1ENR_TIM2EN; // Activa reloj TIM
   APB=APB1(); // Lee fuente APB
   break;
 }
    Tim->CR1|=TIM_CR1_CEN; // Encendido del TIM
	Tim->ARR=0xFFFF; // Asigna m�ximo conteo al Timer
    // Configura Modo Encoder
    Tim->CCMR1|=TIM_CCMR1_CC1S_0;
    Tim->CCMR1|=TIM_CCMR1_CC2S_0;
    // Activa entradas canal 1 y 2
    Tim->CCER&=~(TIM_CCER_CC2P|TIM_CCER_CC1P);
    Tim->SMCR|=3UL;
    Tim->CNT=32768; // Inicia Timer en valor central
}
int  Timers ::GetDiferencial(int Tmr){
	int dif;
	switch(Tmr){
  case 1:Tim=TIM1; // Asigna estructura TIM1
  break;
  case 2:Tim=TIM2; // Asigna estructura TIM2
  break;
  case 3:Tim=TIM3; // Asigna estructura TIM3
  break;
  case 4:Tim=TIM4; // Asigna estructura TIM4
  break;
  case 5:Tim=TIM5; // Asigna estructura TIM5
  break;
  case 9:Tim=TIM9; // Asigna estructura TIM9
  break;
  case 10:Tim=TIM10; // Asigna estructura TIM10
  break;
  case 11:Tim=TIM11; // Asigna estructura TIM11
  break;
 }
  // Calcula la diferencia
  dif=32768-(int)Tim->CNT;
  Tim->CNT=32768; // Inicia Timer en valor central
  return dif; // Retorna valor
}
int  Timers ::GetPosicion(int Tmr){
	int PosEncoder=0;
	  // Incrementa posici�n del Encoder
  // con la diferencial actual
    PosEncoder+=GetDiferencial(Tmr);
 return PosEncoder; // Retorna valor de posici�n
}

void i2c :: IDOSC(int I2C,int Velocidad){
	  unsigned int ccr,tr,cc2;
      if(I2C==1){ //PB8 SCL Y PB9 SDA
		I2=I2C1;
	   RCC->APB1ENR|=RCC_APB1ENR_I2C1EN;
	   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 
	   GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	   GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1); // Configura como AF
    // Configura PB8 y PB9 como Open Drain
       GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    // Configura la velocidad alta
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
    // Activa las resistencias Pull-up
       GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);
    // Configura el Alternate Function para I2C1
       GPIOB->AFR[1] |= (4 << (4 * (8 - 8))) | (4 << (4 * (9 - 8)));
	 }else if(I2C==2){
		I2=I2C2;
	  }else if(I2C==3){
		I2=I2C3;
		// No se necesita configuración adicional aquí
	  } 
 //I2->CR1|=I2C_CR1_PE; // Activaci�n del I2C
 //I2->CR1|=I2C_CR1_SWRST; // Reset del I2C
 //I2->CR1= ~I2C_CR1_SWRST;
 I2C1->CR1 = 0; // Habilita el reset del I2C1
//  I2C1->CR1 &= ~I2C_CR1_SWRST; // Desactiva el reset del I2C1
//I2->FLTR=16; // Apaga filtro an�logo
 cc2=((unsigned char)(R.PCLK1()/1000000))&0x3F;
 I2->CR2 = cc2; // Calculo del FREQ 
 I2->CCR=0;
 ccr=(unsigned int)(R.PCLK1()/(2.0*Velocidad)); // Calculo del CCR
 if(ccr<4)ccr=4; // M�nimo permitido en modo Sm
 I2->CCR =ccr; // Asignaci�n del CCR
 tr=(unsigned char)(R.PCLK1()/1000000)+1;
 I2->TRISE =tr; // Calculo del TRISE 1us
 I2C1->CR1 |= I2C_CR1_PE; // Activa el I2C1
}
void i2c :: DatoI2C(unsigned char direccion, unsigned char dato) {
	(void) I2C1->SR1;
    (void) I2C1->SR2;
    // Activar ACK para recibir ACK del esclavo
    I2C1->CR1 |= I2C_CR1_ACK; 
    // Generar la condición de inicio (START)
    I2C1->CR1 |= I2C_CR1_START;
	while(!(I2->SR1&I2C_SR1_SB)); // Espera fin del START
	(void) I2C1->SR1;
    // Transmitir la dirección del esclavo con el bit de escritura (0)
    I2C1->DR = direccion&0xFE;  // o simplemente direccion (el bit menos significativo debe ser 0 para escritura
    // Esperar a que se establezca la bandera de dirección (ADDR) o Acknowledge Failure (AF)
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
    // Leer SR1 y SR2 para limpiar la bandera de ADDR
    (void) I2C1->SR1;
    (void) I2C1->SR2;
    // Esperar hasta que el registro de datos esté listo para transmitir (TXE)
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    // Transmitir el dato al registro de datos (DR)
    I2C1->DR = dato; 
    // Esperar a que se complete la transmisión del byte (esperar a que BTF se establezca)
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    // Generar condición de parada (STOP)
    I2C1->CR1 |= I2C_CR1_STOP;
    // Esperar hasta que el bus esté libre (BUSY se borra)
    while (I2C1->SR2 & I2C_SR2_BUSY);
}
void i2c :: DatosI2C(unsigned char direccion,unsigned char *dato,int n){
 I2->CR1|=I2C_CR1_ACK; // Activa ACK
 I2->CR1|=I2C_CR1_START; // Genera START
 while(!(I2->SR1&I2C_SR1_SB)); // Espera fin del START
 (void)I2->SR1; // Limpia banderas
 I2->DR=direccion&0xFE; // Transmite direcci�n del esclavo
 while(!(I2->SR1&I2C_SR1_ADDR)); // Espera trasmisi�n
 (void)I2->SR1; // Limpia banderas
 (void)I2->SR2;
 for(int k=0;k<n;k++){ // Transmite n Bytes
  while(!(I2->SR1&I2C_SR1_TXE));
  I2->DR=dato[k];  
  while(!(I2->SR1&I2C_SR1_BTF));
 }
 I2->CR1|=I2C_CR1_STOP; // Genera STOP
 while((I2->SR2&I2C_SR2_BUSY)); // Espera bus libre
}
bool i2c :: TestearDireccion(unsigned char id){
 unsigned int d;
 bool Ok;   
 Ok=false;
 I2->SR1=0;
 I2->CR1|=I2C_CR1_ACK; // Activa ACK
 I2->CR1|=I2C_CR1_START; // Genera START
 while(!(I2->SR1&I2C_SR1_SB)); // Espera fin del START
 I2->DR=id&0xFE; // Transmite direcci�n del esclavo
 while(1){ 
  d=I2->SR1;
  if(d&I2C_SR1_ADDR){Ok=true;break;}
  if(d&I2C_SR1_AF){Ok=false;break;}
 }
 I2->CR1|=I2C_CR1_STOP; // Genera STOP
 while((I2->SR2&I2C_SR2_BUSY)); // Espera bus libre
 return Ok;
}
unsigned char i2c :: RecibirI2C(unsigned char adr){
unsigned char dat;
 I2->CR1|=I2C_CR1_ACK; // Activa ACK
 I2->CR1|=I2C_CR1_START; // Genera START
 while(!(I2->SR1&I2C_SR1_SB)); // Espera fin del START
 (void)I2->SR1; // Limpia banderas
 I2->DR=(adr|1); // Transmite direcci�n del esclavo
 while(!(I2->SR1&I2C_SR1_ADDR)); // Espera trasmisi�n
 I2->CR1&=~I2C_CR1_ACK; // Activa NO ACK
 (void)I2->SR1; // Limpia banderas
 (void)I2->SR2;
 I2->CR1|=I2C_CR1_STOP; // Genera STOP
 while(!(I2->SR1&I2C_SR1_RXNE)); // Espera dato disponible
 dat=I2->DR; // Lee dato
 return dat; // Retorna el dato
}
void i2c :: RecibirI2Cs(unsigned char adr,unsigned char *dat,int n){
 int i=0,k=n;   
 I2->CR1|=I2C_CR1_ACK; // Activa ACK
 I2->CR1|=I2C_CR1_START; // Genera START
 while(!(I2->SR1&I2C_SR1_SB)); // Espera fin del START
 (void)I2->SR1; // Limpia banderas
 I2->DR=(adr|1); // Transmite direcci�n del esclavo 
 while(!(I2->SR1&I2C_SR1_ADDR)); // Espera trasmisi�n
 (void)I2->SR1; // Limpia banderas
 (void)I2->SR2;
 while(k>2){ // Buble de n-2 Bytes
  while(!(I2->SR1&I2C_SR1_RXNE)); // Espera dato disponible
  dat[i++]=I2->DR; // Lee dato
  I2->CR1|=I2C_CR1_ACK; // Activa ACK
  k--;
 }
 while(!(I2->SR1&I2C_SR1_RXNE)); // Espera dato disponible
 dat[i++]=I2->DR; // Lee dato
 I2->CR1&=~I2C_CR1_ACK; // Activa NO ACK
 I2->CR1|=I2C_CR1_STOP; // Genera STOP
 while(!(I2->SR1&I2C_SR1_RXNE)); // Espera dato disponible
 dat[i++]=I2->DR; // Lee dato
}
void i2c :: WriteDir(unsigned char sla,unsigned char adr,unsigned char dat){
	unsigned char Buf[2];
	Buf[0]=adr;
	Buf[1]=dat;
	DatosI2C(sla,Buf,2);
}
unsigned char i2c :: ReadDir(unsigned char sla,unsigned char adr){
	DatoI2C(sla,adr);
	return RecibirI2C(sla);
}

unsigned long Reloj :: APB1(void){
 unsigned long PC;
  PC=HCLK(); // Leer frecuencia del Core
  // Eval�a pre-escala del PCLK1
 switch((RCC->CFGR>>RCC_CFGR_PPRE1_Pos)&7){
  case 4:PC/=2.0;return 2*PC;
  case 5:PC/=4.0;return 2*PC;
  case 6:PC/=8.0;return 2*PC;
  case 7:PC/=16.0;return 2*PC;
  default:return PC;
 }
}
unsigned int Reloj :: HCLK(void){
	SystemCoreClockUpdate();
 return (unsigned int)SystemCoreClock;
}
unsigned int Reloj ::PCLK1(void){ // Funci�n para identificar frecuencia APB1
 unsigned int PC;
 PC=HCLK(); // Leer frecuencia del Core
 // Eval�a pre-escala del PCLK1
 switch((RCC->CFGR>>RCC_CFGR_PPRE1_Pos)&7){
  case 4: PC/=2.0;break;
  case 5: PC/=4.0;break;
  case 6: PC/=8.0;break;
  case 7: PC/=16.0;break;
  default:break;
 }
 return PC; // Retorna PCLK1
}
unsigned long Reloj ::APB2(void){
 unsigned long PC;
 PC=HCLK(); // Leer frecuencia del Core
  // Eval�a pre-escala del PCLK2
 switch((RCC->CFGR>>RCC_CFGR_PPRE2_Pos)&7){
  case 4:PC/=2.0;return 2*PC;
  case 5:PC/=4.0;return 2*PC;
  case 6:PC/=8.0;return 2*PC;
  case 7:PC/=16.0;return 2*PC;
  default:return PC;
 }
}
void Reloj :: Sys(double T){
 //void SysTick_Handler(void){ } Hacer Cada T 
 SystemCoreClockUpdate();//Actualiza CoreClock
 SysTick_Config(SystemCoreClock*T);//Periodo de T segundos   
}
