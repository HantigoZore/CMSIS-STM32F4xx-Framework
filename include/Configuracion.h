#include "stm32f4xx.h"  
#include "stdio.h"

// DEFINICIONES DE PINES //
#define PA0   0x00
#define PA1   0x01
#define PA2   0x02
#define PA3   0x03
#define PA4   0x04
#define PA5   0x05
#define PA6   0x06
#define PA7   0x07
#define PA8   0x08
#define PA9   0x09
#define PA10  0x0A
#define PA11  0x0B
#define PA12  0x0C
#define PA13  0x0D
#define PA14  0x0E
#define PA15  0x0F
#define PB0   0x10
#define PB1   0x11
#define PB2   0x12
#define PB3   0x13
#define PB4   0x14
#define PB5   0x15
#define PB6   0x16
#define PB7   0x17
#define PB8   0x18
#define PB9   0x19
#define PB10  0x1A
#define PB11  0x1B
#define PB12  0x1C
#define PB13  0x1D
#define PB14  0x1E
#define PB15  0x1F
#define PC0   0x20
#define PC1   0x21
#define PC2   0x22
#define PC3   0x23
#define PC4   0x24
#define PC5   0x25
#define PC6   0x26
#define PC7   0x27
#define PC8   0x28
#define PC9   0x29
#define PC10  0x2A
#define PC11  0x2B
#define PC12  0x2C
#define PC13  0x2D
#define PC14  0x2E
#define PC15  0x2F

//Arduino
#define AP0 PA0
#define AP1 PA1
#define AP2 PA4
#define AP3 PB0
#define AP4 PC1
#define AP5 PC0
#define DP0 PA3
#define DP1 PA2
#define DP2 PA10
#define DP3 PB3
#define DP4 PB5
#define DP5 PB4
#define DP6 PB10
#define DP7 PA8
#define DP8 PA9
#define DP9 PC7
#define DP10 PB6
#define DP11 PA7
#define DP12 PA6
#define DP13 PA5
#define DP14 PB9
#define DP15 PB8
// Definiciones seriales Arduino
#define SPI_CLK  DP13
#define SPI_MISO DP12
#define SPI_MOSI DP11
#define I2C_SCL DP15
#define I2C_SDA DP14
#define USART_TX DP1
#define USART_RX DP0
//

#define LED1 PA5
#define USER PC13  


#ifndef _CONFIGURACION_H
#define _CONFIGURACION_H

// Definici�n de apuntador a funci�n //
typedef void (*FunInt)(void);
typedef void (*FunInt2)(unsigned char d);

class Reloj{
  private:

  public:
   unsigned long APB1(void);
   unsigned long APB2(void);
   unsigned int  HCLK(void);
   unsigned int PCLK1(void);
   void Sys(double T);
};

class Pines{
   private:
     GPIO_TypeDef * Port;
     unsigned int mask1, mask0; 
  public:
  void ModoPin(char Pin, int Modo);
  void SalidaPin(char Pin, int Salida);
  int  EntradaPin(char Pin, int Lectura);
  void Interrupcion(FunInt Funcion,char Pin, int Lectura); //Interrupciones
};

class Analogo : Pines{
  private:

  public:
    void Conversion(char Pin); //ADC
    void IniciarADC(void); //Iniciar Conversion 
};

class USART{
  private:
     USART_TypeDef *USA;
     Reloj Re;
  public:
      void Comunicacion(int USART,char Puerto,int Velocidad); //USART
      void TransmitirDato(unsigned char d);
      void TransmitirDatos(char *d);
       void Interrupcion(FunInt fun,int usa);// M�todo para asignar interrupci�n de Rx
};

class Timers : Reloj{
  private:
  TIM_TypeDef * Tim;
  Pines P;
  GPIO_TypeDef * Puerto;
  public:
  void PWM(int Timerr,char Puerto,int CH1,int CH2, int CH3,int CH4,double Periodo);
  void CicloUtil(int Timer,int CH,double Ciclo);
  void Timer(int Time,double Periodo);
  double LeerPeriodo(void);
  void IniciarEncoder(int Tmr,char CanalA,char CanalB);
  int GetDiferencial(int Tmr);
  int GetPosicion(int Tmr);
  void CapturarPulso(int Time,double Periodo,char Puerto,int Pin);
  void Interrupcion(int Time,FunInt Funcion);
};

class i2c{
  private:
  Reloj R;
  public:
  I2C_TypeDef *I2; // Estructura I2C
  void IDOSC(int I2C,int Velocidad);
  void DatoI2C(unsigned char direccion,unsigned char dato);
  void DatosI2C(unsigned char direccion,unsigned char *dato,int n);
  bool TestearDireccion(unsigned char id);
  unsigned char RecibirI2C(unsigned char adr);
  void RecibirI2Cs(unsigned char adr,unsigned char *dat,int n); 
  void WriteDir(unsigned char sla,unsigned char adr,unsigned char dat);
  unsigned char ReadDir(unsigned char sla,unsigned char adr);
};
#endif
