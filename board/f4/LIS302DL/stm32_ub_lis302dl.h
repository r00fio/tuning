//--------------------------------------------------------------
// File     : stm32_ub_lis302dl.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __STM32F4_UB_LIS302DL_H
#define __STM32F4_UB_LIS302DL_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx.h"


// Auflösung
typedef enum {
  SCALE_2G =0,    // Auflösung +/- 2g
  SCALE_8G        // Auflösung +/- 8g
}LIS302_SCALE_t;


// Filter
typedef enum {
  FILTER_OFF =0,  // High-Pass Filter abgeschaltet
  FILTER_2Hz,     // Filter eingeschaltet (2 Hz)
  FILTER_1Hz,     // Filter eingeschaltet (1 Hz)
  FILTER_500mHz,  // Filter eingeschaltet (0,5 Hz)
  FILTER_250mHz   // Filter eingeschaltet (0,25 Hz) 
}LIS302_FILTER_t;


// LIS302-Roh-Daten
typedef struct {
  uint8_t faktor;    // Auflösungsfaktor
  uint8_t x_achse;   // Rohdaten der X-Achse 
  uint8_t y_achse;   // Rohdaten der Y-Achse
  uint8_t z_achse;   // Rohdaten der Z-Achse
}LIS302_RAW_t;
LIS302_RAW_t LIS302_RAW;


// LIS302-Beschleunigungs-Daten
typedef struct {
  int16_t x_achse; // Beschl. der X-Achse (in +/- mg)
  int16_t y_achse; // Beschl. der Y-Achse (in +/- mg)
  int16_t z_achse; // Beschl. der Z-Achse (in +/- mg)
}LIS302_t;
LIS302_t LIS302;



//--------------------------------------------------------------
// ChipSelect-Pin vom LIS302DL
//--------------------------------------------------------------
#define LIS302_CS_PIN         GPIO_Pin_3
#define LIS302_CS_GPIO_PORT   GPIOE
#define LIS302_CS_GPIO_CLK    RCC_AHB1Periph_GPIOE


//--------------------------------------------------------------
// Register Adressen
//--------------------------------------------------------------
#define  LIS302_REG_ID       0x0F   // ID Register
#define  LIS302_REG_CTRL1    0x20   // CTRL1
#define  LIS302_REG_CTRL2    0x21   // CTRL2
#define  LIS302_REG_X        0x29   // X-Achse
#define  LIS302_REG_Y        0x2B   // Y-Achse
#define  LIS302_REG_Z        0x2D   // Z-Achse


//--------------------------------------------------------------
// LIS302DL Defines
//--------------------------------------------------------------
#define  LIS302_ID           0x3B   // ID vom LIS302DL IC
#define  LIS302_PWR_2G       0x47   // Power-On (2G,100Hz,X,Y,Z)
#define  LIS302_PWR_8G       0x67   // Power-On (8G,100Hz,X,Y,Z)
#define  LIS302_FILTER_OFF   0x00   // Filter-Off
#define  LIS302_FILTER_M0    0x10   // Filter-On (2Hz)
#define  LIS302_FILTER_M1    0x11   // Filter-On (1Hz)
#define  LIS302_FILTER_M2    0x12   // Filter-On (0,5Hz)
#define  LIS302_FILTER_M3    0x13   // Filter-On (0,25Hz)

#define  LIS302_2G_FAKTOR    18     // 18mg pro Digit (+/-2g)
#define  LIS302_8G_FAKTOR    72     // 72mg pro Digit (+/-8g)



//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
ErrorStatus UB_LIS302_Init(LIS302_SCALE_t scale);
void UB_LIS302_Read(void);
void UB_LIS302_SetFilter(LIS302_FILTER_t filter);




//--------------------------------------------------------------
#endif // __STM32F4_UB_LIS302DL_H
