//--------------------------------------------------------------
// File     : stm32_ub_lis302dl.c
// Datum    : 06.03.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : GPIO, STM32_UB_SPI1
// Funktion : LIS302DL (3Achs Beschleunigungs Sensor, 8bit)
//            Polling-Mode, keine Interrupts,
//            keine Click- und FreeFall-Auswertung
//
// Hinweis  : Settings :
//            SPI-Mode = 3, FRQ-Max = 10MHz
//            SPI1 [CLK=PA5, MOSI=PA7, MISO=PA6]
//            Chip-Select an PE3
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_lis302dl.h"
#include "stm32_ub_spi1.h"

//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_LIS302_CalcData(void);
void P_LIS302_initIO(void);
void P_LIS302_CS(BitAction wert); 
uint8_t P_LIS302_Read1Adr(uint8_t adr);
void P_LIS302_Write1Adr(uint8_t adr, uint8_t wert);



//--------------------------------------------------------------
// Initialisierung vom LIS302DL
// scale : [SCALE_2G, SCALE_8G]
// Return_wert :
//  -> ERROR   , wenn Initialisierung fehlgeschlagen
//  -> SUCCESS , wenn Initialisierung ok war 
//--------------------------------------------------------------
ErrorStatus UB_LIS302_Init(LIS302_SCALE_t scale)
{
  ErrorStatus ret_wert=ERROR;
  uint8_t wert;

  // Init aller Variabeln
  LIS302.x_achse=0;
  LIS302.y_achse=0;
  LIS302.z_achse=0;
  LIS302_RAW.faktor=LIS302_2G_FAKTOR;
  LIS302_RAW.x_achse=0;
  LIS302_RAW.y_achse=0;
  LIS302_RAW.z_achse=0;


  // Init der Chip-Select-Leitung
  P_LIS302_initIO();
  // ChipSelect auf Hi
  P_LIS302_CS(Bit_SET);
  // init vom SPI-1 im Mode-3
  ret_wert=UB_SPI1_Init(SPI_MODE_3); 
  if(ret_wert==SUCCESS) {
    // auslesen vom Identifier
    wert=P_LIS302_Read1Adr(LIS302_REG_ID);
    if(wert!=LIS302_ID) {
      // empfangene ID ist falsch
      ret_wert=ERROR;
    }
    else {
      // ID ist richtig
      // LIS302 einschalten
      if(scale==SCALE_2G) {
        P_LIS302_Write1Adr(LIS302_REG_CTRL1,LIS302_PWR_2G);
        LIS302_RAW.faktor=LIS302_2G_FAKTOR;
      }
      else {
        P_LIS302_Write1Adr(LIS302_REG_CTRL1,LIS302_PWR_8G);
        LIS302_RAW.faktor=LIS302_8G_FAKTOR;
      }
    }
  }   

  return(ret_wert); 
}


//--------------------------------------------------------------
// Auslesen vom LIS302DL und fuellen der Struktur "LIS302"
// (die Funktion muss zyklisch aufgerufen werden) 
//   LIS302.x_achse => Beschleunigung der X-Achse (in +/- mg)
//   LIS302.y_achse => Beschleunigung der Y-Achse (in +/- mg)
//   LIS302.z_achse => Beschleunigung der Z-Achse (in +/- mg)
//--------------------------------------------------------------
void UB_LIS302_Read(void)
{
  // X-Achse Rohdaten auslesen
  LIS302_RAW.x_achse=P_LIS302_Read1Adr(LIS302_REG_X);
  // Y-Achse Rohdaten auslesen
  LIS302_RAW.y_achse=P_LIS302_Read1Adr(LIS302_REG_Y);
  // Z-Achse Rohdaten auslesen
  LIS302_RAW.z_achse=P_LIS302_Read1Adr(LIS302_REG_Z);

  // Daten in mg-Werte umrechnen
  P_LIS302_CalcData();
}


//--------------------------------------------------------------
// High-Pass Filter einstellen
// filter : [FILTER_OFF, FILTER_2Hz, FILTER_1Hz]
//          [FILTER_500mHz, FILTER_250mHz]
//--------------------------------------------------------------
void UB_LIS302_SetFilter(LIS302_FILTER_t filter)
{
  if(filter==FILTER_OFF) {
    P_LIS302_Write1Adr(LIS302_REG_CTRL2,LIS302_FILTER_OFF);
  }
  else if(filter==FILTER_2Hz) {
    P_LIS302_Write1Adr(LIS302_REG_CTRL2,LIS302_FILTER_M0);
  }
  else if(filter==FILTER_1Hz) {
    P_LIS302_Write1Adr(LIS302_REG_CTRL2,LIS302_FILTER_M1);
  }
  else if(filter==FILTER_500mHz) {
    P_LIS302_Write1Adr(LIS302_REG_CTRL2,LIS302_FILTER_M2);
  }
  else {
    P_LIS302_Write1Adr(LIS302_REG_CTRL2,LIS302_FILTER_M3);
  }
}


//--------------------------------------------------------------
// interne Funktion
// Beschleunigungs Roh-Daten in mg-Werte umrechnen
// (Roh-Daten sind im 2er-Komplement gespeichert)
//--------------------------------------------------------------
void P_LIS302_CalcData(void)
{
  uint8_t wert;
  int16_t beschleunigung; 
 
  // X-Daten umrechnen  
  wert=LIS302_RAW.x_achse;
  if((wert&0x80)!=0) {
    // negativ
    wert=(0xFF^wert);
    wert++;
    beschleunigung=(int16_t)(LIS302_RAW.faktor*wert);
    LIS302.x_achse=0-beschleunigung;
  }
  else {
    // positiv
    beschleunigung=(int16_t)(LIS302_RAW.faktor*wert);
    LIS302.x_achse=beschleunigung;
  }

  // Y-Daten umrechnen  
  wert=LIS302_RAW.y_achse;
  if((wert&0x80)!=0) {
    // negativ
    wert=(0xFF^wert);
    wert++;
    beschleunigung=(int16_t)(LIS302_RAW.faktor*wert);
    LIS302.y_achse=0-beschleunigung;
  }
  else {
    // positiv
    beschleunigung=(int16_t)(LIS302_RAW.faktor*wert);
    LIS302.y_achse=beschleunigung;
  }

  // Z-Daten umrechnen  
  wert=LIS302_RAW.z_achse;
  if((wert&0x80)!=0) {
    // negativ
    wert=(0xFF^wert);
    wert++;
    beschleunigung=(int16_t)(LIS302_RAW.faktor*wert);
    LIS302.z_achse=0-beschleunigung;
  }
  else {
    // positiv
    beschleunigung=(int16_t)(LIS302_RAW.faktor*wert);
    LIS302.z_achse=beschleunigung;
  }
}


//--------------------------------------------------------------
// interne Funktion
// Init der ChipSelect-Leitung
//--------------------------------------------------------------
void P_LIS302_initIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Init der Chip-Select-Leitung
  RCC_AHB1PeriphClockCmd(LIS302_CS_GPIO_CLK,ENABLE);

  GPIO_InitStructure.GPIO_Pin = LIS302_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LIS302_CS_GPIO_PORT, &GPIO_InitStructure);
}


//--------------------------------------------------------------
// interne Funktion
// Pegel von ChipSelect einstellen
//--------------------------------------------------------------
void P_LIS302_CS(BitAction wert) {
  if(wert==Bit_RESET) {
    LIS302_CS_GPIO_PORT->BSRRH = LIS302_CS_PIN;
  }
  else {
    LIS302_CS_GPIO_PORT->BSRRL = LIS302_CS_PIN;
  }
}


//--------------------------------------------------------------
// interne Funktion
// auslesen einer Adresse
// adr    : 0x00 bis 0x3F
// return : 0x00 bis 0xFF
//--------------------------------------------------------------
uint8_t P_LIS302_Read1Adr(uint8_t adr)
{
  uint8_t ret_wert=0;
  uint8_t spi_wert;

  // ChipSelect auf Lo
  P_LIS302_CS(Bit_RESET);

  // eine Adresse auslesen
  spi_wert=(adr&0x3F);
  spi_wert|=0x80;

  // Adresse senden
  UB_SPI1_SendByte(spi_wert);

  // Adresse auslesen
  ret_wert=UB_SPI1_SendByte(0x55);

  // ChipSelect auf Hi
  P_LIS302_CS(Bit_SET);

  return(ret_wert);  
}


//--------------------------------------------------------------
// interne Funktion
// beschreiben einer Adresse
// adr    : 0x00 bis 0x3F
// wert   : 0x00 bis 0xFF
//--------------------------------------------------------------
void P_LIS302_Write1Adr(uint8_t adr, uint8_t wert)
{
  uint8_t spi_wert;

  // ChipSelect auf Lo
  P_LIS302_CS(Bit_RESET);

  // eine Adresse beschreiben
  spi_wert=(adr&0x3F);

  // Adresse senden
  UB_SPI1_SendByte(spi_wert);

  // Adresse beschreiben
  UB_SPI1_SendByte(wert);

  // ChipSelect auf Hi
  P_LIS302_CS(Bit_SET);
}
