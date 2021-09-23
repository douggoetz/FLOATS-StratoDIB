/*
 *  DIBHardware.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  Pin and port definitions for the DIB
 */

#ifndef DIBHARDWARE_H
#define DIBHARDWARE_H

// Define for the type of operation
//#define ZEROPRESS
#define ZEPHYR

// Serial Ports
#define ZEPHYR_SERIAL   Serial1
#define MCB_SERIAL      Serial2
//#define EFU_SERIAL      Serial3 //Removed for Mondo w/ LORA comms

// Digital Pins
#define FTR_POWER       26 //Switch for FTR on and off
//#define FORCEON_232     29  //removed for Mondo
//#define FORCEOFF_232    30
#define SAFE_PIN        31 //Pin for Zephyr safe mode: HIGH in SAFE Mode
//#define MCB_IO_1        32
#define PULSE_LED       36
#define EXT_LED1        39
#define EXT_LED2        30 

// Fiber Optic Switch Digital Pins
//#define Switch2_EFU      3 //High
//#define Switch2_FTR      2
//#define SwitchStatus_EFU 5
//#define SwitchStatus_FTR 4

//LORA pins
#define LORA_INT        25
#define LORA_SS         24
#define LORA_RESET      27

//WizIO Ethernet Pins
#define WizCS            14
#define WizReset         18 //LOW = reset, pulse for 2 us
#define WizPWD           17 // power down for WizIO, Low = normal mode, HIGH = power down
#define WizINT           19 //interrupt 

//LTC2983 Pins and Channels //On SPI0 bus
#define LTC_TEMP_CS_PIN 15
#define LTC_TEMP_RESET_PIN 28

#define THERM_SENSE_CH 2
#define FOTS1_THERM_CH 3
#define FOTS2_THERM_CH 5
#define DCDC_THERM_CH 7
#define SPARE_THERM_CH 9

#define RTD_SENSE_CH 16
#define OAT_PRT1_RTD_CH 18
#define OAT_PRT2_RTD_CH 20


// MS5803 Pressure Sensor I2C pins
#define PressSCL 37
#define PressSDA 38

// Analog Pins
//#define VMON_3V3        A7 
//#define VMON_5V        A22
//#define VMON_12V_MTR    A6
#define VMON_12V       A14 //changed from A20 for MONDO
#define VMON_15V        A2
#define IMON_INST      A15

#endif /* DIBHARDWARE_H */

