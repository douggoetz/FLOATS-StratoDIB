/*
 *  StratoDIB.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file declares an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the FLOATS Data Interface Board, or DIB.
 */

#ifndef STRATODIB_H
#define STRATODIB_H

#include "StratoCore.h"
#include "DIBHardware.h"
#include "DIBBufferGuard.h"
#include "MCBComm.h"
#include "EPICComm.h"
#include <LoopbackStream.h>
#include <LTC2983Manager.h>
#include <FTR3000.h>
#include <Ethernet.h>
#include <i2c_t3.h>
#include <MS5803_02.h> 



#define INSTRUMENT      FLOATS

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

#define MCB_RESEND_TIMEOUT      10
#define ZEPHYR_RESEND_TIMEOUT   60

#define LOG_ARRAY_SIZE  101

#define PAddr  0x77 // MS5803_02BA pressure sensor i2C address

// todo: update naming to be more unique (ie. ACT_ prefix)
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,

    // scheduled actions
    SEND_IMR,
    RESEND_SAFETY,
    RESEND_MCB_LP,
    RESEND_MOTION_COMMAND,
    RESEND_TM,
    RESEND_FULL_RETRACT,
    EXIT_ERROR_STATE,

    // internal command actions
    COMMAND_REEL_OUT,
    COMMAND_REEL_IN,
    COMMAND_DOCK,
    COMMAND_MOTION_STOP,

    //FTR actions
    HOUSEKEEPING,
    IDLE_EXIT,
    POWERON_FTR,
    CONFIGURE_FTR,
    FTR_SCAN,
    CHECK_FTR_STATUS,
    START_EFU,
    LISTEN_EFU,
    BUILD_TELEM,
    BUILD_BURST_TELEM,
    SEND_TELEM,
    FTR_TM_RESEND,

    // used for tracking
    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
    MOTION_DOCK,
    MOTION_UNDOCK
};

enum FlightSubMode_t : uint8_t {
    FTR_SUBMODE = 0,
    MCB_SUBMODE = 1
};

enum FTRStatus_t : uint8_t {
    ENTERSTAT,
    FTR_READY,
    FTR_NOTREADY,
    FTR_ERROR
};

enum FTRMeasureType_t : uint8_t {
    BURST,
    AVERAGE,
};

enum TelemType_t : uint8_t{
    HOUSEFTR = 11,
    DATAEFU = 22,
    HOUSEEFU = 33,
    DATAFTR = 44
};

// Flight mode states for FTR
enum FLStatesFTR_t : uint8_t {
    FTR_ENTRY = MODE_ENTRY,

    // before anything else
    FTR_GPS_WAIT,

    // FTR operation states
    FTR_ENTER_IDLE, //Fiber switch to EFU, turn off FTR, configure LTC chip
    FTR_IDLE, //FLOATS is between measurements and listening for EFU errors
    FTR_WARMUP, //Warmup state for FTR3000 (checks FTR status byte)
    FTR_MEASURE, //Measurement operations
    FTR_EFU_START,
    FTR_EFU, //FLOATS is waiting to recieve EFU telemetry based on a synced time
    FTR_SEND_EFU,
    FTR_VERIFY_EFU,
    FTR_FINISH_EFU,
    FTR_SEND_TELEMETRY,
    FTR_TM_ACK,

    // ----------------------------------------------------
    // Define FTR states here
    // ----------------------------------------------------

    // general off-nominal states in response to following landing states
    FTR_ERROR_LOOP,
    FTR_SHUTDOWN_LOOP,

    // 253 - 255, defined in StratoCore
    FTR_ERROR_LANDING = MODE_ERROR,
    FTR_SHUTDOWN_LANDING = MODE_SHUTDOWN,
    FTR_EXIT = MODE_EXIT
};

class StratoDIB : public StratoCore {
public:
    StratoDIB();
    ~StratoDIB() { };

    // called before the main loop begins
    void InstrumentSetup();

    // called at the end of each main loop
    void InstrumentLoop();

    // called in each main loop
    void RunMCBRouter();

    void RunEFURouter();

private:
    // instances
    MCBComm mcbComm;
    FTR ftr;
    LTC2983Manager ltcManager;
    EPICComm efucomm; 
    EthernetClient client;
    LoopbackStream LoRaBuff;
    MS_5803 sensor;

    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();

    // Sub-modes for DIB FlightMode
    void FlightFTR();
    void FlightMCB();
    FlightSubMode_t flight_submode = FTR_SUBMODE; // reboot default
    
    //uint8_t FltConfig = FLIGHT_ERROR;

    //Hardware Operation functions
    void resetLtcSpi(); //SPI0 reset for use with LTC2983
    void LTCSetup(); //Sets up LTC channels
    void ReadFullTemps(); //gets full temperature string
    void ReadVoltages(); //get voltages
    void FTRStatusReport(uint8_t); //sets ftr_status variable based on FTR status byte retrieved with FTR3000 library
    void ReadPressure();
    void ReadInstCurrent();
    
    void HousekeepingFTR();
    void SaveSingleScan();

    //FTR Operation Parameters
    long HKcounter;
    uint16_t Measure_Period = 10*60; //10 minutes nominally
    uint16_t HK_Loop = 300; //number of seconds between idle HK data retreival
    uint16_t Idle_Period = 30; //Should be opposite duty cycle of measure period minus Start_EFU_Period telemetry period
    uint16_t Stat_Limit = 300/20; //number of FTR status requests before timeout and FTR3000 reset (nominally 5mins/20second requests = 15)
    uint16_t Optical_Resolution = 3; //nominally 1m vertical resolution
    
    int Status_Loop = 20; // number of seconds between FTR status requests
    int Scan_Loop; //number of seconds per scan. FTR3000 scan time hardset to ~20s or ftr.FTRAvg = HW_2_19  and ftr.FTRPulseInterval = INTERVAL_40US
    int Stat_Counter = 0; //number of times status is requested prior to entering measurement state or resetting FTR3000
    int EFU_Loop = 1; //number of seconds between EFU retrieval attempts during EFU telemetry state
    int Scan_Counter = 0; //number of ~20s FTR3000 scans made
    uint8_t EthernetCount = 0;
   
   
    uint8_t Burst_Counter = 0;
    uint8_t Burst_Limit = 8; //8, 2min scans = 16 minutes

    //Operational Flags
    bool EFU_Ready = false;
    bool EFU_Received = false;
    bool EnterMeasure = 0;
    
    // //Zeph Message Handling
    // bool TMtoSend = false;
    // long ZephCount;
    // long SendTimer = 1000;


    //State variables
    uint8_t ftr_status;
    uint8_t measure_type = AVERAGE;


    //Data Variables
    float FOTS1Therm;
    float FOTS2Therm;
    float DC_DC_Therm;
    float SpareTherm;
    float RTD1;
    float RTD2;
    float V_Zephyr;
    float C_Zephyr;
    //float V_3v3;
    //float V_5TX;
    float V_12FTR;
    float P_mbar;
    float  P_tempC; 

    //MS5803_02BA pressure sensor variables
    unsigned long Coff[6];
    unsigned long Ti = 0;
    unsigned long offi = 0;
    unsigned long sensi = 0;
    unsigned int data[3];
    

    //FTR Arrays
    uint16_t RamanLength = 1725;
    byte RamanBin[17000]; //Binary array that contains raw FTR scan
    uint16_t Stokes[2000]; //instant stokes scan from RamanBin
    uint16_t Astokes[2000]; //instant antistokes scan from RamanBin
    uint16_t StokesElements[2000]; //number of good scans per array point for stokes averaging
    uint16_t AstokesElements[2000]; //number of good scans per array point for antistokes averaging
    float StokesAvg[2000]; //stokes CO-add values that will be averaged using elements array and passed as TM
    float AStokesAvg[2000]; //antistokes Co-add values that will be averaging using the elements array and passed as TM
    uint16_t Stokes_Counter = 0;
    uint16_t Astokes_Counter = 0;
    
    String SDFileName = "";

    //Route and Handle messages from EFUComm
    void EFUWatch(); //Sets EFU ready flag when predetermined minute is reached
    void EFURX();
    void HandleEFUMess();
    void BuildEFUTelem();
    void BuildEFUHouseKeeping();

    //EFU variables
    int RSSI_val;
    int CommID = 999;
    int CommVal = 0;
    uint32_t EFU_Time_RX;
    float EFU_Lat_RX;
    float EFU_Lon_RX;
    uint16_t EFU_Alt_RX;
    uint16_t EFU_TsenT_RX;
    uint32_t EFU_TsenP_RX;
    uint32_t EFU_TsenTP_RX;
    uint16_t EFU_VBatt_RX;
    uint16_t EFU_Vdcdc_RX;
    uint16_t EFU_Vteensy_RX;
    float EFU_Tbatt_RX;
    float EFU_Tpcb_RX;
    uint8_t EFU_heat_RX;

    //EFU telem handling arrays
    uint16_t EFUdataCounter = 0;
    uint16_t EFUhouseCounter = 0;
   // uint16_t EFU_TX[10][12] = {};
    uint16_t EFU_TX[12];
    uint16_t EFU_HKTX[7];
    int edex = 0;
    int HKdex = 0;
    uint16_t EFUBuff[12*60]; //12*60
    uint16_t EFUHKBuff[7*100]; // 7*100

    //Handle FTR cans and telemetry
    void HandleFTRBin();
    void XMLHeader(const char messval);
   // void RS232Wake();

    // Telcommand handler - returns ack/nak
    //bool TCHandler(Telecommand_t telecommand);
    void TCHandler(Telecommand_t telecommand);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Correctly set an action flag
    void SetAction(uint8_t action);

    // Monitor the action flags and clear old ones
    void WatchFlags();

    // Handle messages from the MCB
    void HandleMCBASCII();
    void HandleMCBAck();
    void HandleMCBBin();
    uint8_t binary_mcb[50];

    // Start any type of MCB motion
    bool StartMCBMotion();

    // Add an MCB motion TM packet to the binary TM buffer
    void AddMCBTM();

    // Set variables and TM buffer after a profile starts
    void NoteProfileStart();

    // Send a telemetry packet with MCB binary info
    void SendMCBTM(StateFlag_t state_flag, String message);

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;
    bool mcb_reeling_in = false;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // tracks if a resend of any message has already been attempted
    bool resend_attempted = false;

    // current profile parameters
    float deploy_length = 0.0f;
    float retract_length = 0.0f;
    float dock_length = 0.0f;
    float deploy_velocity = 80.0f;
    float retract_velocity = 80.0f;
    float dock_velocity = 20.0f;

    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};

    // keep a statically allocated array for creating up to 100 char TM state messages
    char log_array[LOG_ARRAY_SIZE] = {0};
};

#endif /* STRATODIB_H */