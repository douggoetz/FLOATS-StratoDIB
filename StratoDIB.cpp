/*
 *  StratoDIB.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the FLOATS Data Interface Board, or DIB.
 */

#include "StratoDIB.h"
#include "Serialize.h"

#include <SdFat.h>
#include <SdFatConfig.h>

File datafile;
SdFatSdio SD_FTR;
SdFat sdfat;


StratoDIB::StratoDIB()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
    , mcbComm(&MCB_SERIAL)
    , ftr(&client, &Serial)
    , ltcManager(LTC_TEMP_CS_PIN, LTC_TEMP_RESET_PIN, THERM_SENSE_CH, RTD_SENSE_CH)
    , efucomm(&LoRaBuff)
    , sensor(MS_5803(512))

    
{
}

// --------------------------------------------------------
// General instrument functions
// --------------------------------------------------------

// note serial setup occurs in main arduino file
void StratoDIB::InstrumentSetup()
{

    SPI.begin(); //SPI0 for MONDO WIZIO
    
    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);

    //FTR
    pinMode(FTR_POWER, OUTPUT);
    ftr.poweroff();
    ftr.FTRAvg = HW_2_19; // TO DO: MAKE STRATODIB VARIABLE CONFIGURABLE W/ TC
    ftr.FTRPulseInterval = INTERVAL_40US; // TO DO: MAKE STRATODIB VARIABLE CONFIGURABLE W/ TC
    Scan_Loop = 20;// ftr.FTRScanTime()/1000;

    Serial.println("Setup FTR");
    
    //WIZIO
    pinMode(WizReset, OUTPUT);
    pinMode(WizCS, OUTPUT);
    digitalWrite(WizCS, HIGH); //deselects Wiz820io from SPI0

    Serial.println("Setup WIZIO");

    //LTC2983
    pinMode(LTC_TEMP_RESET_PIN,OUTPUT);
    pinMode(LTC_TEMP_CS_PIN,OUTPUT);
    ltcManager.channel_assignments[FOTS1_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[FOTS2_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[DCDC_THERM_CH]   = THERMISTOR_44006;
    ltcManager.channel_assignments[SPARE_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[OAT_PRT1_RTD_CH] = RTD_PT_100;
    ltcManager.channel_assignments[OAT_PRT2_RTD_CH] = RTD_PT_100;
    ltcManager.InitializeAndConfigure();
    digitalWrite(LTC_TEMP_RESET_PIN, HIGH); //deselects LTC from SPI0
    Serial.println("Setup LTC");

    //MS5803 Pressure Sensor
   // Wire1.begin();
    Wire1.setSDA(PressSDA);
    Wire1.setSCL(PressSCL);
    if (sensor.initializeMS_5803()) {
        Serial.println( "MS5803 CRC check OK." );
    } 
    else {
        Serial.println( "MS5803 CRC check FAILED!" );
    }
   
    // ADC Setup
    pinMode(VMON_12V,INPUT);
    pinMode(VMON_15V,INPUT);
    analogReadResolution(12);
    analogReference(EXTERNAL);
     Serial.println("Setup ADC");

    //LORA Setup
     /* Set up the second SPI Port for the LoRa Module */
    SPI1.setSCK(20);
    SPI1.setMISO(5);
    SPI1.setMOSI(21);

    SPI1.begin(); // SPI1 for MONDO LoRa
    log_debug("Setup SPI1");
    if (!efucomm.Begin(868E6, LORA_SS, LORA_RESET, LORA_INT, SPI1)) {
        log_debug("Starting LoRa failed!");
        while (1);
  }
  
    LoRa.enableCrc();
    LoRa.setSignalBandwidth(250E3);
    LoRa.setSpreadingFactor(11);
    efucomm.SetTXPower(20);

    Serial.println("Setup LORA");

    mcbComm.AssignBinaryRXBuffer(binary_mcb, 50);

    if (!SD_FTR.begin()) {
    Serial.println("SD Card not installed");
    //return false;
    }

    HKcounter = millis();
}

void StratoDIB::InstrumentLoop()
{
    WatchFlags();

}

// void StratoDIB::RS232Wake()
// {

//     pinMode(1,OUTPUT);
//     digitalWrite(1,HIGH);
//     delayMicroseconds(1);
//     digitalWrite(1,LOW);
//     delayMicroseconds(1);
//     pinMode(1,INPUT);
//     Serial1.begin(115200);
// }


// --------------------------------------------------------
// Telecommand handler
// --------------------------------------------------------

// The telecommand handler must return ACK/NAK
void StratoDIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        deploy_length = mcbParam.deployLen;
        SetAction(COMMAND_REEL_OUT); // will be ignored if wrong mode
        break;
    case DEPLOYv:
        deploy_velocity = mcbParam.deployVel;
        break;
    case DEPLOYa:
        mcbComm.TX_Out_Acc(mcbParam.deployAcc); // todo: verification + ack
        break;
    case RETRACTx:
        retract_length = mcbParam.retractLen;
        SetAction(COMMAND_REEL_IN); // will be ignored if wrong mode
        break;
    case RETRACTv:
        retract_velocity = mcbParam.retractVel;
        break;
    case RETRACTa:
        mcbComm.TX_In_Acc(mcbParam.retractAcc); // todo: verification + ack
        break;
    case DOCKx:
        dock_length = mcbParam.dockLen;
        SetAction(COMMAND_DOCK); // will be ignored if wrong mode
        break;
    case DOCKv:
        dock_velocity = mcbParam.dockVel;
        break;
    case DOCKa:
        mcbComm.TX_Dock_Acc(mcbParam.dockAcc); // todo: verification + ack
        break;
    case FULLRETRACT:
        // todo: determine implementation
        break;
    case CANCELMOTION:
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(COMMAND_MOTION_STOP);
        break;
    case ZEROREEL:
        mcbComm.TX_ASCII(MCB_ZERO_REEL); // todo: verification + ack
        break;
    case TEMPLIMITS:
        if (!mcbComm.TX_Temp_Limits(mcbParam.tempLimits[0],mcbParam.tempLimits[1],mcbParam.tempLimits[2],mcbParam.tempLimits[3],mcbParam.tempLimits[4],mcbParam.tempLimits[5])) {
            ZephyrLogWarn("Error sending temperature limits to MCB");
        }
        break;
    case TORQUELIMITS:
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            ZephyrLogWarn("Error sending torque limits to MCB");
        }
        break;
    case CURRLIMITS:
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            ZephyrLogWarn("Error sending curr limits to MCB");
        }
        break;
    // Non-MCB Telecommands -------------------------------
    case GOFTRFLIGHT:
        flight_submode = FTR_SUBMODE;
        HKcounter = millis();
        inst_substate = MODE_ENTRY;
        scheduler.ClearSchedule();
        ZephyrLogFine("Set flight sub-mode to FTR");
        break;
    case GOMCBFLIGHT:
        flight_submode = MCB_SUBMODE;
        inst_substate = MODE_ENTRY;
        ftr.poweroff();
        break;
    case FTRONTIME:
        Measure_Period = dibParam.ftrOnTime;
        log_debug("TC = FTRONTME");
        break;
    case FTRCYCLETIME:
        Idle_Period = dibParam.ftrCycleTime;
        log_debug("TC = FTRCYCLETIME");
        break;
    case SETDIBHKPERIOD:
        HK_Loop = dibParam.hkPeriod;
        log_debug("TC = SETDIBHKPERIOD");
        break;
    case FTRSTATUSLIMIT:
        Stat_Limit = dibParam.statusLimit;
        log_debug("TC = FTRSTATUSLIMIT");
        break;
    case RAMANLEN:
        RamanLength = dibParam.ramanScanLength;
        log_debug("TC = RAMANLEN");
        break;
    case FTRRESOLUT:
        Optical_Resolution = dibParam.ftrSpatialRes;
        log_debug("TC = FTRRESOLUT");
        break;
    case SETMEASURETYPE:  
        //inst_substate = MODE_ENTRY;
        scheduler.ClearSchedule();
        ZephyrLogFine("Resetting FTR loop");
        inst_substate = FTR_GPS_WAIT;

        //measure_type = dibParam.ftrMeasureType;
        //log_debug("TC = SETMEASURETYPE");
          //  if(measure_type == BURST){
           //     Burst_Limit = dibParam.ftrBurstLim;
               // log_debug("Burst Limit Set");
           // }
        break;
    case RESETEFU:

        if(efucomm.TX_ResetInst()){
            ZephyrLogFine("Sending EFU reset");
            CommID = (int)RESETEFU;
            //put ACK functionality here
        }
        else{
            ZephyrLogWarn("EFU reset not sent");
            CommID = 999;
        }
        break;

    case EFUHEATPOINT:  
        if(efucomm.TX_SetHeaters(dibParam.efuHeaterTemp[0], dibParam.efuHeaterTemp[1])){
            CommID = (int)EFUHEATPOINT;  
            CommVal = (int)dibParam.efuHeaterTemp[0] + (int)dibParam.efuHeaterTemp[1];
        }
        else{
            ZephyrLogWarn("EFU Heat Set Point Not Sent");
            CommID = 999;
        }   
        break;

    case EFUCYCLETIME:
        if(efucomm.TX_SetDataRate(dibParam.efuTsenRate)){
            CommID = (int)EFUCYCLETIME;
            CommVal = (int)dibParam.efuTsenRate;
        }
        else{
            ZephyrLogWarn("EFU TSEN TX Time Not Sent");
            CommID = 999;
        }
        break;

    case EFUHKTIME:
        if(efucomm.TX_SetTxRate(dibParam.efuHKRate)){
            CommID = (int)EFUHKTIME;
            CommVal = (int)dibParam.efuHKRate;
        }
        else{
            ZephyrLogWarn("EFU HK TX Time Not Sent");
            CommID = 999;
        }
        break;

    case EFUTXPWR:
        if(efucomm.TX_SetTXPower(dibParam.efuPower)){
            CommID = (int)EFUTXPWR;
            CommVal = (int)dibParam.efuPower;
        }
        else{
            ZephyrLogWarn("EFU HK TX Time Not Sent");
            CommID = 999;
        }
        break;

    case EXITERROR:
        SetAction(EXIT_ERROR_STATE);
        break;
    default:
        log_error("Unknown TC received");
        break;
    }
    //return true;
}

// --------------------------------------------------------
// Action handler and action flag helper functions
// --------------------------------------------------------

void StratoDIB::ActionHandler(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return;
    }

    // set the flag and reset the stale count
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

bool StratoDIB::CheckAction(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return false;
    }

    // check and clear the flag if it is set, return the value
    if (action_flags[action].flag_value) {
        action_flags[action].flag_value = false;
        action_flags[action].stale_count = 0;
        return true;
    } else {
        return false;
    }
}

void StratoDIB::SetAction(uint8_t action)
{
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

void StratoDIB::WatchFlags()
{
    // monitor for and clear stale flags
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (action_flags[i].flag_value) {
            action_flags[i].stale_count++;
            if (action_flags[i].stale_count >= FLAG_STALE) {
                action_flags[i].flag_value = false;
                action_flags[i].stale_count = 0;
            }
        }
    }
}

// --------------------------------------------------------
// MCB Message Router + Handlers
// --------------------------------------------------------

void StratoDIB::RunMCBRouter()
{
    
    SerialMessage_t rx_msg = mcbComm.RX();

    while (NO_MESSAGE != rx_msg) {
        if (ASCII_MESSAGE == rx_msg) {
            HandleMCBASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandleMCBAck();
        } else if (BIN_MESSAGE == rx_msg) {
           HandleMCBBin();
        } else {
            log_error("Unknown message type from MCB");
        }

        rx_msg = mcbComm.RX();
    }
}

void StratoDIB::HandleMCBASCII()
{
    switch (mcbComm.ascii_rx.msg_id) {
    case MCB_MOTION_FINISHED:
        log_nominal("MCB motion finished"); // state machine will report to Zephyr
        mcb_motion_ongoing = false;
        break;
    case MCB_ERROR:
        if (mcbComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    case MCB_MOTION_FAULT:
        if (mcbComm.RX_Motion_Fault(motion_fault, motion_fault+1, motion_fault+2, motion_fault+3,
                                    motion_fault+4, motion_fault+5, motion_fault+6, motion_fault+7)) {
            mcb_motion_ongoing = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "MCB Fault: %x,%x,%x,%x,%x,%x,%x,%x", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            SendMCBTM(CRIT, log_array);
            inst_substate = MODE_ERROR;
        } else {
            mcb_motion_ongoing = false;
            SendMCBTM(CRIT, "MCB Fault: error receiving parameters");
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown MCB ASCII message received");
        break;
    }
}

void StratoDIB::HandleMCBAck()
{
    switch (mcbComm.ack_id) {
    case MCB_GO_LOW_POWER:
        log_nominal("MCB in low power");
        mcb_low_power = true;
        break;
    case MCB_REEL_IN:
        if (MOTION_REEL_IN == mcb_motion) NoteProfileStart();
        break;
    case MCB_REEL_OUT:
        if (MOTION_REEL_OUT == mcb_motion) NoteProfileStart();
        break;
    case MCB_DOCK:
        if (MOTION_DOCK == mcb_motion) NoteProfileStart();
        break;
    case MCB_IN_ACC:
        ZephyrLogFine("MCB acked retract acc");
        break;
    case MCB_OUT_ACC:
        ZephyrLogFine("MCB acked deploy acc");
        break;
    case MCB_DOCK_ACC:
        ZephyrLogFine("MCB acked dock acc");
        break;
    case MCB_ZERO_REEL:
        ZephyrLogFine("MCB acked zero reel");
        break;
    case MCB_TEMP_LIMITS:
        ZephyrLogFine("MCB acked temp limits");
        break;
    case MCB_TORQUE_LIMITS:
        ZephyrLogFine("MCB acked torque limits");
        break;
    case MCB_CURR_LIMITS:
        ZephyrLogFine("MCB acked curr limits");
        break;
    default:
        log_error("Unknown MCB ack received");
        break;
    }
}

void StratoDIB::HandleMCBBin()
{
    float reel_pos = 0;
    uint16_t reel_pos_index = 21; // todo: don't hard-code this

    switch (mcbComm.binary_rx.bin_id) {
    case MCB_MOTION_TM:
        if (BufferGetFloat(&reel_pos, mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length, &reel_pos_index)) {
            snprintf(log_array, 101, "Reel position: %ld", (int32_t) reel_pos);
            log_nominal(log_array);
            //ZephyrLogFine(log_array);
        } else {
            log_nominal("Recieved MCB bin: unable to read position");
        }
        AddMCBTM();
        break;
    default:
        log_error("Unknown MCB bin received");
    }
}

// --------------------------------------------------------
// Profile helpers
// --------------------------------------------------------

bool StratoDIB::StartMCBMotion()
{
    bool success = false;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        snprintf(log_array, LOG_ARRAY_SIZE, "Retracting %0.1f revs", retract_length);
        success = mcbComm.TX_Reel_In(retract_length, retract_velocity); // todo: verification
        break;
    case MOTION_REEL_OUT:
        snprintf(log_array, LOG_ARRAY_SIZE, "Deploying %0.1f revs", deploy_length);
        success = mcbComm.TX_Reel_Out(deploy_length, deploy_velocity); // todo: verification
        break;
    case MOTION_DOCK:
        snprintf(log_array, LOG_ARRAY_SIZE, "Docking %0.1f revs", dock_length);
        success = mcbComm.TX_Dock(dock_length, dock_velocity); // todo: verification
        break;
    case MOTION_UNDOCK:
    default:
        mcb_motion = NO_MOTION;
        log_error("Unknown motion type to start");
        return false;
    }

    ZephyrLogFine(log_array);

    return success;
}

void StratoDIB::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // sync byte
    if (!zephyrTX.addTm((uint8_t) 0xA5)) {
        log_error("unable to add sync byte to MCB TM buffer");
        return;
    }

    // tenths of seconds since start
    if (!zephyrTX.addTm((uint16_t) ((millis() - profile_start) / 100))) {
        log_error("unable to add seconds bytes to MCB TM buffer");
        return;
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        if (!zephyrTX.addTm(mcbComm.binary_rx.bin_buffer[i])) {
            log_error("unable to add data byte to MCB TM buffer");
            return;
        }
    }
}

void StratoDIB::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();
    zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data

    // MCB TM Header
    zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
    // add to header: profile type, auto vs. manual, auto trigger?
}

void StratoDIB::SendMCBTM(StateFlag_t state_flag, String message)
{
    // use only the first flag to report the motion
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateFlagValue(1, state_flag);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}

// --------------------------------------------------------
// Hardware Operation Functions
// --------------------------------------------------------


void StratoDIB::FTRStatusReport(uint8_t status){

    if((status == 0x27) || (status == 0x00) || (status == 0x07)){
        ftr_status = FTR_NOTREADY;
    }

    else if((status == 0x57) || (status == 0x67) || (status == 0x17)) {
        ftr_status = FTR_READY;
    }

    else {
        ftr_status = FTR_ERROR;
    }
}

void  StratoDIB::resetLtcSpi() {
        SPI0_SR |= SPI_DISABLE;
        SPI0_CTAR0 = 0x38004005;
        SPI0_SR &= ~(SPI_DISABLE);
}

void  StratoDIB::LTCSetup(){

    log_debug("LTC Configured");
    ltcManager.channel_assignments[FOTS1_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[FOTS2_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[DCDC_THERM_CH]   = THERMISTOR_44006;
    ltcManager.channel_assignments[SPARE_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[OAT_PRT1_RTD_CH] = RTD_PT_100;
    ltcManager.channel_assignments[OAT_PRT2_RTD_CH] = RTD_PT_100;
    ltcManager.InitializeAndConfigure();
    digitalWrite(LTC_TEMP_RESET_PIN, HIGH);

    ltcManager.connect();
}

void StratoDIB::ReadFullTemps() {
  resetLtcSpi();
  digitalWrite(14, HIGH);
  noInterrupts();
  ltcManager.WakeUp();
  uint16_t ret = ltcManager.CheckStatusReg();

  if ((ret == 0) || (ret == 0xFF)) {
    //Serial.println("Error reading status register, resetting LTC");
    ltcManager.channel_assignments[FOTS1_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[FOTS2_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[DCDC_THERM_CH]   = THERMISTOR_44006;
    ltcManager.channel_assignments[SPARE_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[OAT_PRT1_RTD_CH] = RTD_PT_100;
    ltcManager.channel_assignments[OAT_PRT2_RTD_CH] = RTD_PT_100;
    ltcManager.InitializeAndConfigure();
    ret = ltcManager.CheckStatusReg();
    if ((ret == 0 || ret == 0xFF)) {
      log_error("LTC Reset failed");
      //return 1;
    }
  }

  FOTS1Therm = ltcManager.MeasureChannel(FOTS1_THERM_CH);
  FOTS2Therm = ltcManager.MeasureChannel(FOTS2_THERM_CH);
  DC_DC_Therm = ltcManager.MeasureChannel(DCDC_THERM_CH);
  SpareTherm = ltcManager.MeasureChannel(SPARE_THERM_CH);
  RTD1 = ltcManager.MeasureChannel(OAT_PRT1_RTD_CH);
  RTD2 = ltcManager.MeasureChannel(OAT_PRT2_RTD_CH);
  interrupts();

  //return 0;
}

void StratoDIB::ReadVoltages(){

  float val = analogRead(VMON_15V);

  //vmon_channels[curr_channel].last_voltage = VREF * (raw / MAX_ADC_READ) / vmon_channels[curr_channel].voltage_divider;

  V_Zephyr = (val*(3.0/4096.0))*((10000.0+1130.0)/1130.0);
  
  //V_3v3 = (val*(3.3/4096.0))*((10000.0+10000.0)/10000.0);
 
  val = analogRead(VMON_12V);
  V_12FTR = (val*(3.0/4096.0))*((10000.0+1500.0)/1500.0);
}

void StratoDIB::ReadInstCurrent(){
    //imon_channels[curr_channel].last_current = ((1.65-(VREF * raw / MAX_ADC_READ))/.044); 
    //44 mv/A and minimum V=1.65 based on ACS&1240LLCBTR-030B3 datasheet 

    float raw = analogRead(IMON_INST);   
    C_Zephyr = ((1.65-(3.0 * raw /4096.0))/.044);

}

void StratoDIB::ReadPressure(){

    sensor.readSensor();
    P_mbar = sensor.pressure();
    P_tempC = sensor.temperature();


}

// --------------------------------------------------------
// EFu Message Router + Handlers
// --------------------------------------------------------

void StratoDIB::RunEFURouter(){

    int RXBytes = LoRa.parsePacket();
    if(RXBytes != 0)
    {
        RSSI_val = LoRa.packetRssi();
        uint8_t EFU_msg = efucomm.RX();
    
        while (L_NO_MESSAGE != EFU_msg) {
            if (L_ASCII_MESSAGE == EFU_msg) {
                
                if(EFU_Ready){ // use EFU_Ready flag to know when to retreive and parse EFU messages.
                    HandleEFUMess();
                 }
            } else if (L_ACK_MESSAGE == EFU_msg) {

                
                char buffer [100];
                snprintf ( buffer, 100, "RX EFU ACK for ID: %d and value %d", CommID, CommVal);
                ZephyrLogFine(buffer);

                // String ackmess;
                // ackmess.concat('RX EFU ACK for ID: ');
                // ackmess.concat(String(CommID));
                // ackmess.concat(' and value:');
                // ackmess.concat(String(CommVal));
                // ZephyrLogFine(ackmess.c_str());

                CommVal = 0;
                CommID = 999;

            } else if (L_BIN_MESSAGE == EFU_msg) {
    
            } else {
                log_error("Unknown message type from EFU");
            }

            EFU_msg = L_NO_MESSAGE;
 
        }
    }

}

void StratoDIB::HandleEFUMess(){

    int8_t tmp1;

    switch (efucomm.ascii_rx.msg_id){
        case EFU_TSEN:
            tmp1 = efucomm.RX_Data(&EFU_Time_RX, &EFU_Lat_RX, &EFU_Lon_RX, &EFU_Alt_RX, &EFU_TsenT_RX, &EFU_TsenP_RX, &EFU_TsenTP_RX);
            if(tmp1)
            {
                Serial.println("Received EFU Data Packet");
                BuildEFUTelem();
            }
            break;
        case EFU_HK:
            tmp1 = efucomm.RX_HK(&EFU_Time_RX, &EFU_VBatt_RX, &EFU_Vdcdc_RX, &EFU_Vteensy_RX, &EFU_Tbatt_RX, &EFU_Tpcb_RX, &EFU_heat_RX);
            if(tmp1)
            {
                log_debug("Received EFU Houskeeping");
                BuildEFUHouseKeeping();
            }
            break;           
        default:
            efucomm.TX_Ack(efucomm.ascii_rx.msg_id,false);
            break;  
        } 
}

void StratoDIB::BuildEFUTelem(){

    int rows = sizeof(EFUBuff)/sizeof(EFU_TX);
    uint16_t buffersize = sizeof(EFUBuff);
    uint8_t bytes[sizeof(float)];
 
    EFU_TX[0] = (uint16_t) (EFU_Time_RX>>16);
    EFU_TX[1] = (uint16_t) (EFU_Time_RX);  

    *(float*)(bytes) = EFU_Lat_RX; 
    EFU_TX[2] = (uint16_t)((bytes[0] << 8) + bytes[1]); //low bytes (little endian)
    EFU_TX[3] = (uint16_t)((bytes[2] << 8) + bytes[3]);

    *(float*)(bytes) = EFU_Lon_RX; 
    EFU_TX[4] = (uint16_t)((bytes[0] << 8) + bytes[1]); //low bytes (little endian)
    EFU_TX[5] = (uint16_t)((bytes[2] << 8) + bytes[3]);

    EFU_TX[6] = EFU_Alt_RX;

    EFU_TX[7] = EFU_TsenT_RX;

    EFU_TX[8] = (uint16_t) (EFU_TsenP_RX >> 16);
    EFU_TX[9] = (uint16_t) (EFU_TsenP_RX);  

    EFU_TX[10] = (uint16_t) (EFU_TsenTP_RX >> 16);
    EFU_TX[11] = (uint16_t) (EFU_TsenTP_RX); 


    for(int ii = 0; ii < 12; ii++){ //add received EFU ascii data to buffer
       EFUBuff[edex] = EFU_TX[ii]; 
       edex++;
    }

    EFUdataCounter++; 

    if(EFUdataCounter>=rows){ //when buffer is full send TM

        zephyrTX.clearTm();
        TM_ack_flag = NO_ACK;
        XMLHeader(DATAEFU);
        zephyrTX.addTm(EFUBuff, buffersize/2);
        Serial.print("counter num = ");
        Serial.println(EFUdataCounter);
        EFUdataCounter = 0;
        edex = 0;
        zephyrTX.TM();

        if (ACK == TM_ack_flag) {
            zephyrTX.clearTm();
            Serial.println("Successfully sent EFU data telem");
        
        } else if (NAK == TM_ack_flag) {
            // attempt one resend
            zephyrTX.TM();
            zephyrTX.clearTm();
            Serial.println("Successfully resent EFU data telem");
        }

        else
        {
            Serial.println("Telem ACK not working");
        }
        
    }

}


void StratoDIB::BuildEFUHouseKeeping(){

    int hkrows = sizeof(EFUHKBuff)/sizeof(EFU_HKTX);
    uint16_t buffersizeHK = sizeof(EFUHKBuff);
    //uint8_t bytes[sizeof(float)];    

    //uint32_t testtime = 1627997064;
    EFU_HKTX[0] = (uint16_t) (EFU_Time_RX>>16);
    EFU_HKTX[1] = (uint16_t) (EFU_Time_RX);  
    //EFU_HKTX[0] = (uint16_t) (testtime >> 16);
    //EFU_HKTX[1] = (uint16_t) (testtime); 

    //EFU_HKTX[2] = 3851;
    //EFU_HKTX[3] = 6497;
    EFU_HKTX[2] = EFU_VBatt_RX;
    EFU_HKTX[3] = EFU_Vdcdc_RX;
    
    uint8_t Vteensy = EFU_Vteensy_RX/100;
    //uint16_t Vteensy = 33;
    //EFU_heat_RX = 1;
    EFU_HKTX[4] = (uint16_t) Vteensy + (uint16_t)(EFU_heat_RX << 8);

    //uint16_t Tbatt = 2964;
    uint16_t Tbatt = (uint16_t)((EFU_Tbatt_RX+273.15)*10);
    EFU_HKTX[5] = Tbatt;

    //uint16_t Tpcb = 2967;
    uint16_t Tpcb = (uint16_t)((EFU_Tpcb_RX+273.15)*10);
    EFU_HKTX[6] = Tpcb;

    for(int jj = 0; jj < 7; jj++){
       EFUHKBuff[HKdex] = EFU_HKTX[jj]; 
       HKdex++;
    }

    EFUhouseCounter ++;

    if(EFUhouseCounter>=hkrows){

        zephyrTX.clearTm();
        XMLHeader(HOUSEEFU);
        zephyrTX.addTm(EFUHKBuff, buffersizeHK/2);
        Serial.print("counter num = ");
        Serial.println(EFUhouseCounter);  
        EFUhouseCounter = 0;
        HKdex = 0;
        zephyrTX.TM();

        if (ACK == TM_ack_flag) {
            zephyrTX.clearTm();
            Serial.println("Successfully sent EFU HK telem");
        
        } else if (NAK == TM_ack_flag) {
            // attempt one resend
            zephyrTX.TM();
            zephyrTX.clearTm();
            Serial.println("Successfully resent EFU HK telem");
        }      

        else
        {
            Serial.println("Telem ACK not working");
        }
        
    }
}


// ------------------------------------------------------
// Handle FTR3000 Data
//-------------------------------------------------------

void StratoDIB::SaveSingleScan(){

    ReadFullTemps();
    ReadPressure();
    ReadInstCurrent();

    Serial.println("saving scan to SD");

    String SDFileName;
    SDFileName = "TEST_T.txt";  

    datafile = SD_FTR.open(SDFileName.c_str(), FILE_WRITE);
    datafile.print("#,");
    datafile.print((uint32_t) now());
    datafile.print(",");
    datafile.print(FOTS1Therm);
    datafile.print(",");
    datafile.print(FOTS2Therm);
    datafile.print(',');
    datafile.print(DC_DC_Therm);
    datafile.print(',');
    datafile.print(SpareTherm);
    datafile.print(',');
    datafile.print(RTD1);
    datafile.print(',');
    datafile.print(RTD2);
    datafile.print(",");
    datafile.print(P_mbar);
    datafile.print(",");

    for(int i = 0; i<RamanLength; i++){

        datafile.print(Stokes[i]);  //print all data to serial
        datafile.print(",");
        datafile.print(Astokes[i]);
        datafile.print(",");

    }

    datafile.println("");
    datafile.close();  
    //Serial.println("closing SD");

    Serial.print("#,");
    Serial.print((uint32_t) now());
    Serial.print(",");
    Serial.print(FOTS1Therm);
    Serial.print(",");
    Serial.print(FOTS2Therm);
    Serial.print(',');
    Serial.print(DC_DC_Therm);
    Serial.print(',');
    Serial.print(SpareTherm);
    Serial.print(',');
    Serial.print(RTD1);
    Serial.print(',');
    Serial.print(RTD2);
    Serial.print(",");
    Serial.print(P_mbar);
    Serial.print(",");

    for(int i = 0; i<RamanLength; i++){

        Serial.print(Stokes[i]);  //print all data to serial
        Serial.print(",");
        Serial.print(Astokes[i]);
        Serial.print(",");

    }

    Serial.println("");



}

void StratoDIB::XMLHeader(const char messval){

    ReadVoltages();
    ReadInstCurrent();

    String Message = "";
    bool flag1 = true;
    bool flag2 = true;

    /* Check the values for the TM message header */
    //if ((SpareTherm > 60.0) || (SpareTherm < -60.0))
        //flag1 = false;
    if ((FOTS1Therm > 60.0) || (FOTS1Therm < -60.0))
        flag1 = false;
    //if ((FOTS2Therm > 60.0) || (FOTS2Therm < -60.0))
       // flag1 = false;

    /*Check Voltages are in range */
    if ((V_Zephyr > 19.0) || (V_Zephyr < 12.0))
        flag2 = false;
    if((V_12FTR>13.5) || (V_12FTR<11.0))
        flag2 = false;

    //First Field
    if (flag1) {
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateFlagValue(1, WARN);
    }

    int messint = (int)messval;
    Message.concat(messint);
    zephyrTX.setStateDetails(1, Message);
    Message = "";

     // Second Field
     if (flag2) {
         zephyrTX.setStateFlagValue(2, FINE);
     } else {
         zephyrTX.setStateFlagValue(2, WARN);
     }

    Message.concat(now());
    Message.concat(',');
    Message.concat(FOTS1Therm);
    Message.concat(',');
    Message.concat(RTD1);
    Message.concat(',');
    Message.concat(P_mbar);
    Message.concat(',');
    Message.concat(V_Zephyr);
    Message.concat(',');
    Message.concat(V_12FTR);
    Message.concat(',');
    Message.concat(C_Zephyr);
    Message.concat(',');
    Message.concat(RSSI_val);
    zephyrTX.setStateDetails(2, Message);
    Message = "";


}