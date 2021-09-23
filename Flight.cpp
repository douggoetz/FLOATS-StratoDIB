/*
 *  Flight.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements the MCBOATS flight mode.
 */

#include "StratoDIB.h"



// Flight mode states for MCB
enum FLStatesMCB_t : uint8_t {
    MCB_ENTRY = MODE_ENTRY,

    // before anything else
    MCB_GPS_WAIT,

    // MCB motion states
    MCB_IDLE,
    MCB_START_MOTION,
    MCB_VERIFY_MOTION,
    MCB_MONITOR_MOTION,
    MCB_TM_ACK,

    // general off-nominal states in response to following landing states
    MCB_ERROR_LOOP,
    MCB_SHUTDOWN_LOOP,

    // 253 - 255, defined in StratoCore
    MCB_ERROR_LANDING = MODE_ERROR,
    MCB_SHUTDOWN_LANDING = MODE_SHUTDOWN,
    MCB_EXIT = MODE_EXIT
};


// this function is called at the defined rate
//  * when flight mode is entered, it will start in MCB_ENTRY state
//  * it is then up to this function to change state as needed by updating the inst_substate variable
//  * on each loop, whichever substate is set will be perfomed
//  * when the mode is changed by the Zephyr, MCB_EXIT will automatically be set
//  * it is up to the MCB_EXIT logic perform any actions for leaving flight mode
void StratoDIB::FlightMode()
{
    if (FTR_SUBMODE == flight_submode) {
        FlightFTR();
        HousekeepingFTR();
    } else {
        FlightMCB();
    }

    EFU_Ready = true; // allow EFU messages to be collected and EFU TMs to be sent
}

 void StratoDIB::HousekeepingFTR()
 {

    if( (millis()-HKcounter) >= (HK_Loop*1000) ){
    
        log_debug("Sending Housekeeping");
        zephyrTX.clearTm();
        XMLHeader(HOUSEFTR);
        zephyrTX.TM();
        HKcounter = millis();

    }
 }

 void StratoDIB::FlightFTR()
 {
    switch (inst_substate) {
    case FTR_ENTRY:
        // perform setup
        log_nominal("Entering FTR");
        inst_substate = FTR_GPS_WAIT;
        break;

    case FTR_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_nominal("Waiting on GPS time");
        if (time_valid) {
            inst_substate = FTR_ENTER_IDLE;
            log_nominal("FTR Enter Idle");
        }
        break;

    case FTR_ENTER_IDLE:
        // go into a low power idle state
        ftr.poweroff();
        inst_substate = FTR_IDLE;
        scheduler.AddAction(IDLE_EXIT, Idle_Period);
        break;

    case FTR_IDLE:

        if(CheckAction(IDLE_EXIT)){ //when IDLE exit timer expires move to warmup state

            inst_substate = FTR_WARMUP;
            SetAction(POWERON_FTR);
            log_nominal("Enter Warmup State");
        }
        break;

    case FTR_WARMUP:

       if(CheckAction(POWERON_FTR)){
            log_debug("FTR powered on");
            ftr.poweroff();
            delay(100);
            ftr.start();
            scheduler.AddAction(CONFIGURE_FTR, 15);
            EthernetCount = 0;
        }

        if(CheckAction(CONFIGURE_FTR)){
            log_debug("Configuring FTR");

            if(ftr.EthernetConnect()){ //replace else if with else
                
                scheduler.AddAction(CHECK_FTR_STATUS, Status_Loop);
                Stat_Counter = 0;

            }else{
                scheduler.AddAction(CONFIGURE_FTR, 5);
                log_debug("Attempting to connect Ethernet");
                EthernetCount++;
            }

            if(EthernetCount>=10){
                log_debug("Ethernet Time out - resetting FTR");
                ZephyrLogWarn("Ethernet Timeout");
                SetAction(POWERON_FTR);
            }
        }

        if(CheckAction(CHECK_FTR_STATUS)){
            log_debug("Checking Status");
            ftr_status = ENTERSTAT;
            FTRStatusReport(ftr.status());

            switch(ftr_status){
            case FTR_READY: //if status byte shows data ready
                log_nominal("stat ready");
                Stat_Counter = 0;
                SetAction(FTR_SCAN); //do ftr scan since FTR status indicates that the first scan is ready
                scheduler.AddAction(BUILD_TELEM, Measure_Period); //Set timer for when to end the measurement period
                inst_substate = FTR_MEASURE;
                Stokes_Counter = 0; //counter that keeps track of how many total stokes scans have been averaged
                Astokes_Counter = 0;//counter that keeps track of how many total antistokes scans have been averaged
                //Burst_Counter = 0;
                log_debug("Entering Measure State");
                break;

            case FTR_NOTREADY: //if status byte doesn't show data ready
                log_error("stat not ready");
                scheduler.AddAction(CHECK_FTR_STATUS,Status_Loop);
                Stat_Counter ++;
                Serial.println(Stat_Counter);
                break;

            case FTR_ERROR: //if status byte is out of bounds
                log_error("stat error");
                scheduler.AddAction(CHECK_FTR_STATUS,Status_Loop);
                Stat_Counter ++;
                break;

            default:
                log_error("stat unknown"); //if there isn't communication with FTR
                scheduler.AddAction(CHECK_FTR_STATUS,Status_Loop);
                Stat_Counter ++;
                break;
           }

            if(Stat_Counter >= Stat_Limit){ //if status counter exceeds Status limit then restart FTR
                log_error("stat timeout");
                ZephyrLogCrit("FTR status failure");
                SetAction(POWERON_FTR);
            }

        }
        break;

    case FTR_MEASURE:

        if(CheckAction(FTR_SCAN)){

            log_nominal("Enter Measure State");
            ftr.resetFtrSpi();
            FTRStatusReport(ftr.status());

            switch(ftr_status){
            case FTR_READY: //if status byte shows data ready

                if(ftr.status()==0x67){

                    ftr.readRaman(RamanBin);

                    if(ftr.CheckSumValidate(RamanBin)){ //if status is 0x67, see if checksum from measurement is unique

                        log_nominal("Retrieved Raman");
                        Stat_Counter = 0; //reset stat counter since measurement is valid
                        ftr.BintoArray(RamanBin, Stokes, Astokes, RamanLength);
                        ftr.ClearArray(RamanBin, 17000);

                        SaveSingleScan(); // saves scan to SD card


                        //Co-add arrays and keep track of coadd number for each index point
                        Stokes_Counter += ftr.RamanCoAdd(StokesElements, Stokes, StokesAvg, RamanLength);
                        ftr.ClearArray(Stokes, RamanLength);
                        Astokes_Counter += ftr.RamanCoAdd(AstokesElements, Astokes, AStokesAvg, RamanLength);
                        ftr.ClearArray(Astokes, RamanLength);
                        log_nominal("Cleared Raman Arrays");                     
                    }
                }
                break;

            case FTR_NOTREADY:
                Stat_Counter ++;
                break;    
                
            default:
                Stat_Counter ++;
                break;
            }

            if(Stat_Counter >= Stat_Limit){ //if status counter exceeds Status limit then restart FTR
                log_error("stat timeout");
                ZephyrLogCrit("FTR status failure in measuremnt loop");
                inst_substate = FTR_WARMUP;
                SetAction(POWERON_FTR);
            }

            scheduler.AddAction(FTR_SCAN, Scan_Loop);

        }

        if(CheckAction(BUILD_TELEM)){

            log_nominal("Build Telem");
            zephyrTX.clearTm();
            XMLHeader(DATAFTR);
            uint16_t optlen = floor(RamanLength/Optical_Resolution);//*Optical_Resolution;
            Serial.println(optlen);
            zephyrTX.addTm(optlen);
            zephyrTX.addTm((uint8_t)Optical_Resolution);
            zephyrTX.addTm(Measure_Period);

            ftr.RamanAverage(StokesElements, StokesAvg, Stokes, RamanLength, Optical_Resolution);
            
            zephyrTX.addTm(Stokes_Counter);
            zephyrTX.addTm(Stokes, optlen);

            // add each byte of data to the message
            //  for (int i = 0; i < optlen; i++) {
            //      if (!zephyrTX.addTm(Stokes[i])) {
            //      log_error("unable to add Stokes data byte to FTR TM buffer");
            //      //return;
            //      }
            //  }

            ftr.RamanAverage(AstokesElements,AStokesAvg, Astokes, RamanLength, Optical_Resolution);
            zephyrTX.addTm(Astokes_Counter);
            zephyrTX.addTm(Astokes, optlen);

            // TM_ack_flag = NO_ACK;
            // zephyrTX.TM();
            // inst_substate = FTR_TM_ACK;
            // scheduler.AddAction(RESEND_TM, 30);

        //     for (int i = 0; i < optlen; i++) {
        //         if (!zephyrTX.addTm(Astokes[i])) {
        //         log_error("unable to add Astokes data byte to FTR TM buffer");
        //         //return;
        //         }
        //     }

            SetAction(SEND_TELEM);

        }

        if(CheckAction(SEND_TELEM)){

            TM_ack_flag = NO_ACK;
            zephyrTX.TM();
            inst_substate = FTR_TM_ACK;
            scheduler.AddAction(RESEND_TM, 30);

        }
        break;

    case FTR_TM_ACK:

        if (ACK == TM_ack_flag) {
            inst_substate = FTR_ENTER_IDLE;
            zephyrTX.clearTm();
            ftr.ClearArray(Stokes, 4000);
            ftr.ClearArray(Astokes, 4000);
        
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // first resend attempt
            
            Serial.println("Sending second attempt TM");
            inst_substate = FTR_TM_RESEND;
            TM_ack_flag = NO_ACK;
            zephyrTX.TM();
            scheduler.AddAction(RESEND_TM, 30);


           // zephyrTX.TM();
            // zephyrTX.clearTm();
            // //ftr.ClearArray(Stokes, 2000);
            // //ftr.ClearArray(Astokes, 2000);

            // // Serial.println("Rebuilding and sending TM");
            // // zephyrTX.clearTm();
            // XMLHeader(DATAFTR);
            // uint16_t optlen = floor(RamanLength/Optical_Resolution)*Optical_Resolution;
            // Serial.println(optlen);
            // zephyrTX.addTm(optlen);
            // zephyrTX.addTm((uint8_t)Optical_Resolution);
            // zephyrTX.addTm(Measure_Period);
           
            // ftr.RamanAverage(StokesElements, StokesAvg, Stokes, RamanLength, Optical_Resolution);
            // zephyrTX.addTm(Astokes, optlen);
            // zephyrTX.addTm(Stokes_Counter);
            

            // ftr.RamanAverage(AstokesElements,AStokesAvg, Astokes, RamanLength, Optical_Resolution);
            // zephyrTX.addTm(Astokes_Counter);
            // zephyrTX.addTm(Astokes, optlen);
      
        }

        break;

    case FTR_TM_RESEND:

        if (ACK == TM_ack_flag) {
            inst_substate = FTR_ENTER_IDLE;
            zephyrTX.clearTm();
            ftr.ClearArray(Stokes, 4000);
            ftr.ClearArray(Astokes, 4000);
        
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // second resend attempt with TM rebuild
            
            Serial.println("Rebuilding and sending TM");
        
            zephyrTX.clearTm();
            XMLHeader(DATAFTR);
            uint16_t optlen = floor(RamanLength/Optical_Resolution)*Optical_Resolution;
            Serial.println(optlen);
            zephyrTX.addTm(optlen);
            zephyrTX.addTm((uint8_t)Optical_Resolution);
            zephyrTX.addTm(Measure_Period);
           
            ftr.RamanAverage(StokesElements, StokesAvg, Stokes, RamanLength, Optical_Resolution);
            zephyrTX.addTm(Astokes, optlen);
            zephyrTX.addTm(Stokes_Counter);
            

            ftr.RamanAverage(AstokesElements,AStokesAvg, Astokes, RamanLength, Optical_Resolution);
            zephyrTX.addTm(Astokes_Counter);
            zephyrTX.addTm(Astokes, optlen);

            inst_substate = FTR_ENTER_IDLE;
            zephyrTX.TM();
            zephyrTX.clearTm();
            ftr.ClearArray(Stokes, 4000);
            ftr.ClearArray(Astokes, 4000);
        }
        break;
    case FTR_ERROR_LANDING:
        log_error("Landed in flight error");
        ftr.poweroff();
        inst_substate = FTR_ERROR_LOOP;
        break;
    case FTR_ERROR_LOOP:
        log_debug("FTR error loop");
        ftr.poweroff();

        if (CheckAction(EXIT_ERROR_STATE)) {
            log_nominal("Leaving flight error loop");
            inst_substate = FTR_GPS_WAIT;
        }
        break;
    case FTR_SHUTDOWN_LANDING:
        // prep for shutdown
        log_nominal("Shutdown warning received in FTR");
        ftr.poweroff();
        inst_substate = FTR_SHUTDOWN_LOOP;
        break;
    case FTR_SHUTDOWN_LOOP:
        ftr.poweroff();
        break;
    case FTR_EXIT:
        // perform cleanup
        log_nominal("Exiting Flight from FTR");
        ftr.poweroff();
        break;
    default:
        log_error("Unknown flight FTR state");
        ftr.poweroff();
        inst_substate = FTR_GPS_WAIT;
        break;
    }
}

void StratoDIB::FlightMCB()
{
    switch (inst_substate) {
    case MCB_ENTRY:
        // perform setup
        log_nominal("Entering MCB");
        inst_substate = MCB_GPS_WAIT;
        break;
    case MCB_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_debug("Waiting on GPS time");
        if (time_valid) {
            inst_substate = MCB_IDLE;
        }
        break;
    case MCB_IDLE:
        log_debug("MCB Idle");
        if (CheckAction(COMMAND_REEL_IN)) {
            mcb_motion = MOTION_REEL_IN;
            inst_substate = MCB_START_MOTION;
            resend_attempted = false;
        } else if (CheckAction(COMMAND_REEL_OUT)) {
            mcb_motion = MOTION_REEL_OUT;
            inst_substate = MCB_START_MOTION;
            resend_attempted = false;
        } else if (CheckAction(COMMAND_DOCK)) {
            mcb_motion = MOTION_DOCK;
            inst_substate = MCB_START_MOTION;
            resend_attempted = false;
        }
        break;
    case MCB_START_MOTION:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = MCB_ERROR_LANDING;
        }

        if (StartMCBMotion()) {
            inst_substate = MCB_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = MCB_ERROR_LANDING;
        }
        break;
    case MCB_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            inst_substate = MCB_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = MCB_START_MOTION;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = MCB_ERROR_LANDING;
            }
        }
        break;
    case MCB_MONITOR_MOTION:
        if (CheckAction(COMMAND_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            inst_substate = MCB_IDLE;
            break;
        }

        if (!mcb_motion_ongoing) {
            SendMCBTM(FINE, "Finished commanded motion");
            inst_substate = MCB_TM_ACK;
            scheduler.AddAction(RESEND_TM, 10);
        }
        break;
    case MCB_TM_ACK:
        if (ACK == TM_ack_flag) {
            inst_substate = MCB_IDLE;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            zephyrTX.TM();
            inst_substate = MCB_IDLE;
        }
        break;
    case MCB_ERROR_LANDING:
        log_error("Landed in flight error");
        mcb_motion_ongoing = false;
        mcb_motion = NO_MOTION;
        resend_attempted = false;
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = MCB_ERROR_LOOP;
        break;
    case MCB_ERROR_LOOP:
        log_debug("MCB error loop");
        if (!mcb_low_power && CheckAction(RESEND_MCB_LP)) {
            scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
            mcbComm.TX_ASCII(MCB_GO_LOW_POWER); // just constantly send
        }

        if (CheckAction(EXIT_ERROR_STATE)) {
            log_nominal("Leaving flight error loop");
            inst_substate = MCB_ENTRY;
        }
        break;
    case MCB_SHUTDOWN_LANDING:
        // prep for shutdown
        log_nominal("Shutdown warning received in MCB");
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = MCB_SHUTDOWN_LOOP;
        break;
    case MCB_SHUTDOWN_LOOP:
        break;
    case MCB_EXIT:
        // perform cleanup
        log_nominal("Exiting Flight from MCB");
        break;
    default:
        log_error("Unknown flight MCB state");
        inst_substate = MCB_ENTRY;
        break;
    }
}

