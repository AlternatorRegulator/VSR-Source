//      OSEnergy_CAN.cpp
//
//      Copyright (c) 2016, 2017, 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
//                                                                  http://smartdcgenerator.blogspot.com/
//
//
//
//              This program is free software: you can redistribute it and/or modify
//              it under the terms of the GNU General Public License as published by
//              the Free Software Foundation, either version 3 of the License, or
//              (at your option) any later version.
//
//              This program is distributed in the hope that it will be useful,
//              but WITHOUT ANY WARRANTY; without even the implied warranty of
//              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//              GNU General Public License for more details.
//
//              You should have received a copy of the GNU General Public License
//              along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//


#include "Config.h"
#ifdef SYSTEMCAN                                            // To allow for old and new targets, nothing else happens if we are not compiling for the new CAN board.

#include "OSEnergy_CAN.h"
#include "Sensors.h"
#include "Flash.h"
#include <math.h> 






                                // --------  DEVICE SPECIFIC SELECTIONS
                                //          This source supports several different device types, selected by one of the following #defines:
                                //              #define OSE_ALTERNATOR           
                                //              #define OSE_GENERATOR            
                                //              #define OSE_SOLAR
                                //              #define OSE_BMS
                                //              #define OSE_BATMON
                                //
                                //          The following section contains specific configuraiton information assocated with each device type.




#if  defined (CPU_AVRCAN)
        #include <NMEA2000_CAN.h>                                            // https://github.com/ttlappalainen
   
   
#elif defined (CPU_STM32)
        #include <N2kMsg.h>                                                 // Auto-select does not work for cubeMX, need to bring in includes direcetly.
        #include <NMEA2000.h>


        
        //----  CAN / NMEA2000 linkages.
        //      Need to do them manualy, as the cubeMX does not have auto integration into Timo's lib


        tNMEA2000 &NMEA2000=*(new tNMEA2000_cubeMX());
        tcubeMXStream serStream;

        //*****************************************************************************
        tNMEA2000_cubeMX::tNMEA2000_cubeMX() : tNMEA2000()
        {
        }
        
         
        
#endif



                                
#if defined (OSE_ALTERNATOR)
    #include "Alternator.h"
    
    tCCS canConfig = {                    //  Details for an ALTERNATOR REGUALTOR
            0,                            // .BI_OVERRIDE            --> Battery Instance attached to.  0=use DIP switches, override with $CCN command
            1,                            // .DEVICE_INSTANCE        --> Default 'Charger' instance.  Override with $CCN
            70,                           // .DEVICE_PRIORITY        --> Default 'Device Ranking' - 70, Below AC powered chargers.  Override with $CCN command
            true,                         // .CONSIDER_MASTER        --> Default, is no one else steps up to the plate - shall we try to be master?  Override with $CCN
            false,                        // .SHUNT_AT_BAT           --> Until user explicitly tells us otherwise, we need to assume the shunt is NOT connected to the battery when we are the RBM.
            RVCDCbt_Unknown,              // .BATTERY_TYPE           --> Default, we do not know unless the user tells us.
            true,                         // .ENABLE_OSE             --> Default, push out OSEnergy (RV-C) messages (This is NEEDED to support remote instrumentation, prioritization, etc.)  Override with $CCN
            true,                         // .ENABLE_NMEA2000        --> Default, push out NMEA-2000 messages.  Override with $CCN
            false,                        // .ENABLE_NMEA2000_RAT    --> Default, do not look for a NMEA2000 device to remotely supply battery amperage and temperature - and we will send out PGN: 127506
            0,                            // .ENGINE_INSTANCE        --> Default = 1st engine in boat.  (Used with N2K PGN: 127488L and J1939 RPMs  PGN:61444L)
            128                           // .LAST_CAN_ID            --> Default 128 - referred starting range for "Power Components"  in RV-C spec
        };

            
        //--  Specific information used by NMEA2000.SetDeviceInformation()
        #define SERIAL_NUMBER      1                                    // Unique number. Use e.g. Serial number.  (Well, we will just always be #1)
        #define DEVICE_FUNCTON   141                                    // Device function=Alternator. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define DEVICE_CLASS      50                                    // Device class=Propulsion.    See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define MANUF_CODE      1401                                    // Manuf Code: Just chose free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf




#elif defined (OSE_GENERATOR)
   #include "Alternator.h"                                             

    tCCS canConfig = {                    //  Details for an DC GENERATOR REGUALTOR as used in the DC Generator
            0,                            // .BI_OVERRIDE            --> Battery Instance attached to.  0=use DIP switches, override with $CCN command
            1,                            // .DEVICE_INSTANCE        --> Default 'Charger' instance.  Override with $CCN
            60,                           // .DEVICE_PRIORITY        --> Default 'Device Ranking' - 60, Below AC powered chargers and alternators.  Override with $CCN command
            false,                        // .CONSIDER_MASTER        --> Default, is no one else steps up to the plate - shall we try to be master?  Override with $CCN
            false,                        // .SHUNT_AT_BAT           --> Until user explicitly tells us otherwise, we need to assume the shunt is NOT connected to the battery when we are the RBM.
            RVCDCbt_Unknown,              // .BATTERY_TYPE           --> Default, we do not know unless the user tells us.
            true,                         // .ENABLE_OSE             --> Default, push out OSEnergy (RV-C) messages (This is NEEDED to support remote instrumentation, prioritization, etc.)  Override with $CCN
            true,                         // .ENABLE_NMEA2000        --> Default, push out NMEA-2000 messages.  Override with $CCN
            false,                        // .ENABLE_NMEA2000_RAT    --> Default, do not look for a NMEA2000 device to remotely supply battery amperage and temperature - and we will send out PGN: 127506
            10,                           // .ENGINE_INSTANCE        --> Default = picking a high engine number, hope it is free for this 'generator'in boat.  (Used with N2K PGN: 127488L and J1939 RPMs  PGN:61444L)
            128                           // .LAST_CAN_ID            --> Default 128 - referred starting range for "Power Components"  in RV-C spec
        };

            
        //--  Specific information used by NMEA2000.SetDeviceInformation()
        #define SERIAL_NUMBER      1                                    // Unique number. Use e.g. Serial number.  (Well, we will just always be #1)
        #define DEVICE_FUNCTON   141                                    // Device function=DC Generator. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define DEVICE_CLASS      35                                    // Device class=Generator.    See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define MANUF_CODE      1401                                    // Manuf Code: Just chose free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf



        tRVCGenCmd CANGenRequest = RVCGst_Unknown;                     // Generator command request sent in via the CAN to start/stop the generator

     





#elif defined (OSE_BATMON)
    #include "Battery.h"
    

    tCCS canConfig = {
            0,                            // .BI_OVERRIDE            --> Battery Instance attached to.  0=use DIP switches, override with $CCN command
            1,                            // .DEVICE_INSTANCE        --> Default 'Charger' instance.  Override with $CCN
            122,                          // .DEVICE_PRIORITY        --> Default 'Device Ranking' - 122, lower end of the BMS scale.  Override with $CCN command
            true,                         // .CONSIDER_MASTER        --> Default, is no one else steps up to the plate - shall we try to be master?  Override with $CCN
            false,                        // .SHUNT_AT_BAT           --> Until user explicitly tells us otherwise, we need to assume the shunt is NOT connected to the battery when we are the RBM.
            RVCDCbt_Unknown,              // .BATTERY_TYPE           --> Default, we do not know unless the user tells us.
            true,                         // .ENABLE_OSE             --> Default, push out OSEnergy (RV-C) messages (This is NEEDED to support remote instrumentation, prioritization, etc.)  Override with $CCN
            true,                         // .ENABLE_NMEA2000        --> Default, push out NMEA-2000 messages.  Override with $CCN
            false,                        // .ENABLE_NMEA2000_RAT    --> We are BatMon! We do not look for a NMEA2000 device to remotely supply battery amperage and temperature - and we will send out PGN: 127506
            0,                            // .ENGINE_INSTANCE --> Default = 1st engine in boat.  (Used with N2K PGN: 127488L  - n/a for BatMon)
            128                           // .LAST_CAN_ID            --> Default 128 - referred starting range for "Power Components"  in RV-C spec
            };



           
        //--  Specific information used by NMEA2000.SetDeviceInformation()
        #define SERIAL_NUMBER      1                                    // Unique number. Use e.g. Serial number.  (Well, we will just always be #1)
        #define DEVICE_FUNCTON   170                                    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define DEVICE_CLASS      35                                    // Device class=Electrical.     See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define MANUF_CODE      1401                                    // Manuf Code: Just chose free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf

    




#elif defined (OSE_DISPLAY1)
  
   

    tCCS canConfig = {                    //  Details for an ALTERNATOR REGUALTOR as used in the DC Generator
            0,                            // .BI_OVERRIDE            --> Battery Instance attached to.  0=use DIP switches, override with $CCN command
            1,                            // .DEVICE_INSTANCE        --> Default 'Charger' instance.  Override with $CCN
            1,                            // .DEVICE_PRIORITY        --> Default 'Device Ranking' - 1, Display has no prioity  Override with $CCN command
            false,                        // .CONSIDER_MASTER        --> Default, is no one else steps up to the plate - shall we try to be master?  Override with $CCN
            false,                        // .SHUNT_AT_BAT           --> Until user explicitly tells us otherwise, we need to assume the shunt is NOT connected to the battery when we are the RBM.
            RVCDCbt_Unknown,              // .BATTERY_TYPE           --> Default, we do not know unless the user tells us.
            true,                         // .ENABLE_OSE             --> Default, push out OSEnergy (RV-C) messages (This is NEEDED to support remote instrumentation, prioritization, etc.)  Override with $CCN
            false,                        // .ENABLE_NMEA2000        --> Default, Dsiplay has no NMEA2000 messages to push out.  (Perhaps later if a Bridge-type function is added).  Override with $CCN
            false,                        // .ENABLE_NMEA2000_RAT    --> Default, do not look for a NMEA2000 device to remotely supply battery amperage and temperature - and we will send out PGN: 127506
              0,                          // .ENGINE_INSTANCE        --> Default = picking a high engine number, hope it is free for this 'generator'in boat.  (Used with N2K PGN: 127488L and J1939 RPMs  PGN:61444L)
            128                           // .LAST_CAN_ID            --> Default 128 - referred starting range for "Power Components"  in RV-C spec
        };

            
        //--  Specific information used by NMEA2000.SetDeviceInformation()
        #define SERIAL_NUMBER      1                                    // Unique number. Use e.g. Serial number.  (Well, we will just always be #1)
        #define DEVICE_FUNCTON   130                                    // Device function=Display. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define DEVICE_CLASS     120                                    // Device class=Dsiplay.    See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        #define MANUF_CODE      1401                                    // Manuf Code: Just chose free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf



       
            
            
#else
        #error Unsupported OSEnergy Device Type in OSEnergy_CAN.h 
#endif 




    


uint8_t         batteryInstance;                                        // What is the CAN Battery Instance we are associated with?  
float           CAN_RBM_amps;                                           // Remote instrumentation of the battery
uint32_t        CAN_RBM_ampsRefreshed;                                  // Time (millis) when all the Remote Battery current was received.
float           CAN_RBM_volts;
float           CAN_RBM_voltsOffset;                                    // Snapshot copy of measuredAltVolts at the time the remote battery voltage was received.
int             CAN_RBM_voPWMvalue;                                     // Snapshot of PWM value at time the remote voltage was received.  (Used to adjust the offset voltage based on changes in alternator output)
uint16_t        CAN_RBM_dVdT;                                           // in mV/S
uint32_t        CAN_RBM_voltsRefreshed;                                 // Time (millis) when all the Remote Battery Voltage was received.
int             CAN_RBM_temp;                                           // Have we been sent a temperature (Deg-C)?
uint32_t        CAN_RBM_tempRefreshed;                                  // Time (millis) the Remote Battery temperature was received.
bool            ignoringRBM = false;                                    // Has the Remote Battery Master sent us something so unbelievable that we should ignore it??  
                                                                        // (Example, too large a deltaV between locally measured Voltage and the reported remote battery voltage?)  
                                                                        
uint32_t        CAN_RAT2000_lastReceived = 0UL;                         // When did we last receive a NMEA-2000 Remote battery Amp and Temp message?
float           CAN_RAT2000_amps;                                       // What was the reported battery Amperage?
int             CAN_RAT2000_temp;                                       // And temperature?
                                                                     

float           CAN_RBM_desiredVolts;                                   // Master coordinated charge points we are to follow.
float           CAN_RBM_desiredAmps;
tRVCBatChrgMode CAN_RBM_desiredChargeState;                             // What mode does the remote master want us to use?  If 'undefined', they are only sending status (Volts, amps, temp).
uint8_t         CAN_RBM_sourceID;                                       // Who is it we think is our CAN Master?   Will = 0 if we are not currently linked to anyone valid.
bool            CAN_weAreRBM;                                           // Is it us??
uint8_t         CAN_RBM_messages;                                       // And which messages has this master sent us?  (Do not validate a remote master until they have sent
                                                                        // us all the critical battery information - ref #define FLAG_DCx's ).
uint32_t        CAN_LPCS_lastReceived = 0UL;                            // Of all the known charging active sources - when was the last time we 'heard from' one that was lower priority then us?
uint32_t        CAN_EPCS_lastReceived = 0UL;                            // Equal to us?   
uint32_t        CAN_HPUUCS_lastReceived = 0UL;                          // And likewise, when did we hear from someone 'higher' then us which was currently not producing its max output
int             average_EPC_utilization;                                // Noted average utilization of all of equal priority charging sourced on the same battery

uint8_t         CAN_ASCII_source = 0;                                   // If we are receiving ASCII characters via the CAN, this is the ID of who sent them.  =0 if ASCII CAN comms is inactive

uint16_t        J1939_Engine_RPMs;                                      // If we are configured to supoort J1939, and are attached to an engine which sends out J1939 messages we may pick up the RPMs
uint32_t        J1939_Engine_RPM_received = 0UL;                        //  If we do, this is the last time we recevied an RPM message.  Note this can also be used as a flag to indicate if we EVER
                                                                        // have seend J1939 engine RPMs, to make perhaps decisions about putting regulator into sleep mode if the engien has stopped.



   //----      Local Variables
 uint8_t        CAN_ASCII_reply;                                        //  Place to stash who made the request to use, we will reply back to them...
 uint32_t       CAN_RBM_lastReceived;                                   // When did the current CAN_RBM_sourceID device last talk to us?
 uint32_t       CAN_RBM_validatedTime;                                  // And for how long have we known them?
 uint8_t        CAN_RBM_potentialSourceID;                              // Is there a potential new kid we are watching?  To see if they make it past the settling period?
 uint8_t        CAN_RBM_devicePriority;                                 // How 'smart' is the master we are linked with? (0 = not very, need to look for someone smarter)
 unsigned char  SID;                                                    // SID to 'align' different NMEA messages.  Incremented each time N2kDCStatus_message() is sent out.

#define SBSZ  32                                                                                                                // 32 byte 'bit' arrays 
                                                                        // Two bit flag table to coordinate 'Charger Status' and 'Charger Status2' PGNs.
uint8_t        chargerSBHP[SBSZ];                                       // Bit = 1 --> CAN address is on same DC_SOURCE as us, and Higher priority.
uint8_t        chargerSBEP[SBSZ];                                       // Bit = 1 --> CAN address is on same DC_SOURCE as us, and Equal priority.
uint8_t        chargerSBLP[SBSZ];                                       // Bit = 1 --> CAN address is on same DC_SOURCE as us, and Lower priority.
                                                                        // Caution:  Make these three tables the SAME SIZE - see: reset_BIT_arrarys()
                                                                        // (I HATE needing to use these table, but - RV-C spec has some holes where it does not provide clear linkage....)
                                                                        // (NMEA-2000 is much worst, and I am not going to try and fill them --  see forced_CAN_ID for one of the workarouds...)
                                                                        


    //-- Queues to hold the J1939 - TERMIANAL sending and receiving characters.
#define cASCII_RX_BUFFER_SIZE  9                                        // Am using simple FiFo which has roaming 'unused' spot - trades off one byte of RAM for simpler code size.
#define cASCII_TX_BUFFER_SIZE (OUTBOUND_BUFF_SIZE + 10)                 // Large enough to hold status string and an 'AOK' at one time..

uint8_t    _rx_buffer_head = 0;                                         //  We expect to only recevie (and have to deal with) 8 up to 8 characters at a time...
uint8_t    _rx_buffer_tail = 0;
uint16_t   _tx_buffer_head = 0;                                         // Sending buffer however can be rather large, esp when user asked for all-status strings!
uint16_t   _tx_buffer_tail = 0;

char        _cASCII_rx_buffer[cASCII_RX_BUFFER_SIZE];                   // (FIFOs, not string buffers.  No need for +1 to hold the \0 terminator)
char        _cASCII_tx_buffer[cASCII_TX_BUFFER_SIZE];



    typedef struct {
       uint8_t sentThisPeriod : 1;                                      // Working flag, has this message been sent this 'period'?
       uint8_t N2K : 1;                                                 // Only send if we are configured to enable NEMA200 messages
       uint8_t OSE : 1;                                                 // Only send if we are configured to enable OSEnergy messages (RV-C in current implementation)
       uint8_t RBM : 1;                                                 // Only send if we are the RBM (Remote Battery Master)
       uint8_t b5 : 1;
       uint8_t b6 : 1;
       uint8_t b7 : 1;
       uint8_t b8 : 1;
       } tCANHanBitFld;





    //----      The following table holds all potential OSEnergy CAN messages, even if not supported by a given device.
    //          Compile options based on the OSE_xxx flags will device if code is populated for these options.
    //             (I chose to do it that way, else the table its self became very cluttered with lots of #ifdefs)
    //


    typedef struct {
        uint32_t PGN;
        void         (*Receiver)   (const tN2kMsg &N2kMsg);
        bool         (*Transmitter)(tN2kMsg &N2kMsg);
        unsigned int  transPeriod;                                      // 'Normal' transmission time in mS between messages. (0=transmit only upon request)
        tCANHanBitFld flag;                                             // Has a message been sent this 'period' already, has it already had its turn, is it even allowed? 
        } tCANHandlers;

    void N2kDCBatStatus_handler(const tN2kMsg &N2kMsg);
    void RVCDCStatus1_handler  (const tN2kMsg &N2kMsg);
    void RVCDCStatus2_handler  (const tN2kMsg &N2kMsg);
    void RVCDCStatus4_handler  (const tN2kMsg &N2kMsg);
    void RVCDCStatus5_handler  (const tN2kMsg &N2kMsg);
    void RVCDCStatus6_handler  (const tN2kMsg &N2kMsg);
    void RVCDCDisconnectStatus_handler(const tN2kMsg &N2kMsg);
    void RVCDCDisconnectCommand_handler(const tN2kMsg &N2kMsg);
    void RVCChrgStat_handler   (const tN2kMsg &N2kMsg);
    void RVCChrgStat2_handler  (const tN2kMsg &N2kMsg);
    void RVCGenCommand_handler (const tN2kMsg &N2kMsg);
    void RVCTerminal_handler   (const tN2kMsg &N2kMsg);
    void J1939EngineSpeed_handler(const tN2kMsg &N2kMsg);
    void sendNAK_handler       (const tN2kMsg &N2kMsg);

    
    bool N2kDCStatus_message   (tN2kMsg &N2kMsg);
    bool N2kBatConf_message    (tN2kMsg &N2kMsg);
    bool N2kDCBatStatus_message(tN2kMsg &N2kMsg);
    bool N2KEngineParamRapid   (tN2kMsg &N2kMsg);
    bool RVCDCStatus1_message  (tN2kMsg &N2kMsg);
    bool RVCDCStatus1OA_message(tN2kMsg &N2kMsg);
    bool RVCDCStatus2_message  (tN2kMsg &N2kMsg);
    bool RVCDCStatus4_message  (tN2kMsg &N2kMsg);
    bool RVCDCStatus5_message  (tN2kMsg &N2kMsg);
    bool RVCDCStatus5OV_message(tN2kMsg &N2kMsg);
    bool RVCChrgStat_message   (tN2kMsg &N2kMsg);
    bool RVCChrgStatCHP_message(tN2kMsg &N2kMsg);
    bool RVCChrgStat2_message  (tN2kMsg &N2kMsg);
    bool RVCChrgConfig_message (tN2kMsg &N2kMsg);
    bool RVCChrgConfig2_message(tN2kMsg &N2kMsg);
    bool RVCChrgConfig3_message(tN2kMsg &N2kMsg);
    bool RVCChrgConfig4_message(tN2kMsg &N2kMsg);
    bool RVCChrgEqualStat_message(tN2kMsg &N2kMsg);
    bool RVCChrgEqualConfig_message(tN2kMsg &N2kMsg);
    bool RVCProdId_message     (tN2kMsg &N2kMsg);
    bool RVCTerminal_message   (tN2kMsg &N2kMsg);
    bool ISODiagnostics_message(tN2kMsg &N2kMsg);
    bool ISODiagnosticsER_message(tN2kMsg &N2kMsg);
            
    tCANHandlers CANHandlers[]={
        #ifdef SUPPORT_NMEA2000                                     //  Sent | N2K| RVC | RBM   <<-- Characterization flags, when are these messages allowed, have they been sent recently, etc..
        {127506L,NULL,                  &N2kDCStatus_message,      667,{false,true,false,false}},   // 0x1F212
        {127508L,&N2kDCBatStatus_handler,&N2kDCBatStatus_message,  667,{false,true,false,false}},   // 0x1F214
        {127513L,NULL,                  &N2kBatConf_message,         0,{false,true,false,false}},   // 0x1F219
        {127488L,NULL,                  &N2KEngineParamRapid,      500,{false,true,false,false}},   // 0x1F200         // Note I am using 500ms vs. 100mS per N2K Spec.  (Why SOO fast?) 
        #endif
        #ifdef SUPPORT_RVC                                        
        {0x1FFFD,&RVCDCStatus1_handler, &RVCDCStatus1_message,     500,{false,false,true,true}},
        {0x1FFFD, NULL,                 &RVCDCStatus1OA_message,   100,{false,false,true,true}},
        {0x1FFFC,&RVCDCStatus2_handler, &RVCDCStatus2_message,     500,{false,false,true,true}},
        {0x1FEC9,&RVCDCStatus4_handler, &RVCDCStatus4_message,    5000,{false,false,true,true}},
        {0x1FEC8,&RVCDCStatus5_handler, &RVCDCStatus5_message,     500,{false,false,true,true}},
        {0x1FEC8, NULL,                 &RVCDCStatus5OV_message,   100,{false,false,true,true}},            // Special instance, called every 100mS when over voltage.
        {0x1FEC7,&RVCDCStatus6_handler,           NULL,              0,{false,false,true,true}},
        {0x1FED0,&RVCDCDisconnectStatus_handler,  NULL,              0,{false,false,true,false}},
        {0x1FECF,&RVCDCDisconnectCommand_handler, NULL,              0,{false,false,true,false}},
        {0x1FFC7,&RVCChrgStat_handler,  &RVCChrgStat_message,     5000,{false,false,true,false}},
        {0x1FFC7, NULL,                 &RVCChrgStatCHP_message,   500,{false,false,true,false}},           // Special instance, send out every 500mS (Max) if % drive changes.
        {0x1FEA3,&RVCChrgStat2_handler, &RVCChrgStat2_message,     500,{false,false,true,false}}, 
        {0x1FFC6,&sendNAK_handler,      &RVCChrgConfig_message,      0,{false,false,true,false}},           // For now we do not allow CPE configuration via the RVC protocol.
        {0x1FF96,&sendNAK_handler,      &RVCChrgConfig2_message,     0,{false,false,true,false}},
        {0x1FECC,&sendNAK_handler,      &RVCChrgConfig3_message,     0,{false,false,true,false}},
        {0x1FEBF,&sendNAK_handler,      &RVCChrgConfig4_message,     0,{false,false,true,false}},
        {0x1FF99, NULL,                 &RVCChrgEqualStat_message,5000,{false,false,true,false}},
        {0x1FF98,&sendNAK_handler,      &RVCChrgEqualConfig_message, 0,{false,false,true,false}},
        {0xFEEB,  NULL,                 &RVCProdId_message,          0,{false,false,true,false}},
        {0x17E00,&RVCTerminal_handler,  &RVCTerminal_message,       50,{false,false,true,false}},           // Terminal handler called every 50mS to send out 'Next portion' of string.
        
        #ifdef OSE_GENERATOR
          {0x1FFDA, &RVCGenCommand_handler,         NULL,            0,{false,false,true,false}},
          #endif  /* OSE Generator */
        #endif    /* RV-C */
        #ifdef SUPPORT_J1939
        {0xF004,&J1939EngineSpeed_handler,         NULL,              0,{false,false,true,false}},
        #endif

            // Common J1939 type messages we need to handle.
        {0x1FECA, NULL,                 &ISODiagnostics_message,  5000,{false,false,true,false}}, 
        {0x1FECA, NULL,                 &ISODiagnosticsER_message,1000,{false,false,true,false}},           // Special instance, sent out more often during fault condition.
        
        {0,NULL,NULL,0,false}                                                                               // ----PGN of 0 indicates end of table----
        };
              // Note:  send_CAN() scans this table from the beginning each time looking for the next message who has timed out and is ready to be sent.
              //        As such, this table becomes a priority order for CAN messages.  be careful placing a very low time period message early in the table
              //        as that may cause that one message to dominate transmissions - preventing later entries from being serviced.





//--  Internal prototypes (helper functions, etc)
void reset_BIT_arrarys(void);
bool invokeCANTx (tCANHandlers TxHandle, tN2kMsg &N2kMsg);
void send_CAN(void); 
void check_CAN(void);
void decide_if_CAN_RBM(void);


//------------------------------------------------------------------------------------------------------
// Initialize CAN
//
//      This function will startup the CAN bus
//
//
//------------------------------------------------------------------------------------------------------

bool initialize_CAN(void) {

    
        #ifdef CPU_STM32
            HAL_GPIO_WritePin(CAN_Stby_GPIO_Port, CAN_Stby_Pin, GPIO_PIN_RESET);        // For now, CAN transeiver is ALWAYS active - add better power managment later.
        #endif

   

        
        invalidate_RBM();                                                               // Before starting the CAN - initialize all the Remote Battery Master watches
        
        NMEA2000.SetProductInformation( "1",                                            // Manufacturer's Model serial code  (We will just always be #1, user can make a unique ID via .REG_NAME)
                                        PRODUCT_CODE,                                   // Manufacturer's product code
                                        systemConfig.DEVICE_NAME,                       // Manufacturer's  Model ID  -- we will use the users configured regulator name for better clarity.
                                        firmwareVersion,                                // Manufacturer's software version code
                                        HARDWARE_VERSION,                               // Manufacturer's Model version
                                        0,                                              // We present no load on the NEMA-2000 bus.  (Load equivalence x * 50mA)
                                        1300,                                           // NMEA2000 version used
                                        0                                               // Sertification level ??
                                        );
                               
                                           
        NMEA2000.SetDeviceInformation(SERIAL_NUMBER,                                    // Unique number. Use e.g. Serial number.  (Well, we will just always be #1)
                                      DEVICE_FUNCTON,                                   // Device function=Alternator. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                      DEVICE_CLASS,                                     // Device class=Propulsion.    See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                      MANUF_CODE                                        // Manuf Code: Just chose free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                      );

        #ifndef CPU_AVRCAN                                                              //-- Streaming NMEA2000 calls have been removed in the special 'edited' N2K lib used on with AVR CPUs (to save space)                                                                                                             
            NMEA2000.SetForwardStream(&serStream);                                      // New 'portable' version of NMEA2000 libs needs to have the serial stream declared.
            NMEA2000.EnableForward(false);                                              // If they are are enabled, do not forward CAN messages to the Serial port.
            #ifdef DEBUG
               NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);                           // While debugging - send error and status messages to the Serial port.
            #endif
        #endif


        // NMEA2000.SetSingleFrameMessages(SingleFrameMessages);                           // Register our extended list of messages we will accept.
        // NMEA2000.ForwardOnlyKnownMessages(false);                                       // Have the NMEA2000 lib send us ALL non-system messages, we will sort out which ones we want to deal with.
                                                                                        //  WAIT, These are the DEFAULT setting!  So - - -  Do not really need to call these. 

        NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,canConfig.LAST_CAN_ID);               // Configure for normal node.  try for the same CAN-ID we had last time, else start looking for dynamic addresses @ 128 (Preferred "Power Components" range in RV-C spec)        
        NMEA2000.SetN2kCANMsgBufSize(2);                                                // Define only 2x reception buffers - should be sufficient.
        NMEA2000.SetN2kCANSendFrameBufSize(20);                                         //  Allmost all of our transmissions are simple, and a 20x deep outbound queue should be sufficent (vs. 40x default)
        NMEA2000.SetMsgHandler(handle_CAN_Messages);                                    // Callback function NMEA2000.ParseMessages() uses when a CAN message is received.
        NMEA2000.SetISORqstHandler(handle_CAN_Requests);                                // Callback function NMEA2000.ParseMessages() uses when an ISO Request is received.
        NMEA2000.Open();                                                                // And start up the CAN controller.
        
        reset_BIT_arrarys();                                                            // Reset the chargerSBEP and chargerSBHP flag tables.
        
        return(true);
}


//---- Helper function, this will 'clear out' the bit-array tables used to 'synchronize' RVC Charger Status and Charger Status2 messages.

void reset_BIT_arrarys(void) {
    int i;

    for (i = 0; i < SBSZ; i++) {
        chargerSBHP[i] = 0x00;
        chargerSBEP[i] = 0x00;
        chargerSBLP[i] = 0x00;
    }

    average_EPC_utilization = 0;

}





//------------------------------------------------------------------------------------------------------
// Manage CAN
//
//      This function will handle any Tx/Rx processing around the CAN.  It will also deal with power-states
//      (for systems which have that capability), ala during Sleep it will power-down the CAN subsystem.
//      
//
//
//------------------------------------------------------------------------------------------------------

bool manage_CAN(void){
    
    
    check_CAN();                                                                                    // See if we have any incoming messages.
    send_CAN();                                                                                     // Send out CAN status messages.
    decide_if_CAN_RBM();                                                                            // Decide who the Can RemoteBatteryMaster will be.  Including if it should be us.

    
    #ifdef CPU_STM32
        //---  STM32F0 projects are ran under RTOS, and feature the ability to go into low-power 'sleep' mode.
        //      Here we check to see if the sytem is sleeping, and if so power-down the CAN subsystem.
        if (systemState == sleeping) {                                                              // Time to put CAN subsystem into low-power mode
            HAL_GPIO_WritePin(CAN_Stby_GPIO_Port, CAN_Stby_Pin, GPIO_PIN_SET);                      // CAN transeiver to standby mode
            
            __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_SLK);                                                 // Set Sleep-IRQ for   CEC_CAN_IRQHandler() in stm32f0xx_it.c
            HAL_CAN_Sleep(&hcan);
        } 
        else if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_SLAK) == 1) {                                   // System is awake  .. but CAN is sleeping!
            __HAL_CAN_DISABLE_IT(&hcan, CAN_IT_WKU);                                                // Turn off IRQs -- individual message
            __HAL_CAN_DISABLE_IT(&hcan, CAN_IT_SLK);                                                // And Sleep
  
            HAL_GPIO_WritePin(CAN_Stby_GPIO_Port, CAN_Stby_Pin, GPIO_PIN_RESET);                    // Repower CAN transeiver
            HAL_CAN_WakeUp(&hcan);                                                                  // Restart the CAN subsystem.  
            }
        #endif
              
                    
    return(true);
            
}









//------------------------------------------------------------------------------------------------------
// Send CAN
//
//      This function dispatched CAN messages to the CAN bus.  Using the table CANHandlers, it will scan the table
//      looking for a message whose time has come, and who has not been sent for awhile.
//
//
//------------------------------------------------------------------------------------------------------

void send_CAN(void){
    int         i;
    tN2kMsg     N2kMsg;
    bool static OV_Triggered = false;
        

    //---- Before we do the regualor CAN stuff, if we are the RBM - need to see if battery just entered Over Voltage condition.
    //      If so, send out the OV message ASAP, to let the world know it needs to do something now!
    #ifdef OSE_RBM
    if ((CAN_weAreRBM) && (measuredBatVolts > (targetBatVolts + LD1_THRESHOLD))) {              // Using the same trigger point as the 1st Load Dump threshold for chargers
        
        if (OV_Triggered == false) {
            RVCDCStatus5OV_message(N2kMsg);                                                     // Force out an Over Volt message ASAP at 1st notice
            NMEA2000.SendMsg(N2kMsg);                                                           //  And send it out.  
            OV_Triggered = true;                                                                // Only force OV message once, then fall back to 'normal' update timing.  
            return;                                                                             // And return - puts a little time between this forced message and what every is next. 
            }
        } 
    else
        OV_Triggered = false;
    #endif




    //--- Done with all that, lets see if there is another CAN message that needs to be sent out.
  
    for (i=0; CANHandlers[i].PGN!=0; i++) {                                                   // Scan the table, looking for someone who is ready to be sent out.
        if (CANHandlers[i].transPeriod == 0)                  continue;                       // This table entry does not want to be sent (ala, send only on request).

        if ((millis() % CANHandlers[i].transPeriod) > (CANHandlers[i].transPeriod / 2)) {     // We are in the time period for this message to be sent.
            if (CANHandlers[i].flag.sentThisPeriod == true)   continue;                       // However, if we have already sent it - look for one that is actually ready to be sent.

            if (!invokeCANTx(CANHandlers[i], N2kMsg))         continue;                       // If this function is not to be transmitted at this time, keep looking for one that is.
            
            CANHandlers[i].flag.sentThisPeriod = true;
            return;                                                                           // We got one out!  Done for now.  Do some more processing in the mainloop to 
            }                                                                                 // space CAN messages out some - letting the hardware catch up.
        else
            CANHandlers[i].flag.sentThisPeriod = false;                                       // In the back half of the time period - reset the 'transmitted' flag

    }

}



//--  Invoke CAN Transmit handlers
//      Helper function will check to see if the passed Tx entry is allowed to be executed (exists, is enabled, etc.)
//      If so, Tx handler will be called and TRUE will be returned (Even if message is not sent).
//      If no function, or function is not allowed (e.g., NMEA2000 has been disabled)  FALSE is returned

bool invokeCANTx (tCANHandlers TxHandle, tN2kMsg &N2kMsg) {

    if (TxHandle.Transmitter == NULL)                     return(false);                    // Entry does not have a message transmitting procedure assigned.
    
    //-- Check the flags to see if we are even enabled to send these types of messages
    if (TxHandle.flag.N2K && !canConfig.ENABLE_NMEA2000)  return(false);                    // User has disabled NMEA2000 messages, perhaps due to conflict in the system.
    if (TxHandle.flag.OSE && !canConfig.ENABLE_OSE)       return(false);               
    if (TxHandle.flag.RBM && !CAN_weAreRBM)               return(false);    

     //--  OK to try and send.
    if (TxHandle.Transmitter(N2kMsg))                                                       // Call the transmitting procedure 
        NMEA2000.SendMsg(N2kMsg);                                                           //  And if the handler was successful, push the message into the Tx queue.  
     
    return (true);    
}












#ifdef SUPPORT_NMEA2000 
  /*****************************************************************************
                    // NMEA2000-DC Detailed Status  - PGN127506
                    // Input:
                    //  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
                    //                          to indicate that they are measured at same time.
                    //  - DCInstance            DC instance.
                    //  - DCType                Defines type of DC source. See definition of tN2kDCType
                    //  - StateOfCharge         % of charge
                    //  - StateOfHealth         % of heath
                    //  - TimeRemaining         Time remaining in minutes
                    //  - RippleVoltage         DC output voltage ripple in V
                     */
bool N2kDCStatus_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
 
      if (canConfig.ENABLE_NMEA2000_RAT == true)                                                // Have we been configured to look at this NMEA-2000 device for Remote Amps and Temperature? 
           return(false);                                                                       //   If so, lets not confuse things with our own message.



      SID++;                                                                                    // Increment the SID for this next group of messages to be sent out.
      SetN2kDCStatus(N2kMsg,SID,batteryInstance,N2kDCt_Alternator,
                     N2kUInt8NA,N2kUInt8NA,N2kDoubleNA,                                         // We do not know what % change, % health, nor Time remaining is.
                     N2kDoubleNA);                                                              // We do not know the DC Ripple
      return (true);




#elif defined (OSE_BATMON) || \
      defined (OSE_BMS)

      if (canConfig.ENABLE_NMEA2000_RAT == true)                                                // Have we been configured to look at this NMEA-2000 device for Remote Amps and Temperature? 
           return(false);                                                                       //   If so, lets not confuse things with our own message.



      SID++;                                                                                    // Increment the SID for this next group of messages to be sent out.
      SetN2kDCStatus(N2kMsg,SID,batteryInstance,N2kDCt_Battery,
  //!! HEY!!    Fill in the real stuff, you are the BatMon afterall!!
                     N2kUInt8NA,N2kUInt8NA,N2kDoubleNA,                                         // We do not know what % change, % health, nor Time remaining is.
                     N2kDoubleNA);                                                              // We do not know the DC Ripple
      return (true);

      

#else
    return(false);
#endif
    
}




 /*****************************************************************************
                    // NMEA2000-Battery Configuration Status   -- PGN127513
                    // Note this has not yet confirmed to be right. Specifically Peukert Exponent can have in
                    // this configuration values from 1 to 1.504. And I expect on code that I have to send
                    // value PeukertExponent-1 to the bus.
                    // Input:
                    //  - BatteryInstance       BatteryInstance.
                    //  - BatType               Type of battery. See definition of tN2kBatType
                    //  - SupportsEqual         Supports equalization. See definition of tN2kBatEqSupport
                    //  - BatNominalVoltage     Battery nominal voltage. See definition of tN2kBatNomVolt
                    //  - BatChemistry          Battery See definition of tN2kBatChem
                    //  - BatCapacity           Battery capacity in Coulombs. Use AhToCoulombs, if you have your value in Ah.
                    //  - BatTemperatureCoeff   Battery temperature coefficient in %
                    //  - PeukertExponent       Peukert Exponent
                    //  - ChargeEfficiencyFactor Charge efficiency factor
                    */
bool N2kBatConf_message(tN2kMsg &N2kMsg){
    tN2kBatType         N2kBatType;
    tN2kBatEqSupport    N2kBatEqSupport;
    tN2kBatChem         N2kBatChem;
    tN2kBatNomVolt      N2kBatNomVolt;

  
    // Translate the tRVCBatType into NMEA2000 types, as best as we can.
    // If there is not a match, we will try using the CAN 'Undefined' value of 0xff...
    N2kBatType = (tN2kBatType) 0xff;
    N2kBatChem = (tN2kBatChem) 0xff;
    
    switch   (canConfig.BATTERY_TYPE) { 
        case RVCDCbt_Flooded:       N2kBatType      = N2kDCbt_Flooded; 
                                    N2kBatChem      = N2kDCbc_LeadAcid;
                                    break;
                                    
        case RVCDCbt_Gel:           N2kBatType      = N2kDCbt_Gel; 
                                    N2kBatChem      = N2kDCbc_LeadAcid;
                                    break;
                                    
        case RVCDCbt_AGM:           N2kBatType      = N2kDCbt_AGM; 
                                    N2kBatChem      = N2kDCbc_LeadAcid;
                                    break;

        case RVCDCbt_LiPo:
        case RVCDCbt_LiFeP04:       N2kBatChem      = N2kDCbc_LiIon;
                                    break;
                                    
        case RVCDCbt_NiCad:         N2kBatChem      = N2kDCbc_NiCad;
                                    break;
        
        case RVCDCbt_NiMh:          N2kBatChem      = N2kDCbc_NiMh;
                                    break;
        
        default:                    break;
    }
   

    if ((chargingParms.EQUAL_BAT_V_SETPOINT != 0.0)  && (chargingParms.EXIT_EQUAL_DURATION != 0))
        N2kBatEqSupport = N2kDCES_Yes;
    else
        N2kBatEqSupport = N2kDCES_No;

    if      (systemVoltMult < 1.5)       N2kBatNomVolt = N2kDCbnv_12v;
    else if (systemVoltMult < 2.5)       N2kBatNomVolt = N2kDCbnv_24v;
    else if (systemVoltMult < 2.8)       N2kBatNomVolt = N2kDCbnv_32v;
    else if (systemVoltMult < 3.6)       N2kBatNomVolt = N2kDCbnv_42v;
    else                                 N2kBatNomVolt = N2kDCbnv_48v;                      // We do not support anything greater then a 48v system...

    SetN2kBatConf(N2kMsg,batteryInstance,N2kBatType,N2kBatEqSupport,N2kBatNomVolt,N2kBatChem,AhToCoulomb(500*systemAmpMult),chargingParms.BAT_TEMP_1C_COMP,0,0);
    return (true);
 }




/****************************************************************************
                    // NMEA2000-Battery Status  - PGN127508
                    // Input:
                    //  - BatteryInstance       BatteryInstance.
                    //  - BatteryVoltage        Battery voltage in V
                    //  - BatteryCurrent        Current in A
                    //  - BatteryTemperature    Battery temperature in degrees-K. Use function CToKelvin, if you want to use degrees-C.
                    //  - SID                   Sequence ID.
                    */
bool N2kDCBatStatus_message(tN2kMsg &N2kMsg){
    double  batTempK;

    if (canConfig.ENABLE_NMEA2000_RAT ==  true) return(false);                      //  Or we are looking for a proper NMEA2000 device to inform us of the battery status
                                                                                    //  (NMEA2000 is not the primary goal of this regulator, and as such we are not very good at it..)
    
    if (measuredBatTemp == -99)
        batTempK = N2kDoubleNA;                                                     // Temperatures are not available.
    else
        batTempK = CToKelvin((double)measuredBatTemp);


    SetN2kDCBatStatus(N2kMsg,batteryInstance,(double)measuredBatVolts,(double)measuredBatAmps,batTempK,SID);
    return (true);
}



/****************************************************************************
                    // Engine parameters rapid  - 127488L
                    // Input:
                    //  - EngineInstance        Engine instance.
                    //  - EngineSpeed           RPM (Revolutions Per Minute)
                    //  - EngineBoostPressure   in Pascal
                    //  - EngineTiltTrim        in 
                    */
bool N2KEngineParamRapid(tN2kMsg &N2kMsg) {
#if defined (OSE_ALTERNATOR) 
    if (measuredRPMs <= 0)  return(false);                                              // Sorry, we do not seem to know the RPMs at this time.
    

    SetN2kEngineParamRapid(N2kMsg, canConfig.ENGINE_INSTANCE, (double) measuredRPMs);  
    N2kMsg.Priority=8;                                                                  //  Override default priority (3), this message is not THAT important to us...;

    return (true);

    
#else
    return(false);
#endif 

}



#endif                          /*** #ifdef SUPPORT_NMEA2000 ***/



//=========================================================================================================================================================================
//=========================================================================================================================================================================
//=========================================================================================================================================================================
//=========================================================================================================================================================================


#ifdef SUPPORT_RVC

/*****************************************************************************
                    // DC Source Status 1
                    // Input:
                    //  - Instance              DC Instance (bus) ID.
                    //  - Device Priority       Relative ranking of DC Source
                    //  - DC Voltage            0..3212.5v, in 50mV steps
                    //  - DC Current            -2M..+2MA, in 1mA steps (0x77359400 = 0A)
                    */
bool RVCDCStatus1_message(tN2kMsg &N2kMsg){
    uint32_t  Adc;
                            
     if ((shuntAmpsMeasured == false) || (canConfig.SHUNT_AT_BAT == false))                 // Even if we ARE actively charging, but
      Adc = N2kUInt32NA;                                                                    //   .. have no idea what the battery current is
    else
      Adc = (uint32_t)(measuredBatAmps  * 1000.0) + 0x77359400;                             // The Shunt is working!
                                                                                            // Side note:  Need to force Amps = 0 if we are not in a charging state, to allow the self-prioritization
                                                                                            // schema to resolve who is the lowest priority CHARGING source.  (HEY!! What does this mean?  Am I doing this, or need to do it???? )
    SetRVCDCSourceStatus1(N2kMsg, batteryInstance, canConfig.DEVICE_PRIORITY,
                                (uint16_t) (measuredBatVolts * 20.0),
                                Adc);
                                
    return (true);
}



bool RVCDCStatus1OA_message(tN2kMsg &N2kMsg){

    if (measuredBatAmps > targetBatAmps)                                                    // Special 100mS message when over target  (OA --> Over Amps)
        return(RVCDCStatus1_message(N2kMsg));                                               // For current, just use the standard CAN priority, no need to raise its priority.
    else    
        return (false);    
}




/*****************************************************************************
                    // DC Source Status 2
                    // Input:
                    //  - Instance              DC Instance (bus) ID.
                    //  - Device Priority       Relative ranking of DC Source
                    //  - Source Temperature    -273 to 1735 Deg-C  in 0.03125c steps
                    //  - State of Charge       Batteries: % SOC;  DC Charging sources:  Current % output.
                    //  - Time Remaining        Estimated number of minutes until SOC reaches 0%
                    */

bool RVCDCStatus2_message(tN2kMsg &N2kMsg){ 
    int16_t tempSend;
                           
 
    if (measuredBatTemp == -99)  
       tempSend = N2kInt16NA;                                               // Temperature is not available.
    else 
       tempSend = (int) ((float)measuredBatTemp / 0.03125);


    uint8_t SOC;
    SOC = N2kUInt8NA;                                                       // Until we code in the state-of-charge info, send n/a
      
    uint16_t TR;
    TR = N2kUInt16NA;                                                       // Same to Time Remaining.
      
    SetRVCDCSourceStatus2  (N2kMsg, batteryInstance, canConfig.DEVICE_PRIORITY, tempSend, SOC, TR);
    return (true);

}






/*****************************************************************************
                    // DC Source Status 4
                    // Input:
                    //  - Instance              DC Instance (bus) ID.
                    //  - Device Priority       Relative ranking of DC Source
                    //  - Desired Charge Mode   Charging mode / state being requested.
                    //  - Desired DC Voltage    Target voltage for chargers to deliver  0..3212.5v, in 50mV steps
                    //  - Desired DC Current    Target current for charger to deliver combined  -1600A..1612.5A, in 50mA steps (0x7D00 = 0A)
                    //  - Battery Type
                     */
bool RVCDCStatus4_message(tN2kMsg &N2kMsg){
    tRVCBatChrgMode state;

                            
    switch (chargingState) {
        case ramping:
        case post_ramp:
        case determine_ALT_cap:
        case bulk_charge:           state = RVCDCbcm_Bulk;
                                    break;
                         
        
        case acceptance_charge:     state = RVCDCbcm_Absorption;
                                    break;
                         
            
        case overcharge_charge:     state = RVCDCbcm_Overcharge;
                                    break;
                         
        
        case forced_float_charge: 
        case float_charge:          state  = RVCDCbcm_Float;
                                    break;
                         
        
        
        case equalize:              state = RVCDCbcm_Equalize;
                                    break;
                         
        
        case RBM_CVCC:              state = RVCDCbcm_CVCC;
                                    break;
                         
        default:                    state = RVCDCbcm_Disabled;
                                    break;             
        }
    
                      
                        
                        
    SetRVCDCSourceStatus4(N2kMsg, batteryInstance, canConfig.DEVICE_PRIORITY, state,
                                 (uint16_t) (targetBatVolts * 20.0),
                                 (uint16_t) (targetBatAmps  * 20.0) + 0x7D00,
                                 canConfig.BATTERY_TYPE);
    return (true);            

}




/*****************************************************************************
                     // DC Source Status 5
                     // Input:
                     //  - Instance              DC Instance (bus) ID.
                     //  - Device Priority       Relative ranking of DC Source
                     //  - DC Voltage            High precision value in 1mV.  Useful for remote instrumentation
                     //  - VDC ROC               Rate-of-change (dV/dT) in mV/s  -- 32000 = 0 mV/s
                     */
bool RVCDCStatus5_message(tN2kMsg &N2kMsg){
    
                            
     SetRVCDCSourceStatus5(N2kMsg, batteryInstance,
                        canConfig.DEVICE_PRIORITY,
                        (uint32_t) (measuredBatVolts * 1000.0),
                        N2kUInt16NA);                                                   //!  THIS NEEDS TO BE EDITED, SEND OUT THE dV/dT, NOW JUST PLACEHOLDER
    
    return (true);

}



bool RVCDCStatus5OV_message(tN2kMsg &N2kMsg){                                           // Special instance, called every 100mS - used to send higher priority OverVoltage messages

    if (measuredBatVolts > targetBatVolts)  {                                           // Are we in an overvoltage situation?

        SetRVCDCSourceStatus5(N2kMsg, batteryInstance,
                            canConfig.DEVICE_PRIORITY,
                            (uint32_t) (measuredBatVolts * 1000.0),
                            N2kUInt16NA);                                               //!!  THIS NEEDS TO BE EDITED, SEND OUT THE dV/dT, NOW JUST PLACEHOLDER

        N2kMsg.Priority = 2;                                                            // Raise the priority for this special message.
        return (true);
    } else
       return(false);                                                                   // All is well, no need to send out panic message!
}













/*****************************************************************************
                    // Charger Status - 1FFC7h
                    // Input:
                    //  - Instance
                    //  - Charge Voltage                0..3212.5v, in 50mV steps
                    //  - Charge Current                -1600..+1512.5 in 50mA steps (0x7D00 = 0A)
                    //  - % max current
                    //  - Operating State               (Bulk, float, etc)
                    //  - Default PO state
                    //  - Auto Recharge
                    //  - Force Charged 
                    */

bool RVCChrgStat_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
 
     tRVCBatChrgMode    state;
     tRVCChrgForceChrg  forcedChrg = RVCDCfc_Cancel;
     bool               autoRechg; 
    

    
    
    switch (chargingState) {
        case ramping:
        case post_ramp:
        case determine_ALT_cap:
        case bulk_charge:           state = RVCDCbcm_Bulk;
                                    break;
                         
        
        case acceptance_charge:     state = RVCDCbcm_Absorption;
                                    break;
                         
            
        case overcharge_charge:     state = RVCDCbcm_Overcharge;
                                    break;
                         
        
        case forced_float_charge:   forcedChrg = RVCDCfc_Float;     // No break;  Fall through the float mode for the rest.

        case float_charge:          state      = RVCDCbcm_Float;
                                    break;
                         
        
        
        case equalize:              state = RVCDCbcm_Equalize;
                                    break;
                         
        
        case RBM_CVCC:              state = RVCDCbcm_CVCC;
                                    break;
                         
        default:                    state = RVCDCbcm_Disabled;
                                    break;             
    }
   
     

    if ((chargingParms.FLOAT_TO_BULK_AMPS  != 0)   ||
        (chargingParms.FLOAT_TO_BULK_AHS   != 0)   ||
        (chargingParms.FLOAT_TO_BULK_VOLTS != 0.0))
        autoRechg = true;
    else
        autoRechg = false;
         

         
    SetRVCChargerStatus(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                          (uint16_t)(targetBatVolts * 20.0),
                          (int16_t) (targetAltAmps  * 20.0) + 0x7D00,
                          ALT_Per_Util(),
                          state,
                          true,
                          autoRechg,
                          forcedChrg);

    return (true);
    
#else
    return(false);                                                              // The device does not support this PGN
#endif    
}




bool RVCChrgStatCHP_message(tN2kMsg &N2kMsg){
 #if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
       
    int static priorPDR;                                // Prior 'Drive %'
    int i;
    
    
    i = ((100*fieldPWMvalue) / FIELD_PWM_MAX);
    
    if (abs(priorPDR - i) > 5) {
        RVCChrgStat_message(N2kMsg);                    // Changed more then 5%
        priorPDR = i;   
        }

    return (true);
    
#else
    return(false);                                                              // The device does not support this PGN
#endif            
}
    
    
    
    
    
 


/*****************************************************************************
                    // Charger Status2 - 1FEA3h  
                    // Input:
                        - Instance              Instance of charger
                    //  - DC Source Instance    DC Instance (bus) ID associated with
                    //  - Device Priority       Relative ranking of DC charging Source
                    //  - DC Voltage            0..3212.5v, in 50mV steps
                    //  - DC Current            -1600..1612.5a, in 50mA steps (0x7D00 = 0A)
                    //  - Temperature           -40..210 in deg-C, in 1C steps
                    */
bool RVCChrgStat2_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
        
    uint16_t  Adc;
    int8_t    temp;
    
        
    if ((chargingState < ramping)       ||                                              // If we are not in a charging state
        (chargingState > RBM_CVCC)      || 
        (fieldPWMvalue == FIELD_PWM_MIN)  ||                                            //  .. or the field it turned off
       ((fieldPWMvalue <= thresholdPWMvalue) && (tachMode)))                            //     (including if we are holding the Field a little high to keep the tach running)
       Adc = 0x7D00;                                                                    // tell the world that the Amps being delivered are 0 -- we are inactive at this point.
   else {
       if ((shuntAmpsMeasured == false)  || (canConfig.SHUNT_AT_BAT == true))           // We are actively charging, though we have no idea what the current being delivered is..
            Adc = N2kUInt16NA;                                                          // Still need to inform other chargers we are active, for the prioritization to work.
       else
           Adc = (uint16_t)(measuredAltAmps  * 20.0) + 0x7D00;                          // Finally.  We are charging, and it seems the Shunt is working!
    }
                                                                                        // Side note:  Need to force Amps = 0 if we are not in a charging state, to allow the self-prioritization
                                                                                        // schema to resolve who is the lowest priority CHARGING source.

    if (measuredAltTemp == -99)
        temp = N2kInt8NA;
    else
        temp = measuredAltTemp;
        
                                                                                        
    SetRVCChargerStatus2(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                                 batteryInstance,
                                 canConfig.DEVICE_PRIORITY,
                                 (uint16_t)(measuredAltVolts * 20.0),
                                 Adc,
                                 temp);
                                  
    return (true);

    
#else
    return(false);                                                                          // The device does not support this PGN
#endif 
}




/*****************************************************************************
                    // Charger Configuration Status - 1FFC6h
                    // Input:
                    //  - Instance
                    //  - Charging Algorithm  
                    //  - Controller Mode
                    //  - Battery Sensor Present
                    //  - Charger AC Line          Line 1 or 2 (AC Chargers only)
                    //  - Linkage Mode
                    //  - Battery Type
                    //  - Battery Bank Size         0..65,530 Ah, 1Ah increments
                    //  - Maximum charging current  0..250, 1A increments
                    */
bool RVCChrgConfig_message(tN2kMsg &N2kMsg) {
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
        
    bool      linked;
    uint8_t   maxAmps;

    linked = ((CAN_RBM_sourceID != 0) && (!ignoringRBM));

     if (altCapAmps > 0)    maxAmps = min(altCapAmps, 250);
        else                maxAmps = N2kUInt8NA;
    
    SetRVCChargerConfigStatus(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                              (chargingState==RBM_CVCC) ? RVCDCca_ConstantVoltage:RVCDCca_3Stage,  
                              linked ? RVCDCcm_Standalone:RVCDCcm_Linked,
                              (measuredBatTemp != -99), 
                              RVCDCcl_na,                                                           // We are not connected to any AC line 
                              linked, 
                              RVCDCbt_Unknown,  
                              (uint16_t) (500.0*systemAmpMult), 
                              maxAmps);
                                      
    return (true);

#else
    return(false);                                                                          // The device does not support this PGN
#endif     
}


/*****************************************************************************
                    // Charger Configuration Status2 - 1FF96h
                    // Input:
                    //  - Instance
                    //  - Max Charge Current %
                    //  - Max AC current %          Of attached line      (AC Chargers only) 
                    //  - Shore Breaker Size        0..250, 1A increments (AC Chargers only)
                    //  - Default Batt Temp
                    //  - Recharge Voltage           0..3212.5v, in 50mV steps
                    */
bool RVCChrgConfig2_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    SetRVCChargerConfigStatus2(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                               100, 
                               N2kUInt8NA, 
                               N2kUInt8NA, 
                               (uint8_t)  (BAT_TEMP_NOMINAL), 
                               (uint16_t) (chargingParms.FLOAT_TO_BULK_VOLTS * systemVoltMult *  20.0));
    return (true);

#else
    return(false);                                                                          // The device does not support this PGN
#endif      
}



/*****************************************************************************
                        // Charger Configuration Status3 - 1FECCh
                        // Input:
                        //  - Instance
                        //  - Bulk Voltage           0..3212.5v, in 50mV steps
                        //  - Absorption Voltage     0..3212.5v, in 50mV steps
                        //  - Float Voltage          0..3212.5v, in 50mV steps
                        //  - Temp Comp              mV/K
                        */
bool RVCChrgConfig3_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)

    SetRVCChargerConfigStatus3(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                               (uint16_t) (chargingParms.ACPT_BAT_V_SETPOINT  * systemVoltMult *  20.0), 
                               (uint16_t) (chargingParms.ACPT_BAT_V_SETPOINT  * systemVoltMult *  20.0), 
                               (uint16_t) (chargingParms.FLOAT_BAT_V_SETPOINT * systemVoltMult *  20.0), 
                               (uint8_t)  (chargingParms.BAT_TEMP_1C_COMP     * 1000));
    return (true);

#else
    return(false);                                                                          // The device does not support this PGN
#endif  
}



/*****************************************************************************
                        // Charger Configuration Status4 - 1FEBFh
                        // Input:
                        //  - Instance
                        //  - Bulk Time           0..65,530min in 1min steps
                        //  - Absorption Time     0..65,530min in 1min steps
                        //  - Float Time          0..65,530min in 1min steps
                        */
bool RVCChrgConfig4_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    SetRVCChargerConfigStatus4(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                               (uint16_t) (65530), 
                               (uint16_t) (chargingParms.EXIT_ACPT_DURATION  / 1000), 
                               (uint16_t) (chargingParms.EXIT_FLOAT_DURATION / 1000));
    return (true);

    
#else
    return(false);                                                                          // The device does not support this PGN
#endif  
}


/*****************************************************************************
                        // Charger Equalization Status - 1FF99h
                        // Input:
                        //  - Instance
                        //  - Time Remaining            0..65,530min in 1min steps
                        //  - Pre-Charging
                        */
bool RVCChrgEqualStat_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)

        
    SetRVCChargerEqualStatus(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                             (chargingState == equalize) ? ((millis() - altModeChanged) / 1000) : 0, 
                             (chargingState != equalize));

    return (true);

    
#else
    return(false);                                                                          // The device does not support this PGN
#endif 
}



/*****************************************************************************
                        // Charger Equalization Configuration Status - 1FF98h
                        // Input:
                        //  - Instance
                        //  - Equalization Voltage      0..3212.5v, in 50mV steps
                        //  - Equalization Time         0..65,530min in 1min steps
                        */
bool RVCChrgEqualConfig_message(tN2kMsg &N2kMsg){
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    SetRVCChargerEqualConfigStatus(N2kMsg, RVC_CHARGER_TYPE, canConfig.DEVICE_INSTANCE, 
                                  (uint16_t) (chargingParms.EQUAL_BAT_V_SETPOINT * systemVoltMult *  20.0), 
                                  (uint16_t) (chargingParms.EXIT_EQUAL_DURATION / 1000));
    return (true);

    
#else
    return(false);                                                                          // The device does not support this PGN
#endif 
}





/*****************************************************************************
                        // Product Identificaiton -  FEEBh
                        // Input:
                        //  - 8-bit single-byte coded graphic character text, with four field delimited by "*"
                        //    Field 1: Make
                        //    Field 2: Model
                        //    Field 3: Serial number
                        //    Field 4: Unit number (for products where the RV-C node is separate from the mechanical unit.)
                        */

bool RVCProdId_message(tN2kMsg &N2kMsg) {
    char      buff[OUTBOUND_BUFF_SIZE+1];
    int       cnt;
    
    cnt = snprintf_P(buff, OUTBOUND_BUFF_SIZE, PSTR("****"));

    if (cnt <= 8) {                                                                             // We can pack this into a single message
 
        N2kMsg.SetPGN(65259L);
        N2kMsg.Priority=6;
        for (int i=0; i<cnt; i++)
              N2kMsg.AddByte(buff[i]);  
          
        return (true);       

    } else {

        // ERROR, we need to send multi-packet message -- need to add code here for that once we start putting in more details into the string.
        //!! HEY!! Need to handle this multi-byte message thing!
    }


  return (false);

}




/*****************************************************************************
                        // Terminal - 17E00h 
                        // Input:
                        //  - Source / Destination
                        //  - Count                     0..8
                        //  - Characters                Buffer with up to 8 characters
                        */
bool RVCTerminal_message(tN2kMsg &N2kMsg){
    int count;
    char buff[8];
  
    if (_tx_buffer_head != _tx_buffer_tail) {                                                   // Is there an ASCII string we need to push out in a CAN wrapper?
        count = 0;
        while ((_tx_buffer_head != _tx_buffer_tail) && (count < 8)) {                           // Pull up to 8 characters from the buffer to send.
            buff[count]     = _cASCII_tx_buffer[_tx_buffer_tail];
            count++;
            _tx_buffer_tail = (_tx_buffer_tail + 1) % cASCII_TX_BUFFER_SIZE;
            }


        SetRVCPGNTerminal(N2kMsg, CAN_ASCII_reply, count, buff);
        NMEA2000.SendMsg(N2kMsg); 
    }
return(false);                                                                                  // We have already done SendMgs();
}




//----------------------------------------------------------------------------------------------------------
//  CAN ASCII Write 
// 
//      Places the passed string into the CAN-Terminal queue.  Note there is no check for overruns, so if the head
//      overtakes the tail, data will be overwritten.
//
//
void CAN_ASCII_write(char *stng) {
    int i;

    if (CAN_ASCII_source != 0) {                                                                // If no one has sent us anything, don't try to reply!
        CAN_ASCII_reply = CAN_ASCII_source;                                                     // We will be replying to the node who sent us the requesting ASCII text.

        i = 0;
        while (stng[i] != 0) {
            _cASCII_tx_buffer[_tx_buffer_head] = stng[i];
            i++;
            _tx_buffer_head = (_tx_buffer_head + 1) % cASCII_TX_BUFFER_SIZE;
        }
    }
}


//----------------------------------------------------------------------------------------------------------
//  CAN ASCII Tx Available 
// 
//      Returns how many characters are currently in the outbound (Tx) CAN ASCII Terminal buffer.  
//      Returns 0 if none.
//
//
//
uint8_t CAN_ASCII_TxAvailable(void) {

 return ((unsigned int)(cASCII_TX_BUFFER_SIZE + _tx_buffer_head - _tx_buffer_tail)) % cASCII_TX_BUFFER_SIZE;

}


#endif          /*** #ifdef SUPPORT_RVC  ***/











/*****************************************************************************
                        // ISO Diagnostics message - 1FECAh
                        // Input:
                        //  - On / Off
                        //  - Active / Standby
                        //  - DSA                       Default Source Address (Standard fault codes) 
                        //  - SPN                       Service Point Number  (Device Specific)
                        //  - FMI                       Failure Mode Identifier      
                        //  - Occurrence Count
                        //  - DSA Extension
                        //  - Bank Select
                        */
bool ISODiagnostics_message(tN2kMsg &N2kMsg){
    bool  onStatus;
    bool  errorRed, errorYellow;
    uint32_t  SPN = 0;                                          // Assume there is no fault.

    onStatus    = (chargingState != FAULTED); 
                   
    errorRed    = (chargingState == FAULTED);
    errorYellow = ((chargingState == FAULTED_REDUCED_LOAD) || (requiredSensorsFlag != 0)); // We are hurting a little, but still operational.

    if (errorRed | errorYellow)
      SPN = (uint32_t)faultCode | 0x70000;                      // Push up to the SPN our internal error number, using this large offset getting into the 'proprietary' range.
                                                                // Maybe later we can make this smarter, send out a series of SPNs during a fault...  
                                                                // Standard ones & our own custom ones.
    
    SetISODiagnosticsMessage(N2kMsg,onStatus,true,errorRed,errorYellow,
                                0,                              // unknown DSA    (Make this smarter later on?)
                                SPN,                            
                                ISOfmi_DVfni,                   // Preferred FMI when SPN is a proprietary one. 
                                0x7F,                           // Count unavailable
                                0xFF,                           // No extended DSA
                                0x0F);                          // No Banks defined
                          
    return (true);
}


bool ISODiagnosticsER_message(tN2kMsg &N2kMsg) {
    
    if ((chargingState == FAULTED)  || (chargingState == FAULTED_REDUCED_LOAD) || (requiredSensorsFlag != 0)) {
        ISODiagnostics_message(N2kMsg);
        return(true);
        }    
    else
        return(false);
   
}



//---   Forces out ISO Diag message, called from FAULT_HANDLER() to make sure the world knows we have faulted.
void send_ISO_Diag(void) {

    tN2kMsg     N2kMsg;
    
    ISODiagnostics_message(N2kMsg);                                                     // Make up the ISO Diag string .. 
    NMEA2000.SendMsg(N2kMsg);                                                           //  .. nd send it out.  
     
}
        
















//------------------------------------------------------------------------------------------------------
// Check CAN Messages
//
//      This function will check to see if there is any incoming messages.  If so, the appropriate call-back handler will be called.
//      Note that there are two classes of call-backs, those which are handled by the function handleCANMessager() below (ours), and also
//      some internal ones, esp the J1939 address reclaiming processes.
//
//
//------------------------------------------------------------------------------------------------------

void check_CAN(void){
    NMEA2000.ParseMessages();
}









//------------------------------------------------------------------------------------------------------
// Handle CAN Messages
//
//      These are the call-back functions registered into NMEA2000.ParseMessages();   They are called when a new CAN message has been received
//      or an external request has been made of us for a message. These functons function will parse the CAN message to decide if it is something
//      we care about and/or are able to respond to (All directed from the CANHandlers[] table), and if so will take the appropriate action.
//
//
//------------------------------------------------------------------------------------------------------

void handle_CAN_Messages(const tN2kMsg &N2kMsg){
    int i;

    for (i=0; CANHandlers[i].PGN!=0 && !(N2kMsg.PGN==CANHandlers[i].PGN); i++);             // Scan the vector table, looking for a matching PGN #

    if (CANHandlers[i].PGN==0)                                  return;                     // No PGN match found in the table
    if (CANHandlers[i].Receiver == NULL)                        return;                     // Entry does not have an Rx  message Rxhandling procedure assigned.
    if (CANHandlers[i].flag.N2K && !canConfig.ENABLE_NMEA2000)  return;                     // Message is a N2K one, and User has disabled NMEA2000 messages, perhaps due to conflict in the system.
    if (CANHandlers[i].flag.OSE && !canConfig.ENABLE_OSE)       return;                     // Message is an OSEnergy one, and user has disabled OSEnergy   
   
    CANHandlers[i].Receiver(N2kMsg);                                                        // Well, looks like we can support this incoming message.  Call the handler.
}




bool handle_CAN_Requests(unsigned long requestedPGN, unsigned char requester, int reviceIndex){
    tN2kMsg   N2kMsg;
    int i;

    for (i=0; CANHandlers[i].PGN!=0 && !(requestedPGN==CANHandlers[i].PGN); i++);           // Scan the vector table, looking for a matching PGN #

        if (CANHandlers[i].PGN==0)      return(false);                                      // We got to the end of the table, and no match was found
                                                                                            //  tell requester we do not support this PGN.
        return(invokeCANTx(CANHandlers[i], N2kMsg));                                        // Yes, we do support this message.  Try to send it out and tell 
                                                                                            //  request if we were not able to.
}






//******************************************************************************
//  Send the CAN Negitave Acknowledgement message - in response of them asking us to do something we do not want to do.
void sendNAK_handler(const tN2kMsg &msg){ 
    tN2kMsg   msgR;
    
    SetN2kPGNISOAcknowledgement(msgR,ISOat_NAK,0xff,msg.PGN);
    msgR.Destination  = msg.Source;                                                         // Direct the response to original requester.
    
    NMEA2000.SendMsg(msgR); 
    
}





#ifdef SUPPORT_NMEA2000
//*****************************************************************************
void N2kDCBatStatus_handler(const tN2kMsg &N2kMsg){                                         // NMEA-2000 Battery Status (RAT --> Remote battery Amps and Temperature)
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
        
    uint8_t  RATbatInstance;                                                                // allows use of NEMA2000 battery monitor if nothing else is available
                                                                                            // See users guide for cautions.
    
    double RATbatVolts;
    double RATbatAmps;
    double RATBatTempK;
    uint8_t SID;

    if (ParseN2kDCBatStatus(N2kMsg,RATbatInstance,RATbatVolts,RATbatAmps,RATBatTempK,SID)) {// Parse out the message

    if ((CAN_RBM_sourceID              == 0)        &&                                      // Only look at NMEA2000 RAT if we do NOT have a valid RBM via RV-C protocols
        (canConfig.ENABLE_NMEA2000_RAT == true)     &&                                      // Have we been configured to look to a NMEA-2000 device for Remote Amps and Temperature?
        (batteryInstance               ==  RATbatInstance)) {                               // And let's also make sure they think we are talking about the same battery.
        
        CAN_RAT2000_lastReceived = millis();                                                // Well then, lets note what time we got this information  (Will be used later in resolve_BAT_VoltAmpTemp();  )
        CAN_RAT2000_amps         = RATbatAmps;                                              //  tuck away the reported battery current.  (Also used later in resolve_BAT_VoltAmpTemp();  )
        CAN_RAT2000_temp         = (int) (RATBatTempK + 273.15);                            // Convert battery temp to unit type we use.
        }
    }
#endif
}

#endif                          /*** #ifdef SUPPORT_NMEA2000 ***/






//-----   Low-level basic J1939 messages, if we are attached to an engine
//       which sends them out we can pick up some details - specifically RPMs as opposed to using the Stator signal.
#ifdef SUPPORT_J1939


//*****************************************************************************

void J1939EngineSpeed_handler(const tN2kMsg &N2kMsg){                                   // J1939 Engine Speed - PGN 61444  
#if defined (OSE_ALTERNATOR)      
    int index;                                                                          // Thank you Jeremy M. for the concept!
    
    if (N2kMsg.PGN!=61444L) return;                                                     // Parsing this directly here, no Lib calls.

    index=3;                                                                            // Data of interest start at bytes 3 & 4
    J1939_Engine_RPMs              = N2kMsg.Get2ByteUInt(index) / 8;                    // 8x actual RPM
    if (canConfig.ENGINE_INSTANCE != N2kMsg.GetByte(index)) 
        return;                                                                         // Not us.

    J1939_Engine_RPM_received      = millis();                                          // Was us!  Take note of the MILLIS() when we last recevied a valid RPM for this engine.
#endif
}


#endif /* Support J1939 */










#ifdef SUPPORT_RVC

//------------------------------------------------------------------------------------------------------
// Validate CAN
//
//      This is a key function.  It is used when receiving a message to decide if we should pay attention to that message.
//      In doing so it will also decide if the regulator should be placed into 'slave' mode to a smarter master associated with
//      the battery we are attached to.
//
//      A critical global variable:  CAN_RBM_sourceID will indicate if we are linked into a remote device by containing that
//      devices SID.  If however we do not trust any remote devices (Because we have not been hearing from them consistently enough,
//      or they have a lower priority then us), the CAN_RBM_sourceID will be set = 0 to indicate we should work alone.
//      (Or perhaps become the master, but that is decided in decide_if_CAN_RBM()
//
//
//------------------------------------------------------------------------------------------------------

 bool validate_CAN(uint8_t instance, uint8_t devPri, uint8_t sourceID, uint8_t mask) {      // Is this message from someone we should be taking directions from?


    if (instance != batteryInstance)  return(false);                                        // Message must be for the same battery we are attached to . .




    //----  First we will look to see if this device is smarter then us.  If it is not, 
    //      and we are so configured, WE should be putting our hat into the ring to become
    //      the RBM.  And as such should not validate this node.
    //
    //      Do take special note that if we are just starting out (Ramping, or have JUST 
    //      transitined into Bulk) we will listen to what someone else has to say.
    //      We do this to allow for a smooth handoff of the current battery state, specificly the
    //      charging Mode the battery is in, before we take over.


       if ((canConfig.CONSIDER_MASTER) && (chargingState >= bulk_charge)) {                     // We are ready and willing to step up to the plate!
            if (devPri   <  canConfig.DEVICE_PRIORITY)                                          //   As such, this incomming node must have a priority (smartness) better then us.
                return(false);                                                                 
            if ((devPri == canConfig.DEVICE_PRIORITY) && (sourceID < fetch_CAN_localID()))      //   Or if just as smart, they must at least out rank use.
                 return(false);   
        }



    //----  We are going to now validate the trust in this remote device trying to tell us what to do.
    //      Does it have a higher 'prioity' then us?  Have we been hearing from it for a while?

    if ((((sourceID == CAN_RBM_sourceID) || (sourceID == CAN_RBM_potentialSourceID)) &&     // VALIDATE consistency of received messages.
          (millis() - CAN_RBM_lastReceived) >= REMOTE_CAN_MASTER_TIMEOUT)) {                // Has it been a while since our 'master' (or proposed master) has talked to us?
            invalidate_RBM();                                                               // Been a while, too long in fact.  Stop listening to everyone and look for a new 'master' to follow.
            return(false);
            }



    if ((( devPri  > CAN_RBM_devicePriority)      ||                                        // We potentially have a new and smarter 'master'!
         ((devPri == CAN_RBM_devicePriority) && (sourceID > CAN_RBM_sourceID)))             // (or one that is just as smart, but out ranks the current one)

       &&

        (( devPri  > canConfig.DEVICE_PRIORITY) ||                                          // But only if they are smarter than us..
         ((devPri == canConfig.DEVICE_PRIORITY) && (sourceID > fetch_CAN_localID()))  ||    //  (Well, if not smarter, at least as smart, and out ranks us)
          (canConfig.CONSIDER_MASTER == false))                                             //  (UNless we do not want to be the RBM, in which case we don't care how smart they are vs. to us)
       
       &&

        (sourceID !=  CAN_RBM_potentialSourceID)) {                                         // AND it is not the one we are already considering..
            CAN_RBM_potentialSourceID  = sourceID;                                          // Someone new!  Let's remember him,
            CAN_RBM_devicePriority     = devPri;                                            // ..  How smart he is,
            CAN_RBM_validatedTime      = millis();                                          // .. and when we 1st meet this guy.
            CAN_RBM_messages           = 0;                                                 // Clear the flag register on which messages we have received from this guy.
            CAN_RBM_desiredChargeState = RVCDCbcm_Undefined;
            }



    CAN_RBM_lastReceived   = millis();                                                      // Take note that someone we are watching is periodically talking to us.
    CAN_RBM_messages      |= mask;                                                          // Note also which message they have sent us.  (We need to get all of the key ones before 'approving' him.

    if ((millis() - CAN_RBM_validatedTime) <= REMOTE_CAN_MASTER_STABILITY)                  // But have they talked to use for a sufficient period of time?
        return(false);                                                                      // Still in the honey moon period, don't listen to anyone just yet.

     //---- OK, we have a potential master.  They are smarter than us (or as smart, but higher ID), they have been talking to us for the required time period.  Now.
     //     have they been telling us what we need to know during this honey-moon period?  (Meaning, have they sent us the needed RV-C DC messages)?
     if ((CAN_RBM_messages & (FLAG_DC1 | FLAG_DC5))  != (FLAG_DC1 | FLAG_DC5)) {            // We want Battery Volts, Amps, and Temp messages - as min.
          CAN_RBM_potentialSourceID = 0;                                                    // Nope - so let's forget this guy and start all over again.
          return(false);                                                                    //   (Likely will get the same guy, in which case the system will never 'loc in' if top dog is spamming out false info.
          }

    CAN_RBM_sourceID           = CAN_RBM_potentialSourceID;                                 // If we get this far it seems we have a good master to listen to.  Let's remember who they are,
    return(true);                                                                           // and go ahead and listen to them.  

}



//*****************************************************************************
void RVCDCStatus1_handler(const tN2kMsg &N2kMsg){                                           // DC Status 1  (Remote Battery Current)        
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    uint8_t  instance;                                                                      
    uint8_t  devPri;                                                                    
    uint16_t Vdc;
    uint32_t Adc;
 
    if (ParseRVCDCSourceStatus1(N2kMsg, instance, devPri, Vdc, Adc)) {                      // Received a valid CAN message from someone
                
        if ((validate_CAN(instance, devPri, N2kMsg.Source, FLAG_DC1)) &&                    // Is it from someone we should be listing to?
            (Adc != N2kUInt32NA)) {                                                         // And do they actually have a current value to tell us?
              
             CAN_RBM_amps = (float)((int32_t)(Adc - 0x77359400)) * 0.001;                   // Yup - save this info, will be processed in resolve_BAT_VoltAmpTemp();
             CAN_RBM_ampsRefreshed = millis();
        }
    }
    
#endif   
}


//*****************************************************************************
void RVCDCStatus2_handler(const tN2kMsg &N2kMsg){                                           // DC Status 2  (Remote Battery Temperature)
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
        
    uint8_t  instance;
    uint8_t  devPri;
    int16_t  sourceTemp;
    uint8_t  SOC;
    uint16_t TR;

   
    if (ParseRVCDCSourceStatus2(N2kMsg, instance, devPri, sourceTemp, SOC, TR) &&           // Received a valid CAN message from someone
        validate_CAN(instance, devPri, N2kMsg.Source, FLAG_DC2)) {                          // Is it from someone we should be listing to?

        if ((CAN_RBM_sourceID == N2kMsg.Source) &&  (sourceTemp != N2kInt16NA)) {           // Is this THE Remote Battery Master we are listing to?   And did they care to tell us the battery temperature?
          CAN_RBM_temp = (int)((float)sourceTemp / 0.03125);                                // They did!
          CAN_RBM_tempRefreshed = millis();
        }
        else
          CAN_RBM_temp = -99;                                                                // signal that nothing valid has come from a RBM
    } 
       
#endif
}




//*****************************************************************************
void RVCDCStatus4_handler(const tN2kMsg &N2kMsg) {                                          // DC Status 4 (Desired Charge Mode)
   
    uint8_t         instance;
    uint8_t         devPri;
    tRVCBatChrgMode desCM;
    uint16_t        desVolt;
    uint16_t        desAmp;
    tRVCBatType     batType;

   
     if (ParseRVCDCSourceStatus4(N2kMsg, instance, devPri,  desCM, desVolt, desAmp, batType) && // Received a valid CAN message from someone
        validate_CAN(instance, devPri, N2kMsg.Source, FLAG_DC4)) {                              // Is it from someone we should be listing to?
                                                                                                //  (Side note, even if we are a BatMonitor or BMS, we receive these DCStatusx messages, to allow for the cheeck to 
                                                                                                //   see if there is a RBM out there with a higher prioity then us.  But we ignore the values received.)
            #if defined (OSE_ALTERNATOR)  || \
                defined (OSE_GENERATOR)
                if ((chargingState >= post_ramp) || (chargingState == disabled)) {                  // If we are ramping, faulted, or determining alt size - leave it alone.
                    switch (desCM) {                                                                // But if is into the charging phases, take direction from the RBM
                            case RVCDCbcm_Bulk:             set_charging_mode(bulk_charge);
                                                            break;
    
                            case RVCDCbcm_CVCC:             set_charging_mode(RBM_CVCC);
                                                            break;
    
                            case RVCDCbcm_Absorption:       set_charging_mode(acceptance_charge);
                                                            break;
    
                            case RVCDCbcm_Overcharge:       set_charging_mode(overcharge_charge);
                                                            break;
    
                            case RVCDCbcm_Float:            set_charging_mode(float_charge);
                                                            break;
    
                            case RVCDCbcm_Equalize:         set_charging_mode(equalize);
                                                            break;
    
                            case RVCDCbcm_Disabled:         set_charging_mode(disabled);
                                                            break;
    
                            default:                        if ((CAN_RBM_desiredChargeState != RVCDCbcm_Undefined)  && (CAN_RBM_desiredChargeState != RVCDCbcm_Unknown))
                                                                set_charging_mode(ramping);              // Well now, the RBM is not telling us what to do, but at some time they had.
                                                                                                    // Looks like we are on our own, start things all over by going into Ramping phase..
                                                            break;                                  // Otherwise, we just leave things as they currently are - and let the Alternator do its thing.
                            }   /* End of switch/case */
                  }
    
                CAN_RBM_desiredVolts       =  (float) desVolt * 0.050;
                CAN_RBM_desiredAmps        =  (float)((int)(desAmp - 0x7D00)) * 0.050;
                CAN_RBM_desiredChargeState = desCM;                                             // We should take note if the desired mode is charging, or disable...
                canConfig.BATTERY_TYPE     = batType;
            #endif
          }
}



//*****************************************************************************
void RVCDCStatus5_handler(const tN2kMsg &N2kMsg){                                           // DC Status 5 (Remote Battery Voltage)
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
        
    uint8_t   instance;
    uint8_t   devPri;
    uint32_t  Vdc;
    uint16_t  dVdT;

   
    if (ParseRVCDCSourceStatus5(N2kMsg, instance, devPri,  Vdc,  dVdT) &&                   // Received a valid CAN message from someone
        validate_CAN(instance, devPri, N2kMsg.Source, FLAG_DC5)) {                          // Is it from someone we should be listing to?

        CAN_RBM_volts = (float)Vdc * 0.001;                                                 // Yup - save this info, will be processed in resolve_BAT_VoltAmpTemp();
        CAN_RBM_dVdT = dVdT;
        CAN_RBM_voltsOffset = CAN_RBM_volts - measuredAltVolts;                             // Store away the offset snapshot at this moment.
        CAN_RBM_voPWMvalue  = fieldPWMvalue;
        CAN_RBM_voltsRefreshed = millis();
        }
        
#endif
}




//*****************************************************************************
void RVCDCStatus6_handler(const tN2kMsg &N2kMsg){                                           // DC Status 6 (Battery BMS disconnects)
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
        
    uint8_t   instance;
    uint8_t   devPri;
    bool HVls;
    bool HVld;
    bool LVls;
    bool LVld;


    if (ParseRVCDCSourceStatus6(N2kMsg, instance, devPri, HVls, HVld, LVls, LVld)) {
                                                                                            // Received a valid CAN message from someone
                                                                                            // Note that for these disconnect messages we do not care who sent them!

        if (HVls) {                                                                         // Battery is at its High Voltage Limit, we should stop charging.
            set_charging_mode(forced_float_charge);                                         // Force battery into Float mode, try to slow things down.
            }



        if (HVld) {                                                                         // DO SOMETHING NOW!!!  the battery has been disconnected!
            set_ALT_PWM(0);                                                                 // Shut down the PWM ASAP.

            chargingState = FAULTED;
            faultCode     = FC_CAN_BATTERY_HVL_DISCONNECTED;
            }


        }
#endif
}



//*****************************************************************************
void RVCChrgStat_handler(const tN2kMsg &N2kMsg){                                                // Charger Real-time Status message #1
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)

     uint8_t perMax, chrgInst, flag;                                                         
     uint16_t volts, amps; 
     tRVCChrgType  chrgType;
     tRVCBatChrgMode state;
     tRVCChrgForceChrg forcedChrg;
     bool    defPOS, autoRechg; 

  
    if (ParseRVCChargerStatus(N2kMsg, chrgType, chrgInst, volts, amps, perMax, state, defPOS, autoRechg, forcedChrg)) { // Received a valid CAN message from someone?
        if (perMax != N2kUInt8NA) {                                                             // And do they choose to tell us their utilization?  (Critical to do prioritization)
            
            flag = 0x01<<(N2kMsg.Source&0x07);
 
            if (((chargerSBHP[N2kMsg.Source>>3] & flag) != 0) && (perMax < 80))                 // Higher Priority charger on the same battery?  -- being under-utilized?
                CAN_HPUUCS_lastReceived = millis();                                             // Yup - take note of this situation. 



            if ((chargerSBEP[N2kMsg.Source>>3] & flag) != 0)  {                                 // Equal Priority charger on the same battery?
                CAN_EPCS_lastReceived   = millis();                                             // Yup -- 
                
                 if (average_EPC_utilization == 0)                                              // And we also want to record average utilization of our peers.
                    average_EPC_utilization = ALT_Per_Util();                                   //  if 1st time in, use this as the 'starting point'.
                
                average_EPC_utilization += perMax;                                              //  and keep a running average utilization of all equal charging sources!
                average_EPC_utilization /= 2;  //!! HEY!!  THIS SHOULD BE THE COUNT OF EQUAL DEVICES WE KNOW OF, NOT JUST A FIXED 2!!!!
  
            }


            
            if (((chargerSBLP[N2kMsg.Source>>3] & flag) != 0) && (perMax > 20))                 // Lower  Priority charger on the same battery?  -- is it contributing anything?
                CAN_LPCS_lastReceived = millis();                                               // Yup - take note of this situation. 
        }
    }

#elif defined (OSE_BMS) || \
      defined (OSE_BATMON)

   //!! HEY!!       Add some code here - we can use the reception of a Charger Active message to enter higher power states on the BatMon, or BMS.

#endif       
}



    
    

//*****************************************************************************
void RVCChrgStat2_handler(const tN2kMsg &N2kMsg){                                                    // Charger Real-time Status message #2
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    uint8_t  chrgInst, DCInst, flag;
    tRVCChrgType  chrgType;
    uint8_t  devPri;
    uint16_t Vdc;
    uint16_t Adc;
    uint8_t  temp;

    
    if (ParseRVCChargerStatus2(N2kMsg, chrgType, chrgInst, DCInst, devPri, Vdc, Adc, temp)) {       // Received a valid CAN message from someone?
     
        if (DCInst == batteryInstance) {                                                             // Is this someone charging the same battery as us?
            flag = 0x01<<(N2kMsg.Source & 0x07);                                                    // All we are going to do here is note the linkage between this charger and the battery instance.
 
            if (devPri  > canConfig.DEVICE_PRIORITY)    chargerSBHP[N2kMsg.Source>>3] |=  flag;     // Higher priority?  Yes, set bit-array for additional checking in RVCChrgStat_handler()
            else                                        chargerSBHP[N2kMsg.Source>>3] &= ~flag;
            
            if (devPri == canConfig.DEVICE_PRIORITY)    chargerSBEP[N2kMsg.Source>>3] |=  flag;     // Equal priority?
            else                                        chargerSBEP[N2kMsg.Source>>3] &= ~flag;
            
            if (devPri <= canConfig.DEVICE_PRIORITY)    chargerSBLP[N2kMsg.Source>>3] |=  flag;     // Lower priority?
            else                                        chargerSBLP[N2kMsg.Source>>3] &= ~flag;
        }
        else {
            chargerSBHP[N2kMsg.Source>>3] &= ~flag;                                                 // Charger is not even on our same battery
            chargerSBEP[N2kMsg.Source>>3] &= ~flag;
            chargerSBLP[N2kMsg.Source>>3] &= ~flag;
        }


    #ifdef SIMULATION
         simCAN_Ext_amps =  (float)((int)(Adc - 0x7D00)) / 20.0;                                    // Stash away any amperage being 'produced' by another attached device.
         #endif



    }
    
#endif    
}





//*****************************************************************************
void RVCDCDisconnectStatus_handler(const tN2kMsg &N2kMsg){                                   // DC Disconnect Status
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    uint8_t   instance;
    bool cirCon;
    bool recDisCom;
    bool bypassed;

 
    if (ParseRVCDCDisconnectStatus(N2kMsg, instance, cirCon, recDisCom, bypassed)) {        // Received a Disconnect Status CAN message from someone
                                                                                            // Note that for these disconnect messages we do not care who sent them!


        if ((instance == batteryInstance) &&                                                // Are they talking about OUR battery????
            ((!cirCon) || (recDisCom))) {                                                   // Yes, and the battery has been disconnected, or will be soon!
            set_ALT_PWM(0);                                                                 // Shut down the PWM ASAP.

            chargingState = FAULTED;
            faultCode       = FC_CAN_BATTERY_DISCONNECTED;
            }


        }
        
#elif defined (OSE_BMS) || \
      defined (OSE_BATMON)
        
   //!! HEY!!       Whoa!   Someone else is disconnecting OUR battery!!!  Add some code here 

#endif       
}




//*****************************************************************************
void RVCDCDisconnectCommand_handler(const tN2kMsg &N2kMsg){                                 // DC Disconnect Command
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
    uint8_t   instance;
    bool      disCmd;


     if (ParseRVCDCDisconnectCommand(N2kMsg, instance, disCmd)) {                           // Received a Disconnect Command CAN message from someone
                                                                                            // Note that for these disconnect messages we do not care who sent them!

       if ((instance == batteryInstance) &&                                                 // Are they talking about OUR battery????
           (disCmd)) {                                                                      // Yes, and a Disconnect command has been issued for our battery!
            set_ALT_PWM(0);                                                                 // Shut down the PWM ASAP.

            chargingState = FAULTED;
            faultCode       = FC_CAN_BATTERY_DISCONNECTED;
            }

        }

        
#elif defined (OSE_BMS) || \
      defined (OSE_BATMON)
        
  //!! HEY!!        Whoa!   Someone else is disconnecting OUR battery!!!  Add some code here 

#endif 
}









//*****************************************************************************
void RVCGenCommand_handler (const tN2kMsg &N2kMsg) {
#ifdef OSE_GENERATOR
    
        if (!ParseRVCGeneratorCommand(N2kMsg, CANGenRequest)) {
            CANGenRequest = RVCGst_Unknown; 
            };
        
           
 #endif   
}
 








//*****************************************************************************
void RVCTerminal_handler(const tN2kMsg &N2kMsg){                                            // Someone is sending us an ASCII text string!
    int i;
    
    _rx_buffer_tail = 0;                                                                    // We will be placing data into the Rx buffer from 0.

    if (ParseRVCPGNTerminal(N2kMsg, CAN_ASCII_source, i, _cASCII_rx_buffer))                // We can only handle one-packet at a time, so go ahead directly copy the CAN message into out local Rx buffer
        _rx_buffer_head = i;
    else
        _rx_buffer_head = 0;                                                                // If it was not a valid message, clear the flag that anyone is talking to us via ASCII

}





//----------------------------------------------------------------------------------------------------------
//  CAN ASCII Read 
// 
//      If a character is available in the CAN termianl inbound queue, returns it.  Returns 0 if no character is available
//
//
//
char CAN_ASCII_read(void) {

 
  if (_rx_buffer_head == _rx_buffer_tail) {                                         // Head = tail --> queue is empty.
    return (0);
  } else {
     char c = _cASCII_rx_buffer[_rx_buffer_tail];
    _rx_buffer_tail = (_rx_buffer_tail + 1) % cASCII_RX_BUFFER_SIZE;
    return (c);
  }

}



//----------------------------------------------------------------------------------------------------------
//  CAN ASCII Rx Available 
// 
//      Returns how many characters are avaialbe in the inbound (Rx) CAN Terminal buffer.  
//      Returns 0 if none.
//
//
//
uint8_t CAN_ASCII_RxAvailable(void) {

 return ((unsigned int)(cASCII_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % cASCII_RX_BUFFER_SIZE;

}


#endif              /***  #ifdef SUPPORT_RVC   ***/










//------------------------------------------------------------------------------------------------------
// Decide CAN Remote Battery Master
//
//      This function will decide if we need to become the master controlling node for the CAN system - the RBM.
//      The primary purpose of the RBM node is to report the current battery voltage and amps,and charging needs.
//      If this node is the highest Priority transmitting node for a battery instance, it will become the master.  Else, it will let someone
//      smarter do that.
//
//      When installing different devices, the user has the ability to set a nodes 'priority' and care should be taken to make sure there is
//      a good ordering of that priority - in that nodes with good instrumentation of the battery should be higher priority.  This is especially key
//      for devices such as this regulator which can be configured in different ways, from simple with no direct battery insight, to a fully connected
//      one.
//
//  
//
//
//------------------------------------------------------------------------------------------------------

void decide_if_CAN_RBM(void) {
    
 CAN_weAreRBM = false;                                                                   // Assume we do not want to become the master. 
                                                                                         // (As well if we are not including RV-C support)   
#ifdef SUPPORT_RVC
    if (CAN_RBM_sourceID          != 0)  return;                                        // If someone else is already validated as the master, don't even look.
    if (CAN_RBM_potentialSourceID != 0)  return;                                        // Don't look if we are also considering someone else already.  Let that play out.
    if (!canConfig.CONSIDER_MASTER)      return;                                        // Don't try if we have been configured to not even try-out...
    if ((millis() - CAN_RBM_lastReceived) <= (RBM_REMASTER_IDLE_PERIOD) + ((255 - canConfig.DEVICE_PRIORITY) * RBM_REMASTER_IDLE_MULTI))
                                         return;                                        // And if it not been long enough that there has been no master, just hold off for a while.
                                                                                        // Take note that while waiting, we hold off an inverse amount of time relative to our own priority.
                                                                                        // This way 'smarter' devices will start to try to take over before the dumber ones.

    CAN_weAreRBM = true;                                                                // If we made it this far, then yes - we are the master.
                                                                                        // Lets give it a whirl.  If someone else 'smarter' comes alone later, the will knock us off the hill
                                                                                        // during read validation  via validate_CAN().
#endif
}




//------------------------------------------------------------------------------------------------------
// Invalidate CAN Remote Battery Master
//
//      If it is decided that we will no longer be able to follow the RBM (perhaps because it stopped talking to us), this function will 'release' the regulator
//      to resume operation under its own decisions.  
//      Is it also called during startup to prime key variables.
//
//
//------------------------------------------------------------------------------------------------------

void invalidate_RBM(void) {

    CAN_RBM_sourceID           = 0;                                                     // Clear the global flags.  The 'We are listing to THIS master flag'
    CAN_RBM_potentialSourceID  = 0;                                                     // And reset the potential master variables as well - start looking from fresh. 
    CAN_RBM_devicePriority     = 0;
    CAN_RBM_desiredChargeState = RVCDCbcm_Undefined;
    CAN_RBM_lastReceived       = 0;                                         
    CAN_RBM_validatedTime      = 0; 
    ignoringRBM                = false;
    
    #if defined (OSE_ALTERNATOR)  || \
        defined (OSE_GENERATOR) 
           
        if (chargingState == RBM_CVCC)  
                set_charging_mode(ramping);                                             // And if the RBM did not leave us in a valid charging mode just start over again @ Ramping.
    #endif

}





//------------------------------------------------------------------------------------------------------
// Resolve Remote Insturmentaiton Voltage, Amperage, and Temperature 
//
//      If we are attached to a Remove Battery master who is providing detailed battery instrumentation, this function will adjust the passed 
//      battery Voltage, Amperage, and Temperature based on the remote instrumentation information.  It functions by calculating an offset value
//      and using that to make an adjustment to the locally measured data values.  If there is no linkage for remote instrumentation, no adjustment
///     is made.  
//  
//     Before calling this function 1st measuring locally the battery voltage, amperage, and temperature.  Pass those parameters into this function
//      and it will apply the remote battery adjustments as appropriate.
//
//      TRUE will be returned of a valid Remote Instrumentation adjust is made, else FALSE will be returned indicating the local device is on its own.
//   
//        (A note:  The resulting code size what the SAME if Volts/Amps/Temp pointers were passed vs. accessing the global variables directly.)
//        (         As a result, I left this coded to pass parameters - maybe a bit more clear WHAT is being impacted, and might help in future efforts to generalize this into a library.)
//
//------------------------------------------------------------------------------------------------------
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)  
    
bool resolve_RI_VoltAmpsTemp(float* batVolts, float* batAmps, int* batTemp) {
    uint32_t enteredMills;                                                                     // Time in millis() entered (This is used a lot, and having a local var saves code space)  
    bool results;
    
    results = false;                                                                                // Assume we will not be able to make a remote instrumentation linkage.
    enteredMills = millis();                                                                        // Used throughout function and we need to have a consistent value between calculations. 

                                                                                                 
    if ((CAN_RBM_sourceID != 0) && (!ignoringRBM)){                                                 // Is there an OSEnergy (RV-C) device we are listing to?
        
        //---- Does it look like the RBM is still paying attention to us?  (ala, still sending us directions and status?)
        if ((enteredMills - CAN_RBM_lastReceived) >= REMOTE_CAN_MASTER_TIMEOUT)                     // Nope, been too long.
            invalidate_RBM(); 
        else {                                                                                      // Still talking.  OK to check the rest now.

      
            //----  Look at Remote Instrumentation of Amps.
            if ((enteredMills - CAN_RBM_ampsRefreshed) < REMOTE_CAN_MASTER_TIMEOUT) {               // Validate that we received an Amps message recently
                if (abs(CAN_RBM_amps) <= MAX_SUPPORTED_SYSTEM_AMPS) {
                   *batAmps = CAN_RBM_amps;                                                         // Amps move slowly.  Just use the passed value for the next time period until a new one arrives.
                    results = true;
                } else {
                    #ifndef SIMULATION 
                    ignoringRBM = true;                                                             // Whoa here!  WHAT is this guy saying??  So many amps??  Seems nuts.  We will ignore this guy from now on.
                    #endif                                                                          // (If we get such a high value, it is an indication something is very wrong in the system - so do not trust it 
                    }                                                                               //   until it has been resolved)
            }
            

            //----  Look at Remote Instrumentation of Temperature.
            if ((enteredMills - CAN_RBM_tempRefreshed) < REMOTE_CAN_MASTER_TIMEOUT) {               // Validate that we received a Temperature message recently
                 if ((CAN_RBM_temp > -60) && (CAN_RBM_temp < FAULT_BAT_TEMP)   &&                   //  Do some level of data validation.
                     (CAN_RBM_temp != -99))
                    *batTemp = CAN_RBM_temp;                                                        //  and if seems somewhat reasonable, go ahead and use it
            }
                                                                                                    // Take note that if the temperature is wacky we just do not use it, but will not let that totally invalidate the RBM.
                 


                   
            //----- Look at Remote Instrumentation of Volts
            if ((enteredMills - CAN_RBM_voltsRefreshed) < REMOTE_CAN_MASTER_TIMEOUT) {              // Validate that we received a Volt message recently
                if ((abs(CAN_RBM_voltsOffset) <= MAX_SUPPORTED_SYSTEM_VOFFSET)                && 
                    (CAN_RBM_volts            <= (FAULT_BAT_VOLTS_EQUALIZE * systemVoltMult)) && 
                    (CAN_RBM_volts            >= (FAULT_BAT_VOLTS_LOW      * systemVoltMult))) {
                    
                    if (fieldPWMvalue < CAN_RBM_voPWMvalue)
                        *batVolts = *batVolts + (CAN_RBM_voltsOffset * (fieldPWMvalue-thresholdPWMvalue)/(CAN_RBM_voPWMvalue-thresholdPWMvalue));
                                                                                                    // Applying Ohms Law here, if we have dramatically lowered the PWM from when we last received a Remote Instrument VBat,
                                                                                                    // we should 'anticipate' the offset will also be lower.
                                                                                                    // Without doing this correction we will tend of over-drive the alternator, and overshoot the VBat target.
                                                                                                    // This is a rough-n-ready correction, but without this even in simple systems lowering the PWM dramatically can cause 100mV to even 200mV
                                                                                                    // delta; and given we try to regulate to a +/- 10mV goal, that is massive....
                    else
                        *batVolts = *batVolts + CAN_RBM_voltsOffset;                                // Only apply a amperage based reduction factor with lowered PWMs.  Never do an uplift!  
                                                                                                    //   (Better to undershoot until a new remote offset value is received..)
                                                                                                    
                    
                } else {
                    #ifndef SIMULATION
                    ignoringRBM = true;                                                             // Whoa here!  WHAT is this guy saying??  Volts out of range, BIG offsets?  We will ignore this guy from now on.
                    #endif                                                                          // (If we get such a odd values, it is an indication something is very wrong in the system - so do not trust it 
                }                                                                                   //   until it has been resolved)
            }
        }
    }
    else {                                                                                          // We got no RBM.  Maybe user has forced us to listen to a NMEA-2000 battery monitor?  
        if ((enteredMills - CAN_RAT2000_lastReceived) < REMOTE_CAN_RAT2000_TIMEOUT) {               // Validate that we received a NMEA-2000 battery status message recently
            if (abs(CAN_RAT2000_amps) <= MAX_SUPPORTED_SYSTEM_AMPS) {
               *batAmps = CAN_RAT2000_amps;                                                         // Looks like it, and does not seem unreasonable.  So let's use it.
                results = true;
            }
        }
    }

    return (results);
}   
   
#endif
    
    



//------------------------------------------------------------------------------------------------------
// CAN Local ID
//
//      Returns the ID (8-bit Address) presently assigned to this device.
//      Note that the ID may change if there is a conflict in the system.
//
//
//------------------------------------------------------------------------------------------------------

uint8_t fetch_CAN_localID(void){

    uint8_t localID;

    localID = NMEA2000.GetN2kSource();                              // Fetch CAN address we are currently assigned to.

    if (localID != canConfig.LAST_CAN_ID) {
        canConfig.LAST_CAN_ID = localID;                            // We have been moved!  Remember our new 'address', so that next time
        write_CCS_EEPROM(&canConfig);                               // we start up we can try to be at the same place folks knew us to be.
    }

    return(localID);
}


 #endif //SYSTEMCAN









