//------------------------------------------------------------------------------------------------------
//
//
//
// Alternator Regulator based on the Arduino IDE
//
//      Copyright (c) 2016, 2017, 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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
//
//
//
//    This software is a 3/4-stage alternator regulator.  A fundamental difference between this and currently available
//      regulators is the ability to monitor Amps being delivered to the battery as well as Voltage.  By doing so more intelligent 
//      charging transitions can be made, resulting in shorter charging times and longer battery life.
//
//      This work is derived from the Smart Engine Control and Alternator Regulator project, and takes the same approach
//      for alternator regulation, but removes all the engine control and integration. http://smartdcgenerator.blogspot.com/
//
//
//    Default special (not via DIP switch, see #defines in SmartRegulatro.h or Configuration) user features are selected by:
//        - Shorting NTC sensor the ALTERNATOR will place regulator in 1/2 Alternator amps mode.
//        - During operation, tying feature-IN input HIGH will enable Equalize mode..
//              (Except if Charge Profile #8 LiFePO4 is selected, in that case a HIGH feature-in will cause regulator to stop charging (force to float))
//
//
//
//
//
//
//
//
//      A note on this source:  In 2016 the 3rd generation of the Arduino Alternator Regulator was made available, one that was designed to operate as a 'system' 
//        utilizing a CAN bus to communicate.  Though the core functionality is the same there are several hardware changes.  This source code is designed to support both
//        gen 2&3 through compile-time flags.  It uses the target CPU ID (ATmega328 for generation 2, ATmegaxxM1 for generation 3) to select changes in the source.
//        Look for the key defines:  STANDalong and SYSTEMCAN throughout the code.  Other decisions are based on a presence test of a feature, example:  the Battery NTC
//        code is selected based on the presence of BAT_NTC_PORT, as opposed to STANDALONE.  It is hoped this will be somewhat more readable.   I will also comment, not 
//        every small detail is bracketed with #ifdefs, example some small variables I just leave - rather than clutter up the source code with too many #ifdefs.
//
//        Please also note that with the new modularized approach to the source (as opposed to one very very int32_t file), Arduino IDE 1.6.8 or greater must be used.  Else the preprocessor
//        will not be able to correctly arrange the include files and you will get compiler errors!!!
//
//
//
//
//
//
//
//
//      07/27/2018  v1.3.1      Improved load-dump recovery, OSEnergy_DASH support, feedback on dual alts.
//      05/27/2018  v1.3.0      Removed $EBA, Refined dual alts load bal, Required sensors fault check, Disabled "Adaptive Acceptance" if no shunt, Improved RPMs, FeatIn toggle check (equalzie), 
//      02/21/2018  v1.2.1      Added Firefly to CPE#6, fixed Float/Amp bug, Special check for Time&Amps=0 in Acceptance
//      01/30/2018  v1.2.0      CPEs adjusted, Correct Euqalize lockout bug, DIP swithces 250A steps, improved Feature-in() reset check, 
//      11/26/2017  v1.1.2      Corrected: Float-to-bulk triggers - amps, default Ah in CPE#8, RBM validation if we are not RBM enabled,    
//      09/30/2017  v1.1.1      Code size reduction, improved Alt Temp reg, verify shunt working before active regualtion in FLOAT, J1939 engine RPMs, NMEA2000 engine RPMs.    
//      07/13/2017  v1.1.0      Removed 'Favor-32v' flag and 32v autoselect, hold in preramp state if we are receiving ASCII config commands.
//                              Changed from Deg F to Deg C, added Tach-mode flag to $SCT:, added PID_VOLTAGE_SENS (margin, better support of small alternators + large bats)
//      05/15/2017  v1.0.2      Remember assigned CAN node ID, corrected Amp-shunt cal
//      01/28/2017  v1.0.1      Revised SST; to place version number at beginning of string.
//      01/25/2017  v1.0.0      Add support for CAN enabled version of regulator, corrected "$SCO:" command not recognized. Improved auto idle detection and idle field pull-back formula,
//                              Logic to manage A&B Temp ports as well FET NTC sensor.
//                              Changes to API:  $AST; ==> Added VAlt, Fld%, Alt2Temp & FETTemp,  $SCV; ==> Added Idle RPMs value,  $SCA ==> Manage idle RPMs configuration.
//                                               $CCN; ==> New CAN configuration command,   $SCT: accepts % value vs. raw PWM.
//                              Modularized files - REQUIRES USE OF ARDUINO IDE1.6.8 or above.
//                              Corrected bug: 0 amps in CPE for EQUAL_BAT_A_SETPOINT --> Disabled Amp limits during Equalization, but allows equalization.
//                              Check for overvoltage during RAMPING
//                              Improved PID tuning (Voltage), and load-dump handling.
//                              Favor 36v changed to favor32v
//                              Corrected several ASCII command bugs
//
//            
//
//      xx/xx/2016  v0.1.9      Increased idle PWM pull-back gain, corrected Alt Gain ratio
//      03/27/2016  v0.1.8      corrected "$SCO:" command not recognized. 
//      11/10/2015  v0.1.7      Corrected bug in Alternator Temperature regulation code, Refined Alt Temp PID values to be more aggressive. 
//      07/07/2015  V0.1.6      Corrected fatal coding error in manage_alt();, CPE FORCE_TO_FLOAT ASCII voltage reporting, FORCED_FLOAT and TACH mode improvements, 
//                              Changed SST; to show version number,  Warmup duration shorted to 30 seconds, improved NTC sensor range, 
//      03/04/2015  v0.1.5           <WITHDRAWN, FATAL ERROR IN MANAGE_ALT()>   
//                                   Refined PID tuning for Volts/Amps/Watts. Corrected Acceptance time-based exit w/Amp shunt working, verified Arduino IDE 1.6.0
//      02/13/2015  v0.1.4      PD to PID engine, correct Amp-offset, Vout rounding error.
//      10/12/2014  v0.1.3      Improved "Combiner" code, Feature-in 'force-to-float' option, added AmpHour exit criteria to Float/pst-float, 
//                              Allow Floating numbers for Volts and Amps multipliers - greater flexibility, reverse Amp Shunt polarity ASCII flag, 
//      06/20/2014  v0.1.2      Correct INA226 Amp Reg, improved Bluetooth code. 
//      05/17/2014  v0.1.1      Adaptive Acpt Mode time based exit, new ASCII commands, Stall detect and PWM cap, Field drive capping when stalled.
//      04/08/2014  v0.1.0      Code updated to match V0.1.2 of the PCB design - new FET driver, simpler DIP switches, dropped Dual&raw VBat. 
//                              Upgraded to INA-226, revised Bluetooth module.  Added Load Dump pullback logic.  User input to override autovolt detection,
//                              external charge source Amp offset, config lockdown (Security).  Updated default Charge Profiles.  Enabled Voltage-only
//                              capability (shunt not connected).
//      08/24/2013  v0.0.3      1st usable code release -  core regulation and auto-sizing works at basic level.
//      07/23/2013  v0.0.2      Initial debug with regulator hardware - still some issues, but posting to allow external GUI app development
//      06/11/2013  v0.0.1      1st edition posted, has only been tested in UNO card.
//      05/28/2013  v0.0.0      1st edits, code cut from v0.1.0 of the Smart Engine Control and Alternator Regulator source.
//
//
//
//
//
//------------------------------------------------------------------------------------------------------
//
//

                                               
                                                
#include "Config.h"
#include "Flash.h"
#include "CPE.h"
#include "OSEnergy_Serial.h"
#include "Alternator.h"
#include "System.h"
#include "Sensors.h"
#include "LED.h"
#include "OSEnergy_CAN.h"




                                                                        

 /***************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              STARTUP FUNCTIONS                                       *
 *                                                                                      *
 *                                                                                      *
 *      This is called one time at controller startup.                                  *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/
  
    

//
//------------------------------------------------------------------------------------------------------
// Start-up routine.  
//
//      For now both cold start and Watchdog restart will come through here
//
//
//
//------------------------------------------------------------------------------------------------------

void setup() {


  wdt_enable(WDT_PER);                                                                  // Because we do SOOO much during setup use the Watchdog during startup...

  
  #ifdef DEBUG
    stampFreeStack();                                                                   // Put a stamp into all the free memory under the stack, and use it later to do best guess how low the the stack goes.
    #endif

  #if defined (CPU_AVR) || \
      defined (CPU_AVRCAN)                                                                  // We are doing an Arduino compile, set up the ports.
    
    Serial.begin(SYSTEM_BAUD);                                                              // Start the serial port.

     
            //---  Assign Atmel pins and their functions.  
            //         Am using Arduino calls as opposed to Atmel - to make things easier to read.
            //         Make sure to also see the Initialze_xxx(); functions, as they will also assign port values.
    pinMode(LED_RED_PORT,         OUTPUT);
    pinMode(LED_GREEN_PORT,       OUTPUT);                                                  // Note that on the non-system board (the original one), the GREEN LED is
                                                                                            // the same port as the RED one.  Doing it this way saves some #ifdef's...
    pinMode(FEATURE_OUT_PORT,     OUTPUT);
    pinMode(FEATURE_IN_PORT,      INPUT);                                 

      


            //----  DIP switch ports.
      
    #ifdef DIP_BIT0  
      pinMode(DIP_BIT0,             INPUT_PULLUP);                                          // Some boards do not use DIP-0 as an input.
      #endif                                                            
      pinMode(DIP_BIT1,             INPUT_PULLUP);                                          // We will use the weak internal Atmel Pull-ups for the DIP switch.
      pinMode(DIP_BIT2,             INPUT_PULLUP);
      pinMode(DIP_BIT3,             INPUT_PULLUP);
      pinMode(DIP_BIT4,             INPUT_PULLUP);
      pinMode(DIP_BIT5,             INPUT_PULLUP);                                          // BTW, this kind of shows a key delta between Arduino and Atmel programming.
      pinMode(DIP_BIT6,             INPUT_PULLUP);                                          // We could accomplish all these settings with just a couple of register writes,
      pinMode(DIP_BIT7,             INPUT_PULLUP);                                          // as opposed to the dozen or so 'pinMode' calls used in Arduino.




    #ifdef STUFF_SENSE_PORT                                                                 // If this regulator has a stuffing option sense port, look to see which option is selected
      pinMode(STUFF_SENSE_PORT,INPUT_PULLUP);
      delay(10);
      if (digitalRead(STUFF_SENSE_PORT) == LOW) {
        VAltScaler = VALT_SCALER_24v;                                                       // Option Resistor is installed.                           
        AAltScaler = AALT_SCALER_24v;
        }
      #endif
      
#endif      // AVR / Arduino compile
        
        

  
  wdt_reset();                                                                          // Pat-pat..

  


        //--------      Adjust System Configuration
        //              Now that things are initialized, we need to adjust the system configuration based on the Users settings of the DIP switches.
        //              We also need to fetch the EEPROM to see if there was a saved 'custom' configuration the user had entered via the GIU.
        //              
        //              Note the DIP switches (and hence any selections / adjustments) are sampled ONLY at Power up,
        //
        //




                                        //------  Read the DIP Switches, and make adjustments to system based on them
    uint8_t dipSwitch;                                                                   
    dipSwitch = readDipSwitch();                                                        // Read the Dip Switch value                          
                                        
                                                                                        // For Stand-along (2nd gen) regulator, DIP Switch mapping is:
                                                                                        //   Bit 0 = Bluetooth Power Enable (not readable via the Atmel CPU)
                                                                                        //   Bit 1 = Charge Profile 0 - Used to select which entry in chargeParms[] array we should be using.
                                                                                        //   Bit 2 = Charge Profile 1
                                                                                        //   Bit 3 = Charge Profile 2 
                                                                                        //   Bit 4 = Battery Capacity 0 - Used to indicate 'size' of battery, to adjust exit amp values
                                                                                        //   Bit 5 = Battery Capacity 1
                                                                                        //   Bit 6 = Small Alt Mode     - Is the Alternator a small capability, and hence the amps should be limited to prevent overheating?
                                                                                        //   Bit 7 = Tachometer Mode    - Turn on to make sure some small level of field PWM is always sent, even during Float and Post-Float modes..
   #ifdef STANDALONE
     cpIndex       =  (dipSwitch >> 1) & 0x07;                                          //  Mask off the bits indicating the Battery Type selection
     systemAmpMult = ((dipSwitch >> 4) & 0x03) + 1;                                     //  Mask off the bits indicating the Battery Capacity selection and adjust to 1..4x
     smallAltMode  =  (dipSwitch >> 6) & 0x01;                                          //  Mask off the bit indicating Small Alternator Capability
     tachMode      =  (dipSwitch >> 7) & 0x01;                                          //  Mask off the bit indicating Tachometer mode
     #endif
   

                                                                                       // On CAN enabled (3rd gen) regulator, DIP Switch mapping is:
                                                                                        //   Bit 0..1 = System / Battery Number to associate with - to look for an RBM.
                                                                                        //   Bit 2..4 = Charge Profile  (If not under control of RBM - Remote Battery Master)
                                                                                        //   Bit 5..6 = Battery Capacity 0 - Used to indicate 'size' of battery (If not under control of RBM)
                                                                                        //   Bit 7 = Small Alt Mode     - Is the Alternator a small capability, and hence the amps should be limited to prevent overheating?
   #ifdef SYSTEMCAN                                                                                     
   
     batteryInstance     = ((dipSwitch)      & 0x03) + 1;                               //  Mask off the bits indicating the Battery ID being associated with (1..8)
     cpIndex             =  (dipSwitch >> 2) & 0x07;                                    //  Mask off the bits indicating the Battery Type (CPE) selection
     systemAmpMult       = ((float)(((dipSwitch >> 5) & 0x03) + 1)) / 2.0;              //  Mask off the bits indicating the Battery Capacity selection and adjust in 250A increments (Thank you Ben for this insight)
 
     smallAltMode        =  (dipSwitch >> 7) & 0x01;                                    //  Mask off the bit indicating Small Alternator Capability
     tachMode            =  false;                                                      //  No DIP switch for this, so assume do-not-enable TACH mode (User is able to override via ASCII command, we pick that up later)
     #endif

   
   
 
              
                                        //------  Fetch Configuration files from EEPROM.  The structures already contain their heir 'default' values compiled FLASH.  But we check to see if there are 
                                        //        validated user-saved overrides in EEPROM memory.

                                                                                        
   read_SCS_EEPROM(&systemConfig);                                                      // See if there are valid structures that have been saved in the EEPROM to overwrite the default (as-compiled) values
   read_CAL_EEPROM(&ADCCal);                                                            // See if there is an existing Calibration structure contained in the EEPROM.

   #ifdef SYSTEMCAN                                                                                     
    read_CCS_EEPROM(&canConfig);     
    #endif
                                                                                    

                                                                                        
   thresholdPWMvalue = systemConfig.FIELD_TACH_PWM;                                     // Transfer over the users desire into the working variable.  If -1, we will do Auto determine.  If anything else we will just use that
                                                                                        // value as the MIN PWM drive.  Note if user sets this = 0, they have in effect disabled Tack mode independent DIP switch.
   if (systemConfig.FORCED_TM == true)
      tachMode = true;                                                                  // If user has set a specific value, or asked for auto-size, then we must assume they want Tach-mode enabled, independent of the DIP



                                        //---  Initialize library functions and hardware modules
                                        //
    
   initialize_sensors();
   initialize_alternator();
  
  
  #ifdef STANDALONE                                                                     // Startup the stand-alone regulators Vbat and Amps sensor.
    config_BT(systemConfig.USE_BT);                                                     // Configure the Bluetooth, and if user has disabled it via software - turn it off as well...
    #endif
    
           
  #ifdef  SYSTEMCAN 
    initialize_CAN();
    #endif
         
                                                                                        
                                        //-----  Sample the System Voltage and adjust system to accommodate different VBats
                                        //
                                        
   delay (100);                                                                         // It should have only take 17mS for the INA226 to complete a sample, but let's add a bit of padding..                                   
                                                                                
   read_ALT_VoltAmps();                                                                 // Sample the voltage the alternator is connected to 
     
   if       (measuredAltVolts < 17.0)   systemVoltMult = 1;                             //  Likely 12v 'system'
   else  if (measuredAltVolts >= 40.0)  systemVoltMult = 4;                             //  Must be 48v 'system'
   else                                 systemVoltMult = 2;                             //  Anything in-between we will treat as a 24v 'system' 
                                                                                        //    Note that beginning with v1.0.3, '32v' auto-select has been removed, too risky - user should fix voltMult using $SCO: command




                                        //------ Now that we have done all the above work, let's see if the user has chosen to override any of the features!

   if (systemConfig.CP_INDEX_OVERRIDE != 0)    cpIndex        = systemConfig.CP_INDEX_OVERRIDE - 1;
   if (systemConfig.BC_MULT_OVERRIDE  != 0.0)  systemAmpMult  = systemConfig.BC_MULT_OVERRIDE;
   if (systemConfig.SV_OVERRIDE       != 0.0)  systemVoltMult = systemConfig.SV_OVERRIDE;
   if (systemConfig.ALT_AMPS_LIMIT    !=  -1)  altCapAmps     = systemConfig.ALT_AMPS_LIMIT;            // User has declared alternator size - make sure it is recognized before getting into things.
                                                                                                        // (Needed here in case user has forced regulator to float via feature_in out the door
      
                                                                                                      //  bypassing RAMP: mode where this variable would normally get checked)
#ifdef  SYSTEMCAN 
  if (canConfig.BI_OVERRIDE != 0)            batteryInstance = canConfig.BI_OVERRIDE;
  #endif
  
    

   
   if (read_CPS_EEPROM(cpIndex, &chargingParms) != true)                                 // See if there is a valid user modified CPE in EEPROM we should be using.
         transfer_default_CPS(cpIndex, &chargingParms);                                  // No, so prime the working Charge profile tables with default values from the FLASH (PROGMEM) store.


  
 
        //---- And after all that, is the user requesting a Master Reset?

#ifdef FEATURE_IN_RESTORE
   if ((feature_in(true) == true)               &&                                      // Check feature_in port, and do all the 'debounce' when doing so.
       (dipSwitch == MASTER_RESTORE_DIP_CHECK)  &&                                      //  ONLY do Master Restore if CPE#5  is selected, and all the rest are in defined state.
       (systemConfig.CONFIG_LOCKOUT != 2)) {                                            //       And the regualtor has not been really locked out!
                                                                                        // feature_in is active, so do a restore all now. 
        systemConfig.CONFIG_LOCKOUT = 0;                                                // As this is a HARDWARE restore-all, override any lockout.
        restore_all();                                                                  // We will not come back from this, as it will reboot after restoring all.
        }
   #endif





 #ifdef DEBUG
    sendDebugString = true;                                                             // Send out debug string if in debug mode :-)
    #ifdef  SMALL_FLASH 
        ASCII_write("Size of EEPROM being used (out of 1024 bytes) = ");
    #else
        ASCII_write("Size of EEPROM being used (out of 2048 bytes) = ");
    #endif    
    
   //!! HEY!!    Not calculating correctly....    --> Serial.println(FLASH_MEM_USED);

    ASCII_write("Free memory after setup() =");
    ASCII_write(freeMemory());     
    #endif
 




 #ifdef SIMULATION
    ASCII_write("***** SIMULATION MODE ******/r/n");                                // Note `The APP' looks for "SIMUL" to recognize we are in Simulation mode.
   
    //----  One-off selections to help vary different nodes during Simulation.
    //systemConfig.ENGINE_WARMUP_DURATION = 10;
      canConfig.SHUNT_AT_BAT              = true;
    //canConfig.DEVICE_PRIORITY           = 71;
    #endif
    


}   // End of the Setup() function. 



















/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              SUPPORTIVE FUNCTIONS                                    *
 *                                                                                      *
 *                                                                                      *
 *      This routines are called from within the Mainloop functions or Startup.         *
 *      These are mostly low-level functions to read sensors, keyboards, refresh        *
 *         displays, etc...                                                             *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/


//------------------------------------------------------------------------------------------------------
// Stack / Heap stamping
//
//      These functions will place a know pattern (stamp) in un-used memeory between the bottom of the stack
//      and the top of the heap.   Other functions can call later and check to see how much of this
//      'stamping' is retained in order to get an estimate of how much stack space is used during runtime.
//
//      Not quite as good as hardware enforced fences, but better then nothing.
//
//------------------------------------------------------------------------------------------------------
#ifdef DEBUG
 
void stampFreeStack(void) {
    uint8_t *stmpPtr;
    uint8_t *stkPtr;    
    extern unsigned int __heap_start;
    extern void *__brkval;

    if ((int)__brkval == 0) 
        stmpPtr = (uint8_t*)&__heap_start;
    else 
        stmpPtr = (uint8_t*)__brkval;

    stmpPtr += 8;                                       // Just move a little above the top of the the heap.
    stkPtr  = (unsigned char*) &stkPtr - 8;             // And a little below the current stack pointer.

    //Serial.print("Stamping: "); 
    //Serial.print((unsigned int)stmpPtr,HEX);
    //Serial.print(", to: ");
    //Serial.println((unsigned int)stkPtr,HEX);
    
    while (stmpPtr < stkPtr)
       *stmpPtr++ = 0xA5;
             
}

int checkStampStack(void) {
    unsigned int unusedCnt;
    uint8_t *stmpPtr;
    uint8_t *stkPtr;    
    extern unsigned int __heap_start;
    extern void *__brkval;

    if ((int)__brkval == 0) 
        stmpPtr = (uint8_t*)&__heap_start;
    else 
        stmpPtr = (uint8_t*)__brkval;

    stmpPtr += 8;                      // Just move a little above the top of the the heap.
    stkPtr  = (unsigned char*) &stkPtr - 8;             // And a little below the current stack pointer.

    //Serial.print("Free memory=");
    //Serial.println(freeMemory());  
        
    //Serial.print("Looking from: 0x"); 
    //Serial.print((unsigned int)stmpPtr,HEX);
    //Serial.print(", to: 0x");
    //Serial.print((unsigned int)stkPtr,HEX);


    unusedCnt = 0;
    
    while ((stmpPtr < stkPtr) && (*stmpPtr++ == 0xA5))
       unusedCnt++;

    //Serial.print("      Untouched is now: 0x");
    //Serial.println(unusedCnt,HEX);
             
    return (unusedCnt);
}
#else
  int checkStampStack(void) { return(-1); }                                             // Need dummy function when not compiling debug..
#endif

  



#ifdef STANDALONE                                                                         // All of this  Bluetooth stuff is ONLY on the stand-alone version..

//------------------------------------------------------------------------------------------------------
// Configure Bluetooth
//
//      This function is called from Startup.  It will look for the BT module and verify its name, password, mode of operation, etc.
//      and make configuration changes if needed.
//
//      If FALSE is passed in, after making configuration changes, this function will place the Bluetooth module into a powered-down state.
//
//      Note that this function CAN take a int32_t time to complete (Several seconds), depending on the condition of the module...
//      Note also that we ASSUME to BT module is fixed at 9600 baud via hardware selection (or pre-configuration)
//
//------------------------------------------------------------------------------------------------------

void config_BT(bool  enableBT) {

                                                            
   char    buffer[15 + MAX_NAME_LEN + MAX_PIN_LEN];                                     // Used to assemble commands to the Bluetooth
   bool    BTreboot_needed;


        BTreboot_needed = false;                                                        // Assume we do NOT need to reconfigure the Bluetooth module.



        //---   IS there a BT module out there?
        //

                                                                                        // Place RN-41 module into Command mode. 
        send_command("$$$");                                                            // (send_command() makes sure the Serial Buffers are flushed before starting a new sequence.)
        if (!wait_string ("CMD"))  return;                                              //  If no Bluetooth is found, just skip the rest!


        wdt_reset();                                                                    // Some of these BT ops can take a while, make sure the Dog is kept happy :-)



        //----  Seems so, Is it configured as we want it?
        //

        send_command("GN\r");                                                           // Ask for the current configured Name.
        if (!wait_string(systemConfig.DEVICE_NAME)) {                                      // Name returned is something other then what we want.

           strcpy(buffer,"SN,");                                                        // Set Device NAME to what we want.
           strcat(buffer,systemConfig.DEVICE_NAME);
           strcat(buffer,"\r");

           send_command(buffer);
           wait_string ("AOK");                                                         // Let it take.

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }

        wdt_reset();


        
        send_command("GP\r");                                                           // Ask for the current configured Password
        if (!wait_string(systemConfig.DEVICE_PSWD)) {                                   // Password returned is something other then what we want.

           strcpy(buffer,"SP,");                                                        // Set Device PASSWORD.
           strcat(buffer,systemConfig.DEVICE_PSWD);
           strcat(buffer,"\r");

           send_command(buffer);
           wait_string ("AOK");

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }


        wdt_reset();



        //----  Does the user want to turned off (via Software)?
        //

        if (enableBT == false) {                                                        // BT has been disabled by a user in software config.
                
           send_command("Q,1\r");                                                       // Place module into powered down mode
           wait_string ("AOK");                                                         // Let it take.
           send_command("Z\r");                                                         // and low power mode
           wait_string ("AOK");

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }


        wdt_reset();



        //----  Finally, let's look at some other config details, make sure they are how we want them.
        //

        send_command("GA\r");                                                           // Current Mode = Legacy Password?
        if (!wait_string("4")) {                                                        // 4 = legacy password.  (Note dbl quote " vs. single ' is used.
                                                                                        //   We need to mass a NULL terminated string, not a single character)
           send_command("SA,4\r");                                                      
           wait_string("AOK");  

           BTreboot_needed = true;                                                      // And set flag indicated we need to 'reboot' the BT when finished.
           }

        wdt_reset();




        #ifdef DEBUG                                                                    // Set ability to place RN41 into command mode.
        send_command("GT\r");                                                           // in DEBUG, allow both local as well as remote to send 'CMD' and config BT device.
        if (!wait_string("255")) {

           send_command("ST,255\r");                                                    
           wait_string("AOK");  

           BTreboot_needed = true;                                                      
           }
        #else
        send_command("GT\r");                                                           // However in Normal mode, only allow the local Atmel CPU to enter command mode.
        if (!wait_string("253")) {                                                      // Blocking any outside attempt to hi-jack the RN41 Bluetooth!

           send_command("ST,253\r");                                                    
           wait_string("AOK");  

           BTreboot_needed = true;
           }
        #endif

        wdt_reset();






        //----  OK, all done.  Do we need to 'reboot' the module?
        //

        if (BTreboot_needed == true) {                                                  // We made some changes, and need to ask the RN-41 to reboot.
            send_command("R,1\r");                                                      // All done, reset RN41 module with new configuration.
            wait_string ("Reboot");
            delay(500);                                                                 // Specs says it takes 500mS after reboot to respond.
            }
        else
           send_command("---\r");                                                       // Else just drop out of Command Mode.


        wdt_reset();                                                                    // One last time, before we say good-by to config_BT()
        
   
}










//----  Helper function for Bluetooth.
//

void send_command(const char *buffer) {                         // Transmits out the passed string to the serial buffer, after making sure the buffers are flushed.
                                                                // This helps avoid out-of-step issues with the RN-41 device...

   Serial.flush();                                              // Make sure the output and input buffers are clear before sending out a new command.
   while (Serial.available()>0)
          Serial.read();                                        // Clear the input buffer, just dump anything that comes in..

   ASCII_write(buffer);                                         // NOW we can send the actual command.
   Serial.flush();                                              // And wait for the output buffer to clear
   delay(10);                                                   // This, for some reason, helped a lot.  let the RN-41 get things in order before moving on I guess, or something 
                                                                // was changed in the Arduino Flush lib function. . . .
   }



bool wait_string(const char match[]) {                          // Read the serial port until it finds a match for the passed string, or BT timeout has been exceeded.
   uint32_t  startedLooking;                               // At what time did we start looking to for this string?  Used for time-out
   int            index = 0;
 
   startedLooking = millis();

   do {                                                         // Timeout loop . . .
     while (Serial.available()>0){                              //  . . . wrapping around character available / match loop.
        if (Serial.read() == match[index])
                index++;                                        // Looking promising for a match!
            else
                return(false);                                  // No Go.  (Am rather harsh here, response must match EXACTLY as expected to return True.)

        if (match[index] == '\0')                               // Did we make it to the NULL terminator in the passed character string array?
            return(true);                                       // Yes, return success!
        }
    } while ((millis() - startedLooking) < BT_TIMEOUT);

   return(false);                                               // Looks like we timed out...
   }

#endif    // STANDALONE




//------------------------------------------------------------------------------------------------------
// Read DIP Switch
//      This function is called from Startup and will read the DIP switch.  It will return all 7 usable bits 
//      assembled into one returned value.  It is also called during the $SCN: command to make sure the DIP switch 
//      is set to all-on before allowing initial updating of the Name and Password.
//
//
//       For stand-alone regulator, DIP Switch mapping is:
//              Bit 0 = Bluetooth Power Enable (not readable via the Atmel CPU --  0 is always returned.)
//              Bit 1 = Charge Profile bit-0    - Used to select which entry in chargeParms[] array we should be using.
//              Bit 2 = Charge Profile bit-1
//              Bit 3 = Charge Profile bit-2 
//              Bit 4 = Battery Capacity bit-0  - Used to indicate 'size' of battery, to adjust exit amp values
//              Bit 5 = Battery Capacity bit-1
//              Bit 6 = Small Alt Mode           = The Alternator has a small capability, and hence the amps should be limited to prevent overheating
//              Bit 7 = Alt Tach Mode            = Will cause the Regulator to always have some small value of PWM present to allow for external Alt drive Tachs to work
//                                                 Caution with this mode, as it MIGHT cause some overcharging to the battery. . . .
// 
//
//      CAN enabled (systems) regulator, DIP switch mapping is:
//              Bit 0..2 = Which 'battery' / CAN systemID should we look to be connected to? 
//              Bit 3..4 = CPE entery
//              Bit 5..6 = Battery Capacity
//              Bit 7    = Small Alt Mode
//
//------------------------------------------------------------------------------------------------------


uint8_t readDipSwitch() {

uint8_t dipSwitch;


#ifdef SIMULATION
  #ifdef STANALONE
//   return(0b00111100);                                                                         // Testing, CPE=7th, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff, BIG HD FLA battery (Industrial, etc) w/OC mode enabled
     return(0b00110100);                                                                         // Testing, CPE=3rd, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff, BIG FLA battery (T-105, etc)
//   return(0b00000010);                                                                         // Testing, CPE=2nd, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff
//   return(0b00001110);                                                                         // Testing, CPE=8th, Bat Size = 10, Small Alt Mode = off, Tach mode = 0ff  LiFePO4 testing
//   return(0b01000010);                                                                         // Testing, CPE=2nd, Bat Size = 10, Small Alt Mode = ON,  Tach mode = 0ff
     #endif

  #ifdef  SYSTEMCAN
     return(0b11101000);                                                                        // Testing, CPE=3rd - BIG FLA battery (T-105, etc), Bat Size = 11, Small Alt Mode = ON, BatID = 1 
     #endif
     
#endif
     
   
#if defined (CPU_AVR) || \
    defined (CPU_AVRCAN) 

    #ifdef DIP_BIT0   
    dipSwitch  =  digitalRead(DIP_BIT0);                                                        // Start with the LSB switch,
    #else
    dipSwitch  = 0;                                                                             // Stand-alone regulator uses the LSB for Bluetooth Power
    #endif

    dipSwitch |= (digitalRead(DIP_BIT1) << 1);                                                  // Then read the DIP switches one switch at a time, ORing them at their appropriate bit location,
    dipSwitch |= (digitalRead(DIP_BIT2) << 2); 
    dipSwitch |= (digitalRead(DIP_BIT3) << 3);
    dipSwitch |= (digitalRead(DIP_BIT4) << 4);
    dipSwitch |= (digitalRead(DIP_BIT5) << 5);
    dipSwitch |= (digitalRead(DIP_BIT6) << 6);
    dipSwitch |= (digitalRead(DIP_BIT7) << 7);

#elif defined (CPU_STM32)
    
    dipSwitch  = HAL_GPIO_ReadPin(DIP_1_GPIO_Port, DIP_1_Pin);                                      // Start with the LSB switch,
    dipSwitch |= HAL_GPIO_ReadPin(DIP_2_GPIO_Port, DIP_2_Pin) << 1;  
    dipSwitch |= HAL_GPIO_ReadPin(DIP_3_GPIO_Port, DIP_3_Pin) << 2;  
    dipSwitch |= HAL_GPIO_ReadPin(DIP_4_GPIO_Port, DIP_4_Pin) << 3;  
    dipSwitch |= HAL_GPIO_ReadPin(DIP_5_GPIO_Port, DIP_5_Pin) << 4;  
    dipSwitch |= HAL_GPIO_ReadPin(DIP_6_GPIO_Port, DIP_6_Pin) << 5;  
    dipSwitch |= HAL_GPIO_ReadPin(DIP_7_GPIO_Port, DIP_7_Pin) << 6;  
    dipSwitch |= HAL_GPIO_ReadPin(DIP_8_GPIO_Port, DIP_8_Pin) << 7;  

#endif

  return(~dipSwitch);                                                                           // Invert the returned value, as the DIP switches are really pull-downs.  So, a HIGH means switch is NOT active.

}










//------------------------------------------------------------------------------------------------------
// Manage System State 
//      This function look at the system and decide what changes should occur.
//
//      For the stand-alone alternator, it is much simpler - if the Alternator is not running, and not Faulted, start it up!
//
// 
//------------------------------------------------------------------------------------------------------


void manage_system_state() {



  

    //---- Unique application, if this compile option is enabled the regulator will monitor the CAN bus for J1939 engine RPM messages.
    //     If it received some, we can assume the engine is now running and should start a charging cycle.  However, if we do NOT see
    //     these messages in a timely fashion, we will assume the engine has stopped and we should also stop the regulator.
    //     In this usage, the alternator Regulator’s ENABLE pin is often always powered, so the regulator can go into kind of asleep mode 
    //     while waiting to save power.

#ifdef DISABLE_ON_J1939_RPMs
         if (measuredRPMs < J1939_RPM_THRESHOLD) {                                            // Has the engine stopped?
                                                                                              // (Note:  Using Measured_RPMs allow for positive engine stopped signaling via J1939 RPMs,
                                                                                              //   as well as allow for failure-mode fall-back to keep the alternator running if the CAN
                                                                                              //    bus fails, but we are still able to see the engine is still running via Stator attached RPMs)
            set_ALT_PWM(0);                                                                   // Looks like it, as far as we can tell.  Turn off the regulator
            set_charging_mode(disabled);
            return;                                                                           // Do not pass GO until we are able to see engine RPMs.
            }

#endif




   
                        // --------  Do we need to start the Alternator up?

 if ((chargingState == unknown)  || (chargingState == disabled)) {                                                                                         

        set_ALT_PWM(FIELD_PWM_MIN);                                                             // Make sure Field is at starting point. 
        set_charging_mode(pending_R);                                                           // Prepare for Alternator, let manage_ALT() change state into Ramping
        reset_run_summary();                                                                    // Zero out the accumulated AHs counters as we start this new 'charge cycle'
        }


 }














//------------------------------------------------------------------------------------------------------
// Reboot
//
//      This function will disable the Alternator and then forces the Atmel CPU to reboot via a watchdog timeout.
//      It is typically called making a change to the EEPROM information to allow the controller to re-initialize
//      
// 
//
//------------------------------------------------------------------------------------------------------


        
void reboot() {

     set_ALT_PWM(0);                                                            // 1st, turn OFF the alternator, All the way OFF as we will no longer be managing it...
     #ifdef CHARGE_PUMP_PORT
        analogWrite(CHARGE_PUMP_PORT,0);                                        // and the Charge Pump as well.
        #endif

     wdt_enable(WDT_PER);                                                       // JUST IN CASE:  Make sure the Watchdog is enabled! 
                                                                                //  (And in any case, let it be the shorter reboot watchdog timeout.

     blink_LED (LED_RESETTING,LED_RATE_FAST, -1, true);                         // Show that something different is going on...

     ASCII_write("RST;\r\n");
     Serial_flush();                                                            // And then make sure the output buffer clears before proceeding with the actual reset.

   #ifdef OPTIBOOT
     while (true)                                                               // Optiboot!
        refresh_LED();                                                          // Sit around, blinking the LED, until the Watchdog barks.
        

     #else                                                                      // The standard Arduino 3.3v / atMega328 has a bug in it that will not allow the watchdog to 
                                                                                // work - instead the system hangs.  Optiboot fixes this.  (Note the standard bootload for the 
                                                                                // Arduino UNO card is actually an Optiboot bootloader..)
                                                                                // If we are not being placed into an Optiboot target, we need to do something kludge to cause the
                                                                                // system to reboot, as opposed to leverage the watchdog timer.  So:

        void(* resetFunc) (void) = 0;                                           // Kludge - just jump the CPU to address 0000 in the EEPROM.
                                                                                // declare reset function @ address 0.  Work around for non-support in watchdog on Arduino 3.3v
        resetFunc();                                                            // Then jump to it to 'restart' things.
        #endif




}











/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              MAIN LOOP                                               *
 *                                                                                      *
 *                                                                                      *
 *      This gets called repeatedly by the Arduino environment                          *
 *                                                                                      *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/


void loop()  {

   if (chargingState == FAULTED)  {
                wdt_disable();                                                                          // Turn off the Watch Dog so we do not 'restart' things.
                handle_fault_condition();                                                               // Take steps to protect system

                if (faultCode & 0x8000U)                                                                // If the Restart flag is set in the fault code,
                   reboot();                                                                            //    then do a forced software restart of the regulator

                return;                                                                                 // Else bail out, we will need a full hardware reset from the user.
                }







  //
  //
  //-------   OK, we are NOT in a fault condition.  Let's get to business, read Sensors and calculate how the machine should be behaving
  //
  //
#ifndef CPU_STM32 
        if (read_sensors()== false) return;                                                             // If there was an error in reading a critical sensor we have FAULTED, restart the loop.
                                                                                                        // Treat volts and amps sensed directly by the regulator as the ALTERNATOR values. .
        resolve_BAT_VoltAmpTemp();                                                                      // . .   but then look to see if they should also be considered the BATTERY values.
                                                                                                        //       (Will be yes, unless we receive battery volts / amps externally via an ASCII command or the CAN)
#endif                                                                                                  // Reading of Sensors is handled in STM32 by RTOS sensor task.          
        calculate_RPMs();                                                                               // What speed is the engine spinning?
        calculate_ALT_targets();                                                                        // With all that known, update the target charging Volts, Amps, Watts, RPMs...  global variables.
 





  //
  //
  //-------     Now lets adjust the system.  But first, lets make sure we have not exceeded some threshold and hence faulted.
  //
  //

        if (check_for_faults() == true)  return;                                                        // Check for FAULT conditions 
                                                                                                        // If we found one, bail out now and enter holding pattern when we re-enter main loop.


        manage_ALT();                                                                                   // OK we are not faulted, we have made all our calculations. . . let's set the Alternator Field.
        manage_system_state();                                                                          // See if the overall System State needs changing.



  //
  //
  //-------     Tell the world what we are doing, and see if they want us to do something else!
  //
  //
        handle_feature_in();
        check_inbound();                                                                                // See if any communication is coming in via the Bluetooth (or DEBUG terminal), or Feature-in port.

        update_run_summary();                                                                           // Update the Run Summary variables
        send_outbound(false);                                                                           // And send the status via serial port - pacing the strings out.
        update_LED();                                                                                   // Set the blinking pattern and refresh it. (Will also blink the FEATURE_OUT if so configured via #defines
        update_feature_out();                                                                           // Handle any other FEATURE_OUT mode (as defined by #defines) other then Blinking.
 
     
        #ifdef SYSTEMCAN
            manage_CAN();                                                                               // Do any Rx and Tx messages, see if CAN subsystem needs to go to sleep.
            #endif

        
        wdt_reset();                                                                                    // Pet the Dog so he does not bit us!

}                                                                                                       // All done this time around.  Back to Adriano and it will call this 
 
 
 
 
 
 
 


 

