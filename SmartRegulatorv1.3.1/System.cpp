//      System.cpp
//
//      Copyright (c) 2018 by William A. Thomason.                  http://arduinoalternatorregulator.blogspot.com/
//                                                                  http://smartdcgenerator.blogspot.com/
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
//              Note:  in an effort to have as many common files as possible between the different projects,
//              system.cpp will contain kind of a catch-all for loosely defined global functions and variables.
//              Many of these came from 'SmartRegulator.ino' and 'Alternator.cpp'
//



#include "Config.h"
#include "Sensors.h"
#include "Flash.h"
#include "CPE.h"
#include "OSEnergy_Serial.h"
#include "System.h"
#include "LED.h"
#include "OSEnergy_CAN.h"

#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR) 
    #include "Alternator.h"
#elif defined (OSE_BATMON)
    #include "Battery.h"
#endif





//
//------------------------------------------------------------------------------------------------------
//
//      Global Variables
//
//              These control major functions within the program
//
//
//

                                                                     
unsigned        faultCode;                                              // If chargingState is set to FAULTED, this holds a # that indicates what faulted.
uint8_t         requiredSensorsFlag     = 0;                            // Are there any required sensors missing?

char const firmwareVersion[] = FIRMWARE_VERSION;                        // Sent out with SST; status string and CAN initialization for product ID.









                                //---   Default System Config variable is defined here.
#ifdef OSE_ALTERNATOR    
  tSCS systemConfig = {
        false,                          // .REVERSED_SHUNT              --> Assume shunt is not reversed.
        90,                             // .ALT_TEMP_SETPOINT           --> Default Alternator temp - 90c  (Approx 195f)
        1.00,                           // .ALT_AMP_DERATE_NORMAL       --> Normal cap Alternator at 100% of demonstrated max Amp capability, 
        0.75,                           // .ALT_AMP_DERATE_SMALL_MODE   --> Unless user has selected Small Alt Mode via DIP switch, then do 75% of its capability
        0.50,                           // .ALT_AMP_DERATE_HALF_POWER   --> User has shorted out the Alternator Temp NTC probe, indicating they want to do 1/2 power mode.
        -1,                             // .ALT_PULLBACK_FACTOR         --> Used to pull-back Field Drive as we move towards Idle.  
        0,                              // .ALT_IDLE_RPM                --> Used to pull-back Field Drive as we move towards idle.  Set = 0 causes RPMs to be determined automatically during operation.
        0,                              // .ALT_AMPS_LIMIT              --> The regulator may OPTIONALLY be configured to limit the size of the alternator output 
                                        //                                  Set = 0 to disable Amps capping.  Set = -1 to auto-size Alternator during Ramp. (required Shunt on Alt, not Bat)
        0,                              // .ALT_WATTS_LIMIT             --> The regulator may OPTIONALLY be configured to limit the load placed on the engine via the Alternator.
                                        //                                  Set = 0 to disable, -1 to use auto-calc based on Alternator size. (Required Shunt on Alt, not Bat)
        12,                             // .ALTERNATOR_POLES            --> # of poles on alternator (Leece Neville 4800/4900 series are 12 pole alts)
       ((6.7 / 2.8) * 1.00),            // .ENGINE_ALT_DRIVE_RATIO      --> Engine pulley diameter / alternator diameter &  fine tuning calibration ratio       
 (int) ((500/0.050)  * 1.00),           // .AMP_SHUNT_RATIO             --> Spec of amp shunt,  500A / 50mV shunt (Link10 default) and % calibrating error
                                        //                                   CAUTION:  Do NOT exceed 80mV on the AMP Shunt input
        -1,                             // .FIELD_TACH_PWM              --> If user has selected Tach Mode, use this for MIN Field PWM.  
                                        //                                    Set = -1 to 'auto determine' the this value during RAMP phase
                                        //                                    Set =  0 to in effect 'disable' tach mode, independent of the DIP switch.
        0,                              // .FORCED_TM                   --> User can FORCE tach mode independent of DIP switch using $SCT command.  0=DIP/off, 1=Force-on

        true,                           // .USE_BT                      --> Should we try to use the Bluetooth?
        "ALTREG",                       // .DEVICE_NAME                 --> Name of Regulator (Used for NMEA2000 and Bluetooth) module.  MAX 18 CHARS LONG!  (see MAX_NAME_LEN)
        "1234",                         // .DEVICE_PSWD                 --> Password to use for Bluetooth module.  MAX 18 CHARS LONG!  (see MAX_PIN_LEN)         
        DEFAULT_BT_CONFIG_CHANGED,      // .BT_CONFIG_CHANGED           --> BT name and password are still the default.  Updates to configuration data is prevented until the name & password is changed.

         0,                             // .CP_INDEX_OVERRIDE           --> Use the DIP switch selected indexes
         0.0,                           // .BC_MULT_OVERRIDE            --> Use the DIP switch selected multiplier
         0.0,                           // .SV_OVERRIDE                 --> Enable Auto System voltage detection
         0,                             // .CONFIG_LOCKOUT              --> No lockouts at this time.
         60,                            // .ENGINE_WARMUP_DURATION      --> Allow engine 60 seconds to start and 'warm up' before placing a load on it.  
         0};                            // .REQURED_SENSORS             --> Force check and fault if some sensors are not present (ala, alt temp sensor)


tModes   chargingState     = unknown;                                   // What is the current state of the alternator regulators?  (Ramping, bulk, float, faulted, etc...)
tModes   systemState       = unknown;                                   // For Alternator, this is just a dummy placeholder to allow for common code.  

#elif defined OSE_GENERATOR  
  tSCS systemConfig = {
        false,                          // .REVERSED_SHUNT              --> Assume shunt is not reversed.
        90,                             // .ALT_TEMP_SETPOINT           --> Default Alternator temp - 90c  (Approx 195f)
        1.00,                           // .ALT_AMP_DERATE_NORMAL       --> Normal cap Alternator at 100% of demonstrated max Amp capability, 
        0.75,                           // .ALT_AMP_DERATE_SMALL_MODE   --> Unless user has selected Small Alt Mode via DIP switch, then do 75% of its capability
        0.50,                           // .ALT_AMP_DERATE_HALF_POWER   --> User has shorted out the Alternator Temp NTC probe, indicating they want to do 1/2 power mode.
        -1,                             // .ALT_PULLBACK_FACTOR         --> Used to pull-back Field Drive as we move towards Idle.  
        0,                              // .ALT_IDLE_RPM                --> Used to pull-back Field Drive as we move towards idle.  Set = 0 causes RPMs to be determined automatically during operation.
        0,                              // .ALT_AMPS_LIMIT              --> The regulator may OPTIONALLY be configured to limit the size of the alternator output 
                                        //                                  Set = 0 to disable Amps capping.  Set = -1 to auto-size Alternator during Ramp. (required Shunt on Alt, not Bat)
        0,                              // .ALT_WATTS_LIMIT             --> The regulator may OPTIONALLY be configured to limit the load placed on the engine via the Alternator.
                                        //                                  Set = 0 to disable, -1 to use auto-calc based on Alternator size. (Required Shunt on Alt, not Bat)
        12,                             // .ALTERNATOR_POLES            --> # of poles on alternator (Leece Neville 4800/4900 series are 12 pole alts)
       ((6.7 / 2.8) * 1.00),            // .ENGINE_ALT_DRIVE_RATIO      --> Engine pulley diameter / alternator diameter &  fine tuning calibration ratio       
 (int) ((500/0.050)  * 1.00),           // .AMP_SHUNT_RATIO             --> Spec of amp shunt,  500A / 50mV shunt (Link10 default) and % calibrating error
                                        //                                   CAUTION:  Do NOT exceed 80mV on the AMP Shunt input
        -1,                             // .FIELD_TACH_PWM              --> If user has selected Tach Mode, use this for MIN Field PWM.  
                                        //                                    Set = -1 to 'auto determine' the this value during RAMP phase
                                        //                                    Set =  0 to in effect 'disable' tach mode, independent of the DIP switch.
        0,                              // .FORCED_TM                   --> User can FORCE tach mode independent of DIP switch using $SCT command.  0=DIP/off, 1=Force-on

        true,                           // .USE_BT                      --> Should we try to use the Bluetooth?
        "DC_GEN",                       // .DEVICE_NAME                 --> Name of Regulator (Used for NMEA2000 and Bluetooth) module.  MAX 18 CHARS LONG!  (see MAX_NAME_LEN)
        "1234",                         // .DEVICE_PSWD                 --> Password to use for Bluetooth module.  MAX 18 CHARS LONG!  (see MAX_PIN_LEN)         
        true,                           // .BT_CONFIG_CHANGED           --> BT name and password are still the default.  Updates to configuration data is prevented until the name & password is changed.
                                                                            // DC Gen, like gen3 of the alt reg, does not have Bluetooth - so we will simply let things pass for now.
         0,                             // .CP_INDEX_OVERRIDE           --> Use the DIP switch selected indexes
         0.0,                           // .BC_MULT_OVERRIDE            --> Use the DIP switch selected multiplier
         0.0,                           // .SV_OVERRIDE                 --> Enable Auto System voltage detection
         0,                             // .CONFIG_LOCKOUT;             --> No lockouts at this time.
         60,                            // .ENGINE_WARMUP_DURATION      --> Allow engine 60 seconds to start and 'warm up' before placing a load on it.  
         0};                            // .REQURED_SENSORS             --> Force check and fault if some sensors are not present (ala, alt temp sensor)
     

         
tGCS   genConfig = {
    
    
        };



            
            
            
tModes   chargingState  = unknown;                                       // What is the current state of the alternator regulators?  (Ramping, bulk, float, faulted, etc...)
tModes   systemState    = unknown;                                       // What is the current state of the Generator (Starting, Stopped, Warming, etc)
tModes   waterMakerState = unknown;                                      // What is the current state of the Watermaker subsystem 

     
     
#elif defined OSE_SOLAR





#elif defined OSE_BMS

        
tModes   batteryState   = unknown;                                       // What is the current state of the Battery  (Long term storage, fast-charge mode, etc.)
tModes   chargingState  = unknown;                                       // What is the current state of the Charging associated with this battery
tModes   systemState    = unknown;                                       // What is the current state of the Battery Monitor?  (Sleeping, running, faulted..)





#elif defined OSE_BATMON

tModes   batteryState   = unknown;                                       // What is the current state of the Battery  (Long term storage, fast-charge mode, etc.)
tModes   chargingState  = unknown;                                       // What is the current state of the Charging associated with this battery
tModes   systemState    = unknown;                                       // What is the current state of the Battery Monitor?  (Sleeping, running, faulted..)




#elif defined OSE_DISPLAY1




#else
    #error   Undefined systemtype in system.cpp
#endif

    








//----  Some UGLY macros to allow for common code between Arduino and cubeMX  (AVR & STM32 CPUs)
#ifdef  CPU_STM32
    #define  _SET_FO_PORT(x)         HAL_GPIO_WritePin(Feature_OUT_GPIO_Port, Feature_OUT_Pin, x) 
    #define  _GET_FO_PORT()          HAL_GPIO_ReadPin(Feature_IN_GPIO_Port, Feature_IN_Pin) 
    
    #define FOP_ON      GPIO_PIN_SET
    #define FOP_OFF     GPIO_PIN_RESET
    
    
#elif defined(CPU_AVR) || defined(CPU_AVRCAN)
    #define  _SET_FO_PORT(x)         digitalWrite(FEATURE_OUT_PORT, x)
    #define  _GET_FO_PORT()          digitalRead(FEATURE_IN_PORT)


    #define FOP_ON      HIGH
    #define FOP_OFF     LOW

#else
    #error Unsupported CPU in System.CPP 
#endif




void blink_out(int digd);



/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              MAINLOOP SUPPORT FUNCTIONS                              *
 *                                                                                      *
 *                                                                                      *
 *      These routines are called from the Mainloop (as opposed to being inline)        *
 *      They are placed here to help readability of the mainloop.                       *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/







//------------------------------------------------------------------------------------------------------
// Feature in
//      This function will sample the Feature In port and return TRUE if it is currently being held High.
//      Passed flag will tell us if we should just check the Feature_in port, doing all the de-bounce 
//      checking in this one call.  Or if we should just check the debounce and return the last known state
//      if we have not completed the required number of samples (to be checked again at a later time).
//      (e.g:  Should this function behave as a Blocking or non-blocking function?)
//
//      Returns True only if the feature_in pin has been held active during the entire debouncing time.
//
//
//------------------------------------------------------------------------------------------------------

bool feature_in(bool waitForDebounce) {
  bool static    lastKnownState  = false;                                               // Used to retain the featureIn statue during 'next' debounce cycle.
  bool static    proposedState   = false;                                               // Used to see if we have a constant state during entire debounce time period.
  bool           readState;
  int8_t  static debounceCounter = DEBOUNCE_COUNT;                                      // Used to count-down sampling for de-bouncing. 
  int8_t         stuckCounter    = 2*DEBOUNCE_COUNT;                                    // OK, if there is a LOT of noise on the feature-in pin, only stay in this routing for so int32_t.
                                                                                        //  (

  while(--stuckCounter > 0) {                                                           // A bit of safety, this will prevent is from being stuck here forever if there is a noisy feature_in...


     readState = (_GET_FO_PORT() == FOP_ON);                                            // So this 'extra' step to make sure there is no issues between typing of TRUE and HIGH.


     if (readState == proposedState) {                                                  // feature-in looks to be stable.
        if (--debounceCounter <= 0) {                                                   // And it appears it has been for some time as well!
           lastKnownState  = proposedState;                                             //   Recognize the new 'state'
           debounceCounter = DEBOUNCE_COUNT;                                            //   Reset the debounce counter to take a new go at the feature_in port
           break;                                                                       //   And return the new state!
           }
        }
     else {
        proposedState   =  readState;                                                   // Nope, not stable.  Looks like we have a new candidate for the feature_in() port.
        debounceCounter =  DEBOUNCE_COUNT;                                              // Reset the counter and let’s keep looking to see if it remains this way for a while.
        }
     

    if (waitForDebounce == true)
         delay(DEBOUNCE_TIME);                                                          // They want us to do all the debouncing, etc..
    else
        break;                                                                          // Don't want to stick around, so break out of this While() loop.

  }



 return(lastKnownState); 

}










//------------------------------------------------------------------------------------------------------
// Handle FEATURE_IN port
//
//      This function will check the Feature-In port is also checked in this function, 
//      to enable whatever it is configured for by the #defines
// 
//------------------------------------------------------------------------------------------------------

void handle_feature_in(void) { 
                //----- Check the Feature-in port
                //      Doing this 1st as the code below uses a lot of 'returns' to exit this function when it has finished processing input command strings...
 static bool have_seen_low_FI = false;                                  // Have we seen a low feature-in yet?


  #ifdef FEATURE_IN_EQUALIZE
    #ifdef FEATURE_IN_DISABLE_CHARGE                                    // If this feature is enabled, skip equalize with CPE #8
      if (cpIndex != 7)                                                 // So in a very sneaky way, we add on an additional 'if' before continuing on..
    #endif
      if (feature_in(false) == true) {                                  // Check the feature-in port, but do NOT wait around for debouncing.. (We will catch it next time)
         if (((chargingState  == acceptance_charge) || (chargingState   == float_charge))   &&      // True, user is asking for Equalize.  Can they have it?
             (have_seen_low_FI == true))                                                            // Only if the port has toggled from low to high - safety to prevent stuck (or left on) featurein switch.
                set_charging_mode(equalize);                                                        // OK, they want it - it seems like the battery is ready for it - so:  let them have it. 
         } else {
            have_seen_low_FI  = true;
            if (chargingState == equalize)                                                          // We are equalizing, and user dropped Feature-in line.
                set_charging_mode(float_charge);                                                    // So stop equalizing, and go into Float...
         }  

  #endif



   #ifdef FEATURE_IN_DISABLE_CHARGE                                     
      if ((cpIndex == 7) && (chargingState > pending_R)) {                                          // Check to see if we are being asked to force the mode into float via the feature-in port.
          if (feature_in(false) == true) {                                                          // An external device is telling us to keep out of bulk/accept/OC.  (e.g, a LiFoPO4 BMC)
              if (chargingState != forced_float_charge)                                             // If not already in forced_float, change the state.
                set_charging_mode(forced_float_charge);                                                  
              }
            else {                                                                                  // Fearure_in() is no longer being held active.
             if ((chargingState == forced_float_charge) && ((requiredSensorsFlag & RQBatTempSen) == 0))  // All right, Feature-in port is NOT asking use to force float mode.   By chance, did it?
                 set_charging_mode(ramping);                                                             // Yes it had, and now it is no longer doing so.  So am external BMS must be asking us to restart charging.
                                                                                                         //  (Unless it also happens to be that the required battery temperature sensor is missing...)
            }
          }
   #endif
}









//------------------------------------------------------------------------------------------------------
//
// Update FEATURE_OUT port
//
//      This routine will manage the Feature Out port (other than Blinking) depending on how #defined are 
//      at the beginning of the program.  Note that 'communications' type usage of the Feature_out port
//      (Specifically, blinking of the light to mirror the LED in error conditions) is not handled here, but
//      instead as part of the BLINK_XXX() functions above.
//
//
//
//
//------------------------------------------------------------------------------------------------------


void update_feature_out(void) { 

   #ifdef FEATURE_OUT_LAMP                                                                      // This stub is here to prevent the user from enabling more than one feature at a time...
                                                                                                // Nothing here: LED mirror is handled in the LED() functions.

   #elif defined FEATURE_OUT_COMBINER                                                           // Enable FEATURE_OUT port to go active when VBat exceeds ????, useful for connecting external relay to join 2nd battery

        static bool combinerEnabled = false;                                                    // Is the combiner currently enabled?  Used in part to allow a hysteresis check before disabling on low voltage.
  


        switch (chargingState) {
          case acceptance_charge:                                                               // During the Acceptance Phase, see if we are configured for a time-out...
                
                if ((millis() - altModeChanged) > COMBINE_ACCEPT_CARRYOVER)
                           combinerEnabled = false;                                             //  ...disable combiner.

                break;




          case bulk_charge:
                                                                                                // Go through a nested tree of decisions to see if we should enable the combiner
                if (measuredBatVolts >= (COMBINE_CUTIN_VOLTS   * systemVoltMult))   combinerEnabled = true;     // 1st check:  is VBat above the cut-in voltage?   Yes, enable the combiner.
                                                                                                                // Continue the remaining nested checks outside of the SWITCH, in that way
                                                                                                                // the boundary tests are always preformed.  (e.g., during carryover in acceptance mode)
                break;


        
          default:
                 combinerEnabled = false;                                                                       // All other modes, disable combiner

          }


        if (measuredBatVolts >= (COMBINE_DROPOUT_VOLTS * systemVoltMult))   combinerEnabled = false;            // 2nd check:  if VBat is too high, disable combiner
        if (measuredBatVolts <  (COMBINE_HOLD_VOLTS    * systemVoltMult))   combinerEnabled = false;            // 3rd check:  if VBat is below the cut-out, disable combiner.
                                                                                                                // All other cases, just leave it in the state it already is in.  This will provide for
                                                                                                                // a level of hysteresis between the cutin volts and the hold-volts levels




        if (combinerEnabled == true)                                                            // Now that is known which state we want, set the feature_out port.
              _SET_FO_PORT(FOP_ON);     
           else 
              _SET_FO_PORT(FOP_OFF);       
                





   #elif defined FEATURE_OUT_ENGINE_STOP                                                        // Enable FEATURE_OUT port to go active when we enter FLOAT mode.  Useful to auto-stop a DC generator when charging has finished.
        switch (chargingState) {
          case float_charge:
          case forced_float_charge:
          case post_float:
          case FAULTED:
                  _SET_FO_PORT(FOP_ON);                                                         // If we are in Float, or Post_float:  Enable Feature-out to indicate battery is all charged.
                break;

          default:
                  _SET_FO_PORT(FOP_OFF);                                                         // All other modes, Disable Feature-out port
          }

   #endif
   }




















//------------------------------------------------------------------------------------------------------
// Check for Fault conditions 
//
//      This function will check to see if something has faulted on the engine.  
//
//      If a fault condition is found, the appropriate state will be set of FAULTED, the global variable 
//      faultCode will be set, and this function will return TRUE indicating a fault.
//
//      If no fault is found, this function will return FALSE.
// 
//------------------------------------------------------------------------------------------------------

bool  check_for_faults() {

        unsigned u;
 
        
        //----  Alternator doing OK?
        //      
        //      Take note how this Switch statement is structured, with many fall-thoughs to allow testing at approperate stages.
        //


        u = 0;                                                                                  // Assume there is no fault present.
        switch (chargingState)  {
                case unknown:
                case disabled:
                case pending_R:
                case post_float:
                        break;

        

                case ramping:
                case post_ramp:
                       #if defined (OSE_ALTERNATOR)  || defined (OSE_GENERATOR)           
                          if (max(measuredAltTemp,measuredAlt2Temp) >=  systemConfig.ALT_TEMP_SETPOINT)           u = FC_LOOP_ALT_TEMP_RAMP;
                          #endif                                                                // If we reach alternator temp limit while ramping, it means something is wrong.
                                                                                                // Alt is already too hot before we really even start to do anything...

                        //----   And at this point we also check to see if some of the REQUIRED sensors are missing.
                        //       If so, we set approperte flags to notify the rest of the code that different behaivious is exptected.
                        //
                        #if defined (OSE_ALTERNATOR)  || defined (OSE_GENERATOR)           
                        if ((systemConfig.REQURED_SENSORS & RQAltTempSen) && (measuredAltTemp == -99))          requiredSensorsFlag |= RQAltTempSen;  
                        #endif
                        if ((systemConfig.REQURED_SENSORS & RQBatTempSen) && (measuredBatTemp == -99))        { requiredSensorsFlag |= RQBatTempSen;  set_charging_mode(forced_float_charge); }

  

                case determine_ALT_cap:
                case acceptance_charge:
                case overcharge_charge:
                case RBM_CVCC:
                        if ((systemConfig.REQURED_SENSORS & RQAmpShunt) && (shuntAmpsMeasured != true))       { requiredSensorsFlag |= RQAmpShunt;     u = FC_SYS_REQIRED_SENSOR; } 
                                                                                                // By this time wew SHOULD have seen some indication of the amps present..   

                case bulk_charge:                                                               //  Do some more checks if we are running.
                        if (measuredBatVolts            > (FAULT_BAT_VOLTS_CHARGE  * systemVoltMult))           u = FC_LOOP_BAT_VOLTS;
                                                                                                //  Slightly lower limit when not equalizing.
                        break;



                case equalize:
                case float_charge:
                case forced_float_charge:
                        if  (measuredBatVolts           > (FAULT_BAT_VOLTS_EQUALIZE * systemVoltMult))          u = FC_LOOP_BAT_VOLTS;  
                                                                                                // We check for Float overvolt using the higher Equalize level, because when we 
                                                                                                // leave Equalize we will go into Float mode.  This prevents a false-fault, though it
                                                                                                // does leave us a bit less protected while in Float mode...

       
                        break;                                                                  // If we make it to here, all is well with the Alternator!



                default:                                                                        
                        u = FC_LOG_ALT_STATE;                                                   //  Some odd, unsupported mode.   Logic error!

                }




                    
        #if defined (OSE_ALTERNATOR)  || defined (OSE_GENERATOR)           
            if  (measuredAltTemp            > (systemConfig.ALT_TEMP_SETPOINT   * FAULT_ALT_TEMP))                  u = FC_LOOP_ALT_TEMP;
            if  (measuredAlt2Temp           > (systemConfig.ALT_TEMP_SETPOINT   * FAULT_ALT_TEMP))                  u = FC_LOOP_ALT2_TEMP;
            if ((measuredBatVolts           < (FAULT_BAT_VOLTS_LOW * systemVoltMult)) && (fieldPWMvalue > (FIELD_PWM_MAX - FIELD_PWM_MIN)/3))
                                                                                                                    u = FC_LOOP_BAT_LOWV;
                                                                                                    // Check for low battery voltage, but hold off until we have applied at least 1/3 field drive.
                                                                                                    // In this way, we CAN start charging a very low battery, but will not go wild driving the alternator
                                                                                                    // if nothing is happening.  (ala, the ignition is turned on, but the engine not started.)
            #endif
        
        
        
        
        if  (measuredBatTemp            >  FAULT_BAT_TEMP)                                                      u = FC_LOOP_BAT_TEMP;
 
        if ((requiredSensorsFlag != 0) && (systemConfig.REQURED_SENSORS & RQFault))   
                u = FC_SYS_REQIRED_SENSOR;                                                      // A required sensor is missing, and we are configured to FAULT out on that condition.





        if (u !=0) {
            chargingState = FAULTED;
            faultCode  = u;
            }







        //----  System doing OK?

        u = 0;                                                                                  //  Again, use 'u' to receive fault-code if a fault is found.  For not assume no fault.
        if (cpIndex           != (cpIndex & 0x07))                     u = FC_LOG_CPINDEX;      // cpIndex pointer out of range?
        if (systemAmpMult     != constrain(systemAmpMult, 0.0, 10.0))  u = FC_LOG_SYSAMPMULT;   // systemAmpMult out of range?

        #if defined (OSE_ALTERNATOR)  || defined (OSE_GENERATOR)           
           if  (measuredFETTemp   > FAULT_FET_TEMP)                    u = FC_SYS_FET_TEMP;     // Driver FETs overheating?
           #endif
            

        #ifdef SYSTEMCAN        
          if ((batteryInstance <= 0) || (batteryInstance > 100))        u = FC_LOG_BATTINST;    // Battery instance not between 1..100?
          #endif
        
        
        
                                                                        
        if (u !=0) {
          chargingState = FAULTED;                                                              //   Just use the chargingState to indicate these two system faults,
          faultCode  = u;
          }
                        





        if (chargingState     == FAULTED)     return (true);                                  //  Did something fault?
            else                              return (false);

}









//------------------------------------------------------------------------------------------------------
//
//      Handle FAULT condition
//
//
//      This function is called when the system is in a FAULTED condition.  It will make sure things are
//      shut down, and blink out the appropriate error code on the LED.
//
//      If the Fault Code has the 'restart flag' set (by containing 0x8000), after blinking out the fault code
//      the regulator will be restarted.  Else the regulator will continue to blink out the fault code.
//
//
//------------------------------------------------------------------------------------------------------

void handle_fault_condition() {

        unsigned j;
        char buffer[80];

        //-----  Make sure the alternator, etc. is stopped.
       set_ALT_PWM(0);                                                                           // Field PWM Drive to Low (Off) 
        #ifdef CHARGE_PUMP_PORT  
            analogWrite(CHARGE_PUMP_PORT,0);                                                     // and the Charge Pump as well.
            #endif
                
 

        j = faultCode & 0x7FFFU;                                                                // Masking off the restart bit.

        snprintf_P(buffer,sizeof(buffer)-1, PSTR("FLT;,%d,%d\r\n"),                             // Send out the Fault Code number and the Required Sensor Flag.
                         j,
                         requiredSensorsFlag); 



        ASCII_write(buffer);
        send_outbound(true);                                                                    // And follow it with all the rest of the status information.
        while (send_outbound(false));                                                           //  (Keep calling until all the strings are pushed out)


        blink_LED (LED_FAULTED, LED_RATE_FAST, 2, true);                                        // Blink out 'Faulted' pattern to both LED and LAMP
        while (refresh_LED() == true);                                                          // Send the pattern out to the LED.

        delay (1000);                                                                           // 1 second pause

        

        if (j >= 100)
           blink_out (((j/100) % 10));                                                          // Blink out Fault number 100's 1st

        blink_out (((j/ 10) % 10));                                                             // Blink out Fault number 10's 2nd
        blink_out ((j       % 10));                                                             // Blink out Fault number units last
        
        delay (5000);                                                                           // Pause 5 seconds
                                        
}




void blink_out(int digd) {

        static const PROGMEM  unsigned num2blinkTBL[9] = {                                      // Table used convert a number into a blink-pattern
        0x0000,                                                                                 // Blink digit '0'  (Note, Arduino pre-processor cannot process binary decs larger than 8 bits..   So, 0x____)
        0x0002,                                                                                 // Blink digit '1'
        0x000A,                                                                                 // Blink digit '2'
        0x002A,                                                                                 // Blink digit '3'
        0x00AA,                                                                                 // Blink digit '4'
        0x02AA,                                                                                 // Blink digit '5'
        0x0AAA,                                                                                 // Blink digit '6'
        0x2AAA,                                                                                 // Blink digit '7'
        0xAAAA                                                                                  // Blink digit '8'      For 9, you will need to blink out "1"+8 :-)
        };                                                                                      //   (BTW: This 1+8 is why the other digits appear to be 'shifted left' one bit, to make a smooth LED blink when doing a 9



     if (digd == 9) {
           blink_LED (0x0001, LED_RATE_NORMAL, 1, true);                                        // 9 has to be sent as an "1"+8
           while (refresh_LED() == true);       
           digd--;
           }

      blink_LED (pgm_read_dword_near(num2blinkTBL+digd), LED_RATE_NORMAL, 1, true);                             
      while (refresh_LED() == true);                                                            // Send the pattern out to the LED.
      delay (750);
      }











      
      
      
#ifdef CPU_STM32

 /***************************************************************************************
 ****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              NMEA2000 Bridge Functions                               *
 *                                                                                      *
 *                                                                                      *
 *      Normally the NMEA2000 lib will automatically include a 'bridge' library,        *
 *      such as NMEA2000_AVR.  But for the HAL / cubeMX deployment of the STM32F        *
 *      there is no existing bridge library integrated with the NMEA2000 lib.           *
 *      As such we need to include some object bridges here.   
 //!! HEY!!  Move this into a separate module of bridge functions, like NMEA2000_AVR, maybe call it:  NMEA2000_STM32_HAL or NMEA2000_STM32_cubeMX  -- and post it to git-hub.
 *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************
 ****************************************************************************************/







//*****************************************************************************
bool tNMEA2000_cubeMX::CANOpen() {
    
     CAN_FilterConfTypeDef FilterConfig;
    
    
 
    //--- Finish flushing out the hcan handle so we are able to use it.
    static CanTxMsgTypeDef TxMessage;                                                       // Add Tx and Rx transfer buffers (Surpirsed they were not there already!)
    static CanRxMsgTypeDef RxMessage;
    static CanRxMsgTypeDef Rx1Message;
    hcan.pTxMsg  = &TxMessage;                                                              // One Tx fifo
    hcan.pRxMsg  = &RxMessage;                                                              // Two Rx Fifo's
    hcan.pRx1Msg = &Rx1Message;      
    
    //can1.frequency(250000);  --> Frequency is configured by cubeMX as part of the clock definitions.
    

    if(HAL_CAN_Init(&hcan) != HAL_OK)  {
        /* Initialization Error */
        return(false);
        }

        
        
    //---   Configure the CAN Filters.  Higher priority things to RX0, rest to RX1
    FilterConfig.FilterNumber       = 0;
    FilterConfig.FilterMode         = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale        = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh       = 0x0000;
    FilterConfig.FilterIdLow        = 0x0000;
    FilterConfig.FilterMaskIdHigh   = 0x0000;
    FilterConfig.FilterMaskIdLow    = 0x0000;
    FilterConfig.FilterFIFOAssignment = CAN_FIFO0;
    FilterConfig.FilterActivation   = ENABLE;
    FilterConfig.BankNumber         = 14;
    
        

    if(HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK) {
        // Filter configuration Error 
        Error_Handler();
        }

      
        
        
        
  
    return(true);  
}





//*****************************************************************************
bool tNMEA2000_cubeMX::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{

    
    hcan.pTxMsg->RTR   = CAN_RTR_DATA;
    hcan.pTxMsg->IDE   = CAN_ID_EXT;
    hcan.pTxMsg->DLC   = len;
    hcan.pTxMsg->ExtId = id;

   for (int i=0; i<len && i<8; i++) hcan.pTxMsg->Data[i]=buf[i];

   if(HAL_CAN_Transmit(&hcan, 10) != HAL_OK) {                             // try to place it into a Tx mailbox, but only for 10mS before timing out.
        /* Transmition Error */
        return(false);
        }
    

    //if (wait_sent){
    //     //---- Must have been a Fast Packet, and Tx order needs to be kept.  But we are already in FIFO mode, so no need to wait.
    //     }

    return (true);
}






//*****************************************************************************
bool tNMEA2000_cubeMX::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
//!! HEY!!  This is kind of a temp solution - rework to use IRQs, and perhaps a queue for Rx of CAN.  (Lower power, better performance).
     
    if (HAL_CAN_Receive(&hcan, CAN_FIFO0, 0) == HAL_OK) {                            // See if something has arrived, but do not wait around if not.
        id=hcan.pRxMsg->ExtId;
        len=hcan.pRxMsg->DLC;
        for (int i=0; i<len && i<8; i++) buf[i]=hcan.pRxMsg->Data[i];
        return(true);    
        }
    
    if (HAL_CAN_Receive(&hcan, CAN_FIFO1, 0) == HAL_OK) {                            //!! HEY!!  Hum...  As I have no filters set up, will there really be anything here?
        id=hcan.pRx1Msg->ExtId;
        len=hcan.pRx1Msg->DLC;
        for (int i=0; i<len && i<8; i++) buf[i]=hcan.pRx1Msg->Data[i];
        return(true);    
        }
    
    return(false);
        
} //!! HEY!!  Change this to interupt driven call-back "HAL_CAN_RxCpltCallback()", receive the message and also send  an RTOS 'WAKE UP' signal to the main loop.
  //    change the mainloop to go into a mad int32_t sleep mode until it receives a wakeup signal from the RTOS.  hav the CAN Rx send taht signal, look to see of the USB can
//      send one upon an Rx, and even look at sensors.cpp to send the signal if it notes a change in the switches status???
//      Need something for a wrapper around CAN - if system is in sleep mode need to put into place filters to only look for key messages (Start Generator, a Request_PGN message,
//      maybe the terminal message???  (Anything critical in the NMEA2000 lib?? - ala house-keeping?).  
//
//      Perhaps HAL_PCD_DataInStageCallback() can be used in a like way for the USB???






/********************************************************************
*    Other 'Bridge' functions and classes
*
*
*
**********************************************************************/

int tcubeMXStream::read() {
    //-- Take note, this is the normal STDIO - aka, the default UART.  It is NOT the USB serial stream for that see: USB_ASCII_READ()
    // It is here only to keep the NMEA2000 lib from complaining.   Maybe in the future, just have it return -1 each time...
    
  if (!feof(stdin))
      return (getc(stdin));                                                     // Something to read!  Go get it and return it.
  else
      return -1;                                                                // Nothing to read,
}






size_t tcubeMXStream::write(const uint8_t* data, size_t size) {

    CDC_Transmit_FS((uint8_t*)data, size); 

    return size;
    
}



uint32_t millis(void) {
    return (HAL_GetTick());
}

void delay (uint32_t ms) {
     //HAL_Delay(ms);  
    osDelay(ms);                                                                    // We are in an RTOS framework, don't just sit around burring HAL cycles..
}
    
         
      

#endif  /* CPU_STM32 */
      
      




       


