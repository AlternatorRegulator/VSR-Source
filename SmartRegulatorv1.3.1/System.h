//      System.h
//
//      Copyright (c) 2018 by William A. Thomason.                  http://arduinoalternatorregulator.blogspot.com/
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



#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "Config.h"
#include <RVCMessages.h> 




typedef enum tModes {unknown   =0, disabled, FAULTED, FAULTED_REDUCED_LOAD, sleeping,                                           // Used by most or all  (0..9)
                     pending_R=10, ramping, determine_ALT_cap,   post_ramp=15,  bulk_charge=20, acceptance_charge, overcharge_charge,  float_charge=30, forced_float_charge, post_float=36, equalize=38, RBM_CVCC,
                                                                                                                                // Alternator / Charger Mode specific (10..39)
                                                                                                                                //  (BMS uses a subset for chargerMode, keeping order to simplify external ASCII apps.)
                     discharge=40, recharge, LTStore, holdSOC,                                                                  // BatMon specific  (4x)
                     stopped  =50, stopping, starting, pre_warming, warming, running,    priming_oil=59,                        // Generator engine specific  (5x)
                     pending_WM=60, WM_enabled, post_WM                                                                         // Watermaker (Aug load for DC Gen)  (6x)
                    } tModes;                                                                                                   // Take care not to change the order of these, some tests use
                                                                                                                                // things like <= pending_R









                    
                                //----- This structure is used to hold many global 'system' configurations values.  By placing them into a Structure (as opposed to
                                //      using #define statements) there is the ability to modify them during run-time, and even save them into FLASH.

#define MAX_NAME_LEN  18                                         // Up to 18 characters in the NAME, also 18 for the PASSWORD used in the Bluetooth and CAN.
#define MAX_PIN_LEN   18                                         //  (#define needs to be here as Arduino has poor pre-processing and cannot look ahead)
                                                                //  !!!! Something very odd here...  The size of SCV struct seems to be at a crossover point.  By making ONE of these
                                                                //   18 instead of 20, I save over 100 bytes of code size...  No idea why, but with both at 18 I save 110 bytes of
                                                                //   complied code, and 110 bytes if huge!!   So, 18 it is...

                    
#if defined (OSE_ALTERNATOR)  || \
    defined (OSE_GENERATOR)
    
typedef struct  {                                               // System Configuration Structure used by Alternator and DC Generator

   bool         REVERSED_SHUNT;                                 // Is the AMP Shunt wired up backwards?  If so, this Flag will invert the readings via Software.                
                                                                //------  Default values for Alternator Goals (Amps, temp, etc..)
   uint8_t      ALT_TEMP_SETPOINT;                              // Temp range we want the Alternator to run at (max). Anything above it, start backing off Alternator Field drive.
   float        ALT_AMP_DERATE_NORMAL;                          // In normal operation, run the Alternator this % of it  demonstrated max capability
   float        ALT_AMP_DERATE_SMALL_MODE;                      // But if user has selected Small Alternator Mode, then scale back to a lower %
   float        ALT_AMP_DERATE_HALF_POWER;                      // If user shorts out the Alt Temp probe, we will then use this value to derate.
   int          ALT_PULLBACK_FACTOR;                            // Exponential factor in RPM based pull-back factor; to protect Alts by reducing max field PWM at low RPMs
                                                                // How many RPMs are needed to increase idle-pull-back 1% above 1/4 power at idle.
                                                                // Range 0..10, set = 0 to disable pullback feature.  See set_VAWL() in manage_ALT();
                                                                // Set = -1 to trigger fixed 70% cap when stator signal is not seen.
   int          ALT_IDLE_RPM;                                   // Used in conjunction with PBF to manage Field Drive at lower RPMs.  Establishes the 'starting point' for 1/4 field drive.
                                                                // Range 0..1500, set = 0 to enable auto determination of Idle RPMs. 
   
   int          ALT_AMPS_LIMIT;                                 // The regulator may OPTIONALLY be configured to limit the maximum number of AMPs being drawn from the alternator.
                                                                // For example if you have small wire sizes in the alternators wiring harness. Place the size of the Alternator here. 
                                                                // If this is set = -1, the regulator will enable a auto-size-determining feature and during Bulk phase it will for
                                                                // short periods of time drive the alternator very hard to measure its Amps viability.  (this is triggered any time an
                                                                // increase in noted Amps exceeds what was noted before, OR if an increase in RPMS is noted..)
                                                                // CAUTION:  During Auto-size phase Full-Drive will occur EVEN if the half-power mode has need selected (by shorting the Alt NTC sender wires)
                                                                // Set this = 0 to disable AMPS based management of the alternator, the regulator will simply drive the Alternator as hard as it can.
                                                                // CAUTION:  If Amps based management is disabled, then features such as Small Alternator, and Half Power will no longer function.
                                                                //          You also run the risk of driving the FIELD at Full Field, with no increase in Alternator output - causes excessive heating in the field.



   int          ALT_WATTS_LIMIT;                                // The regulator also has the ability to limit the max number of Watts load being placed on the engine.  For example if you
                                                                // are using a large alternator on a small engine.  To enable this feature, place the max # of WATTS you want the alternator 
                                                                // to deliver here and the regulator will cap total engine load to this value.  Note this is loosely related to ALT_AMPS_LIMIT,
                                                                // but Watts is a true measurement when talking about loads being placed on engines, as Amps do not take into account the 
                                                                // current battery voltage..
                                                                // Set this to 0 to disable this feature let the regulator drive to the max capability of the Alternator (or the defined 
                                                                // ALT_AMPS_LIMIT above)
                                                                // Set = -1 to auto-calc the Watts limit based on the determined (or defined) size of the Alternator and the current target charge voltage.

                                                                // Note that ALT_WATTS_LIMIT and ALT_AMPS_LIMIT values are NOT adjusted by the sensing of the battery nominal voltage, If you enter 200A and
                                                                // 5000W, the regulator will cap at 200A or 5000W independent of the battery voltage...



                                                                // ------ Parameters used to calibrate RPMs as measured from Alternator Stator Pulses
                                                                //        IF you wish to read 'engine RPMs' from the regulator, you will need to adjust these.  But w/o any adjustment the 
                                                                //        regulator will function correctly, for its purpose.
   uint8_t      ALTERNATOR_POLES;                               // # of poles on alternator 
   float        ENGINE_ALT_DRIVE_RATIO;                         // engine pulley diameter / alternator diameter



                                                                //----   Controller Hardware configuration parameters   Here hardware options are noted.  See Blog for more details
   int          AMP_SHUNT_RATIO;                                // Spec of amp shunt, Amp/mV * % Calibration error (e.g:   250A / 75mV shunt * 100% -->  250 / 0.075 * 1.00 = 3,333) 
                                                                //  Note if Shunt voltage will exceed 80mV, then the INA-226 calibration will need to be adjusted.

   int          FIELD_TACH_PWM;                                 // If the user has selected Alternator Tach mode via the DIP switch, then this value will be used for the minimum Field PWM
                                                                // (As opposed to 0).  If this is set = -1, then during initial RAMP the field value to use for Tach mode will be auto-determined
                                                                // and set to a value just before Amps are produced by the alternator.  Set = 0 to disable Tach mode, independent of the DIP switch.


   bool         FORCED_TM;                                      // Has user Forced on Tach-mode via the $SCT command


   bool         USE_BT;                                         // User configurable, Use or Disable Bluetooth for this module?  
   char         DEVICE_NAME[MAX_NAME_LEN+1];                    // What should THIS regulator's name be, in a string.  (+1 to hold the string NULL terminator)                
   char         DEVICE_PSWD[MAX_PIN_LEN+1];                     // Password to be programmed into the Bluetooth. 

   bool         BT_CONFIG_CHANGED;                              // Has the BT Configuration been changed from the Defaults?  (This is used to disable the ability to update charge parameters 
                                                                // or the system config until the Bluetooth is a bit more secure then the as-compiled defaults)

   uint8_t      CP_INDEX_OVERRIDE;                              // User has used issues command to override the DIP switches for the cpIndex.      -1 = use DIP switches.
   float        BC_MULT_OVERRIDE;                               // User has used issues command to override the DIP switches for the bcMultiplier.  0 = use DIP switches.
   float        SV_OVERRIDE;                                    // User forced Voltage Multiplier (1..4) associated with 12v..48v.  Use 0 to enable auto-detect feature.
   uint8_t      CONFIG_LOCKOUT;                                 // 0=no lockout, 1=no config change, 2=no change, no clearing via FEATURE-IN. 
   int          ENGINE_WARMUP_DURATION;                         // Duration in seconds alternator is held off at initial power-on before starting to apply load to engine (Start the RAMP phase)
   uint8_t      REQURED_SENSORS;                                // Flags to indicate the regulator should check if some sensors are not present, ala Alt Temp, battery shunt, etc..
   
   #ifndef      SMALL_FLASH 
     uint8_t    SCSPLACEHOLDER[15];                             // Room for future expansion   
     #endif 
   } tSCS;


                    
#elif defined (OSE_BATMON)  || \
      defined (OSE_BMS)
      
         
typedef struct  {                                               // System Configuration Structure used by Alternator and DC Generator

   bool         REVERSED_SHUNT;                                 // Is the AMP Shunt wired up backwards?  If so, this Flag will invert the readings via Software.                
       
                                                                //----   Controller Hardware configuration parameters   Here hardware options are noted.  See Blog for more details
   int          AMP_SHUNT_RATIO;                                // Spec of amp shunt, Amp/mV * % Calibration error (e.g:   250A / 75mV shunt * 100% -->  250 / 0.075 * 1.00 = 3,333) 
                                                                //  Note if Shunt voltage will exceed 80mV, then the INA-226 calibration will need to be adjusted.
   bool         USE_BT;                                         // User configurable, Use or Disable Bluetooth for this module?  
   char         DEVICE_NAME[MAX_NAME_LEN+1];                    // What should THIS regulator's name be, in a string.  (+1 to hold the string NULL terminator)                
   char         DEVICE_PSWD[MAX_PIN_LEN+1];                     // Password to be programmed into the Bluetooth. 

   bool         BT_CONFIG_CHANGED;                              // Has the BT Configuration been changed from the Defaults?  (This is used to disable the ability to update charge parameters 
                                                                // or the system config until the Bluetooth is a bit more secure then the as-compiled defaults)
   int          BAT_AMPS_LIMIT;                                 // OPTIONALLY be configured to limit the max amps the battery may deliver 
                                                                //    Set = 0 to disable Amps capping. 
   int          BAT_WATTS_LIMIT;                                // OPTIONALLY be configured to limit the max load (discharging) placed on battery.
                                                                //    Set = 0 to disable Amps capping. 
   uint8_t      CP_INDEX_OVERRIDE;                              // User has used issues command to override the DIP switches for the cpIndex.      -1 = use DIP switches.
   float        BC_MULT_OVERRIDE;                               // User has used issues command to override the DIP switches for the bcMultiplier.  0 = use DIP switches.
   float        SV_OVERRIDE;                                    // User forced Voltage Multiplier (1..4) associated with 12v..48v.  Use 0 to enable auto-detect feature.
   uint8_t      CONFIG_LOCKOUT;                                 // 0=no lockout, 1=no config change, 2=no change, no clearing via FEATURE-IN. 
   uint8_t      REQURED_SENSORS;                                // Flags to indicate the regulator should check if some sensors are not present, ala Alt Temp, battery shunt, etc..
   
   uint8_t    SCSPLACEHOLDER[16];                               // Room for future expansion   
   } tSCS;

#endif

                                                                // Bit fileds for REQUIRED_SENSORS above and global variable: requiredSensorsFlag
const uint8_t RQAltTempSen = 0x01;                              // Alternator Temp Sensor Required  -- Go to half-power mode if missing
const uint8_t RQBatTempSen = 0x02;                              // Battery Temp Sensor Required     -- Force into float if missing
const uint8_t RQAmpShunt   = 0x04;                              // Amp shunt Required               -- Fault if missing

const uint8_t RQEngTempSen = 0x80;                              // Engine Temp Sensor Required      --  Go to half-power mode, stop Watermaker
const uint8_t RQEGTTempSen = 0x10;                              // EGT Temp Sensor Required         -- Go to half power mode, stop Watermaker, full throttle
const uint8_t RQSwpTempSen = 0x20;                              // Sea-Water Pump Temp Sensor Required -- Fault if missing
const uint8_t RQWMPSISen   = 0x40;                              // Watermake PSI Sensors Required   -- Disable Watermaker if either is missing.

const uint8_t RQFault      = 0x80;                              // If any of the required sensors are missing, just force a FAULT vs. taking the 'default' action.
   

   
   
   
   

                   
                                //----- This structure is used to hold GENERATOR configurations values.  
 

typedef struct  {                                               // Generator Configuration Structure

     
    int             CHK_OP_SENDER;                              // What PSI is considered sufficent oil pressure.  Checked before starting (Faults if OP is senses) as well
                                                                // as when engien state == running.   
                                                                // Set = 0 to disable oil-pressure checks.
                                                                // Set = -1 to indiacte sender is N.O. sender (Open = no oil pressure)
                                                                // Set = -2 to indiacte sender is N.C. sender (Open = sufficent oil pressure)
  
    bool            CHK_WF_SENDER;                              // Should we be monitoring the cooling water flow sensor for faults?
 


    
    

    //---  STARTING parameters
    //      Change with $ESC    (Engine Starter Config)
    tRVCGenStrTyp   STARTER_INTERFACE;                          // What type of engine start/stop mechanism are we attached to?  
                                                                //      "RCI  -- Run / Crank Input"
                                                                //      "CGSI -- Crank/Glow & Stop inputs
                                                                //      "PSPS --  Preheat/Start input and Prime/Stop 
                                                                //      "SIO  -- Single On/Off input 
                                                                //      "GSCA -- Glow, Start input with Amp Shunt monitoring" method
    
    uint32_t        THROTTLE_TO_IDLE_ADVANCE;                   // Amount of time (in mS) used to move the throttle cable from Full Stop to the presumed
                                                                // idle (starting) position.  It is used in start_engine() to initially position the throttle cable during start up.
                                                                // You will need to experiment some to get this number dialed in for your system.  Set = 0 to disable any initial
                                                                // throttle advancment before engaging starter for 1st time.
  
    uint32_t        GLOW_PLUG_DURATION;                         // Time (in mS) to enable Glow Plug before starting to crank.  0 = disable glow-plug  
    uint32_t        ENGINE_START_CRANK;                         // Crank for no more than this time (in mS) per starting attempt cycle . . 
    uint32_t        START_THROTTLE_BUMP;                        // Between start attempt cycles, advance the throttle this much more. (in mS)  
                                                                //    (Set this to 0 if you do not want the throttle to advance between start attempts. )
    uint32_t        ENGINE_START_REST;                          //     . . . rest this many mS before typing again
    int             ENGINE_MAX_START_TRY;                       // And try starting this many times (cycles) before giving up.
 
    unsigned int    STARTING_FIELD_PWM;                         // If we are to use the Alternator RPMs to indicate the Engine has started, this is the Field PWM value sent 
                                                                //  during cranking.

    uint32_t        STARTING_HOLDOFF;                           // Just after engaging the starter, hold off this many mS before looking at things to allow the initial surge from
                                                                // from the starter to pass.

    float           STARTED_AMPS_THRESH;                        // Ratio (0..1.0) applied to measured amperage vs. peak amperage noted which will indicate the starter is unloading;
                                                                // presumable due to the engien taking up. Set = 0 to disable started determination via Started Amp draw.
    unsigned int    STARTED_RPM_THRESH;                         // Number of measured RPMs which we will take to indicate the engine has started.  Set = 0 to disable started sensing
                                                                // via Engine RPMs.
    uint32_t        ENGINE_POST_START_HOLD;                     // Keep the starter engaged for xx mS after started, just to be sure....
                                                              
   





    //---  RUNNING paramters, used to adjust the trottle while running.
    //     Change with $ETC (Engine Throttle Config)
    uint32_t        ENGINE_PREWARMUP_DURATION;                  // Number of mS after starting to just let the engine idle before advancing the throttle.
    uint32_t        ENGINE_WARMUP_DURATION;                     // Time (in mS) over which we will slowly advance the throttle cable during a 'warm-up'.  
    uint32_t        WARMUP_THROTTLE_BUMP;                       // During warm-up we will advance the throttle this many mS 20x times.  Set = 0 to disable throttle warmup 
                                                                // advancment.
    uint32_t        THROTTLE_BUMP;                              // When adjusting the throttle (faster or slower), bump it this many mS.  (Set = 0 to disable throttle adjustments and 
                                                                // just run fixed-speed.
    uint32_t        THROTTLE_SETTLE_PERIOD;                     //  And wait this int32_t (in mS) after 'bumping' the throttle before making another adjustment. 

    int             ENGINE_IDLE_RPM;                            // Spec sheet idle RPMs.
    int             ENGINE_MAX_RPM;                             // Spec sheet max RPMs  -- These are combined with the ALT_PULLBACK_FACTOR to calculate needed RPMs for a given load
    int             ENGINE_CONT_HP;                             // Spec sheet engine continuous HP at Max RPMs.
    int             ENGINE_QUITE_RPMS;                          // If user asks to startup in Quite Mode, we will limit the RPMs to this value.
    float           ALTERNATOR_EFFICIENCY;                      // Defined efficency % (1--1.0) of alternator.             
//!! HEY!!  Do I need this????   unsigned int    FULL_LOAD_RPM_DROP;                         // How many RPMs is it expected the engine will drop between an unloaded condition (at Full Throttle)
                                                                // and Full Load.  This is used in calculate_ALT_targets() when searching the RPM/Watt array for the 
                                                                // set point WATTs based on demonstrated unloaded RPMs.   Set = 0 to disable this capability.






    //----  STOPPING parameters
    uint32_t        ENGINE_POST_STOP_LOCKOUT;                   // After a stop-request has been made, lockout any additional starting request this many mS.  To assure engine has
                                                                // truly stopped after disengaging the throttle cable / engine-run signals.
    uint32_t        ENGINE_COOLDOWN;                            // Time in mS that engine should be allowed to run at idle with no load before turned off.
    










     uint8_t    SCSPLACEHOLDER[32];                             // Room for future expansion 


} tGCS;

                               
                                
  




        
                //----    Error codes. If there is a FAULTED status, the variable errorCode will contain one of these...
                //              Note at this time, only one error code is retained.  Multi-faults will only show the last one in the checking tree.
                //              Errors with + 0x8000 on them will cause the regulator to re-start, others will freeze the regulator.
                //              (Note combinations like 10, and 11 are not used.  Because one cannot flash out 0's, and kind of hard to 
                //               tell if 11 is a 1+1, or a real slow 2+0)



#define FC_LOOP_BAT_TEMP                12              // Battery temp exceeded limit
#define FC_LOOP_BAT_VOLTS               13              // Battery Volts exceeded upper limit (measured via INA226)
#define FC_LOOP_BAT_LOWV                14  + 0x8000U   // Battery Volts exceeded lower limit, either damaged or sensing wire missing. (or engine not started!)

#define FC_LOOP_ALT_TEMP                21              // Alternator temp exceeded limit
#define FC_LOOP_ALT_RPMs                22              // Alternator seems to be spinning way to fast!
#define FC_LOOP_ALT2_TEMP               23              // Alternator #2 temp exceeded limit
#define FC_LOOP_ALT_TEMP_RAMP           24              // Alternator temp reached / exceeded while ramping - this can NOT be right, to reach target while ramping means way too risky.


#define FC_LOG_ALT_STATE                31              // Global Variable chargingState has some unsupported value in check_for_faults() 
#define FC_LOG_ALT_STATE1               32              // Global Variable chargingState has some unsupported value in manage_ALT() 
#define FC_LOG_CPI_STATE                33              // Global Variable cpIndex has some unsupported value in caculate_ALT_Targets() 
#define FC_LOG_CPINDEX                  34              // Global Variable cpIndex has some unsupported value in check_for_faults() 
#define FC_LOG_SYSAMPMULT               35              // Global Variable systemAmpMult has some unsupported value in check_for_faults() 
#define FC_LOG_BAT_STATE1               36              // Global Variable chargingState has some unsupported value in manage_BAT()    
        
#define FC_SYS_FET_TEMP                 41              // Internal Field FET temperature exceed limit.
#define FC_SYS_REQIRED_SENSOR           42              // A 'Required' sensor is missing, and we are configured to FAULT out.


#define FC_CAN_BATTERY_DISCONNECTED     51              // We have received a CAN message that the battery charging bus has been disconnected.;
#define FC_CAN_BATTERY_HVL_DISCONNECTED 52              // We have noted that a command has been sent asking for the battery bus to be disconnected!
#define FC_LOG_BATTINST                 53              // Battery Instance number is out of range (needs to be from 1..100)

#define FC_ENGINE_UNSUPPORTED_INTERFACE  61             // Attempted to start and/or stop engine with unsupported 'interface' configuration

#define FC_ADC_READ_ERROR                71             // Internal error - unable to use ADC subsystem

#define FC_INA226_READ_ERROR            100 + 0x8000U   // Returned I2C error code is added to this, see I2C lib for error codes.










//---  Global vars and prototypes

extern tSCS     systemConfig;
extern tGCS     genConfig;


extern tModes   chargingState;
extern tModes   systemState;
extern tModes   batteryState;
extern tModes   waterMakerState;

extern unsigned faultCode; 
extern uint8_t  requiredSensorsFlag;
extern char const firmwareVersion[];




bool    feature_in(bool waitForDebounce);
void    handle_feature_in(void);
void    update_feature_out(void);
bool    check_for_faults(void) ;
void    handle_fault_condition(void);





    
    

#endif   /*  _SYSTEM_H_  */
