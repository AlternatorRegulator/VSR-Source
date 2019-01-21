//
//      SmartRegulator.h
//
//      Copyright (c) 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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


#ifndef _SMARTREG_H_
#define _SMARTREG_H_


                                                       
/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              CUSTOMIZING PARAMETERS                                  *
 *                                                                                      *
 *                                                                                      *
 *      Change the following parameters to meet the exact needs of your installation.   *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/


#define OSE_RBM                                                 // And indicate if this device is able to act as a RBM  (Regulaor does have sufficient instrumentation capabilities)




#define FEATURE_OUT_LAMP                                        // Enable FEATURE_OUT port as a LAMP driver, to show ruff status as well as blink out the FAULT code
//#define FEATURE_OUT_ENGINE_STOP                               // Enable FEATURE_OUT port to go active when we enter FLOAT mode.  Useful to auto-stop a DC generator when charging has finished.
//#define FEATURE_OUT_COMBINER                                  // Enable FEATURE_OUT port to go active when alternator is in Accept or Float phase.  That is when the main battery had been 'bulked up'.
                                                                //   Useful for connecting external relay to join 2nd battery, but you need to make sure to set the Capacity DIP switches to reflect the 
                                                                //   total capacity of the TWO (or more) batteries when combined.
                                                                // CAUTION:  Make sure to only pick ONE of these Feature_out port usages, else who knows what you will get!

#define FEATURE_IN_EQUALIZE                                     // Enable FEATURE_IN port to select EQUALIZE mode while regulator is operating
#define FEATURE_IN_RESTORE                                      // Enable FEATURE_IN port to be used during Startup to restore the Regulator to as-shipped (compiled) state if actived at inintal power-on  
                                                                //      NOTE:  "restore" may be combined with other Feature_in defines, as it impacts only startup operation, not running.
                                                                //              "Restore" is only effective if user selects Charge Profile #6


#define FEATURE_IN_DISABLE_CHARGE                               // Enable FEATURE_IN port to prevent entering active charge modes (Bulk, Acceptance, Overcharge) and only allow Float or Post_Float.
                                                                //   This capability will ONLY be active if CPE #8 is selected, and it will also prevent other feature_in options from being usable
                                                                //   with CPE #8.



//#define DISABLE_ON_J1939_RPMs                                  // Should the regulator monitor J1939 engine RPMs to decide when to start up?  If Engine RPMs are not noted, we will assume the engine 
                                                                // is not running and will hold the Charger Status in DISABLED mode.  Once we see RPMs we will start ramping up.
                                                                // If NJ1939 RPMs are lost down the road, we will force the regulator into DISABLED mode.
                                                                //  To use this feature, it is common to always power the ENABLE input on the regulator.
                                                                // A bit experiential, as well as uncommon, hence disabled by default. Requires SYSTEMCAN to be defined.                                                             
                                                                
                                                         



       


                                //----   COMBINER VALUES
                                //       Used to define how the feature-out combiner will work.
                                //       note:  all voltages are in 'nominal' 12v form, and will be adjusted at runtime by the systemVoltMult, but not battery temperature...
#ifdef FEATURE_OUT_COMBINER

  #define       COMBINE_CUTIN_VOLTS     13.2                            // Enable to combiner once OUR battery voltage reaches this level.
  #define       COMBINE_HOLD_VOLTS      13.0                            // Once enabled remain so on even if voltage sags to this value - a hysteresis to reduce relay chatter/cycling.
  #define       COMBINE_DROPOUT_VOLTS   14.2                            // But then disable the combiner once OUR battery voltage reaches this level.           
                                                                        // This is for cases where we are looking to get a boost from the other battery during
                                                                        // bulk phase, but do not want to run the risk of 'back charging' the other battery
                                                                        // once voltage raises a bit.   --HOWEVER--
  #define       COMBINE_ACCEPT_CARRYOVER   0.75*3600000UL               // If we indeed want to continue on and back charge the other battery (say, in the case the other
                                                                        // battery has no other charging source, ala a bow thruster battery), then this is a brute-force
                                                                        // carry over into the Accept phase, keep the combiner on for this many hours. (0.75, or 45 minutes)
                                                                        // If you want this to actually WORK, then you will need to raise the dropout voltage above to a higher
                                                                        // threshold, perhaps 15v or so??
                                                                        // See documentation for more details around these parameters and how to configure them..
  #endif




  
  
 





         





                                // --------  BOARD SPECIFIC SELECTIONS
                                //          This source supports two version of the Arduino Regulator:
                                //              - Original version utilizing the ATmega328 CPU (Through hole w/optional Bluetooth module)
                                //              - 2nd gen version using the ATmegaxxM1 uC and supporting a CAN bus (but no Bluetooth)
                                //
                                //          Here we will detect which CPU is being compiled, and make the appropriate port selections
                                //          for I/O ports and additional include files as needed.
                                //
                                //          Note that ALL I/O PORTS are to be defined here, in one common location.  
                                //          This will help avoid issues with duplicate errors...
                                
                   
                                
#if defined (__AVR_ATmega328P__) || \
    defined (__AVR_ATmega328__)                             // Original ATmega328 based alternator regulator.
       
    #define STANDALONE                                      // Build flag to select different code for this Stand-along regulator.
    #define CPU_AVR                                         // Utilize the Atmel AVR I2C hardware (sensors.cpp)
    #include "Portability.h"                                // Pick up all the IDE specifics, ala PROGMEM
                                           
                                                
          
                                                
    #define OPTIBOOT                                            // Compile this code for use with Optiboot bootloader?  If so, fully enable the Watchdog timer.
                                                                // If Optiboot is not used, and the 'default' Arduino IDE bootloader is used, there are some workarounds
                                                                // needed to make reboot() function, and if the watchdog is ever triggered the regulator will hang vs.
                                                                // restart.  This is a fault / bug in the basic Arduino bootloader for 3.3v + atMega328.  See blog for more details.

                                              
      
      //----   Origional 'Uno' based regulator.
    #define FEATURE_OUT_PORT                 3              // Drives Feature-out open-collector port.
    #define LED_GREEN_PORT                   8              // Connected to status LED on the PCB.
    #define LED_RED_PORT                     8              // The original PCB only has a green LED, so use the same port# for both.
    #define FIELD_PWM_PORT                   9              // Field PWM is connected to Atmel pin-15 (port 9 on Arduino)
    #define CHARGE_PUMP_PORT                 6              // PWM port that drives the FET boost-voltage Charge Pump
    #define BT_STATE_PORT                   10              // Bluetooth STATE status sensing port.
    #define FEATURE_IN_PORT                 11
    #define DIP_BIT1                         7              // DIP Switch bits directly wired to digital port
    #define DIP_BIT2                        A1              // Notice there is no DIP0, that is used to control power to the Bluetooth module.
    #define DIP_BIT3                        A0
    #define DIP_BIT4                         4
    #define DIP_BIT5                         5
    #define DIP_BIT6                        12
    #define DIP_BIT7                        13
    #define DIP_MASK                        0xFE            // There are 7-bits usable in this version, lsb is not used.                                        


    #define NTC_A_PORT                      A2              // Alternator NTC port
    #define NTC_B_PORT                      A3              // Battery NTC port
 
    #define STATOR_IRQ_NUMBER                0              // Stator IRQ is attached to pin-4 on the Atmel CPU (INT-0 /port 2 on the Arduino)

    #define   NTC_AVERAGING                 15              // There are up to 3 NTC sensors, and we will average 5 A/D samples each before converting to a temperature, to smooth things over. (Max 254)
                                                            // Note that this, combined with SENSOR_SAMPLE_RATE will define how often the temperatures get updated...

    #define SYSTEM_BAUD                   9600UL            // Nice and slow Baud, to assure reliable comms with local RN-41 Bluetooth module + future expansion. 
    #define DEFAULT_BT_CONFIG_CHANGED      false            // On the 2nd generation regulator, with support for a Bluetooth module, we need to lockout any changes
                                                            // via ASCII commands into the user has revised the name and password.  See users guide for details.
    
    #define FIELD_PWM_MAX                  0xFF             // Maximum alternator PWM value.  The original Alt Regulator had a separate Charge Pump,
                                                            // and could run true Full Field all day long.

    #define AALT_SCALER_48V        ((((2*4990.0)+221.0) / 221.0) / 50.0)  // Ratio of R19..R22 combined with U9 (INA282) 50x gain followed by resistor dividers.
    #define VALT_SCALER_48V             ((30+19.529)/19.529)      // Scaling values of Voltage Divider resisters on the INA-226 VBat+ sensing line. (R1/R6)
                                                           // (Note that I add in the 'typical' Zin  of 830K of the INA-226 into the 'R10' calculation)
                                                            // (Note also this ratio is 'upside down' from typical - I did this so I could use a * instead of a / in the calc..)
 
    #define MASTER_RESTORE_DIP_CHECK    0b11111000         //  ONLY do Master Restore if DIP switches call for CPE#5  to be selected and all the rest of the switches are ON, BT Off.
    
    #define  set_PWM_frequency() TCCR1B = (TCCR1B & 0b11111000) | 0x04;                     // Set Timer 1 (Pin 9/10 PWM) to 122Hz (from default 488hz).  This more matches
                                                                                            // optimal Alternator Field requirements, as frequencies above 400Hz seem to send

    #define SMALL_FLASH                                     // atMega328 only has 1K flash, need to be carefull with not to overflow it.


  
 
#elif defined(__AVR_ATmega32C1__) || \
      defined(__AVR_ATmega64C1__) || \
      defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64M1__)                           //----  New CAN Enabled regulator hardware.

    #define  SYSTEMCAN                                      // Build flag to select different code for the CAN enabled / 'systems' version.
    #define  CPU_AVRCAN                                     // Need to use software simulation for I2C ports  (sensors.cpp)
    #include "Portability.h"                                // Pick up all the IDE specifics, ala PROGMEM
          
                                                
    #define OPTIBOOT                                            // Compile this code for use with Optiboot bootloader?  If so, fully enable the Watchdog timer.
                                                                // If Optiboot is not used, and the 'default' Arduino IDE bootloader is used, there are some workarounds
                                                                // needed to make reboot() function, and if the watchdog is ever triggered the regulator will hang vs.
                                                                // restart.  This is a fault / bug in the basic Arduino bootloader for 3.3v + atMega328.  See blog for more details.

                                                       
    #define HARDWARE_VERSION   "1.3.1 (2017-01-30)"         // Min PCB Hardware version usable with this version of firmware                                                         
    #define PRODUCT_CODE               100                  //  100 is the product code for ATmega64M1 based Smart Alternator Regulator
    
    
    #define FEATURE_OUT_PORT                 8              // Drives Feature-out open-collector port.
    #define LED_GREEN_PORT                  14              // Connected to status LED on the PCB.
    #define LED_RED_PORT                    13 
    #define FIELD_PWM_PORT                   2              // Field PWM 
    #define FEATURE_IN_PORT                  9
    #define DIP_BIT0                        A6              // DIP Switch bits directly wired to ports
    #define DIP_BIT1                        A8 
    #define DIP_BIT2                        11
    #define DIP_BIT3                        A4
    #define DIP_BIT4                        A3  
    #define DIP_BIT5                        A5 
    #define DIP_BIT6                        A2 
    #define DIP_BIT7                         7
    #define DIP_MASK                        0xFF            // There are 8-bits usable in this version                                        
    #define STUFF_SENSE_PORT                10              // Tells which stuffing option (12..48v or 12..24v) we have.


    #define NTC_A_PORT                      A0              // A port, primarily Alternator temperature sensor
    #define NTC_B_PORT                      A1              // B Port, primary Battery temperature sensor - reverts of 2nd Alternator sensor if battery temperature is provided by external source.
    #define NTC_FET_PORT                    A7              // Onboard FET temperature sensor (Definition is also used to enable FET NTC code in sensors.cpp)
    #define STATOR_IRQ_NUMBER                3              // Stator IRQ is attached to pin-30 on the Atmel CPU (INT-3 /port 12 on the Arduino)

    #define   NTC_AVERAGING            15                   // There are up to 3 NTC sensors, and we will average 5 A/D samples each before converting to a temperature, to smooth things over. (Max 254)
                                                            // Note that this, combined with SENSOR_SAMPLE_RATE will define how often the temperatures get updated...

                                                         
                                                            
        //-----  I2C ports and config values  
        //       The soft I2C lib being used is an ATMEL lib, not Arduino.   So, the port definitions are different...
        //       It is Assembler based and very well done - however, it is not fully 'Arduino' compatible.
        //       Two differences are:  It uses Atmel PORT names/number vs. Arduino 'port' number, and
        //                              those values need to be defined BEFORE including the software lib.   So:
    #define SDA_PORT PORTB                                  // This is Arduino port D5 on the ATmegaxxM1
    #define SDA_PIN 0
    #define SCL_PORT PORTB                                  // This is Arduino port D6 on the ATmegaxxM1
    #define SCL_PIN 1 
    
    //#define I2C_FASTMODE 0                                // Set = 1 to run in fast mode (400 kHz)
    //#define I2C_SLOWMODE 1                                // Set = 1 to run in slow mode (25Khz)   -- Default is normal mode (100Khz)
    #define I2C_TIMEOUT 20                                  // timeout after 20 msec -- do not wait for clock stretching longer than this time
    //#define I2C_NOINTERRUPT 0                             // Set = 1 to disable interrupts during time between START() and STOP() 
    //#define I2C_CPUFREQ (F_CPU/8)                         // slow down CPU frequency - Useful if you plan on doing any clock switching

    #define SYSTEM_BAUD                   115200UL          // With no Bluetooth module, run the serial port faster to min time blocked from full buffer.
    #define DEFAULT_BT_CONFIG_CHANGED      true             // On the regulators w/o a Bluetooth module, we do not need to take the extra security caution of locking out any changes
                                                            // via ASCII commands into the user has revised the name and password.  See users guide for details.

                                                            
    #define FIELD_PWM_MAX                  0xFC             // Maximum alternator PWM value - As the newer version of the regulator does not have a charge-pump,
                                                            // we need to restrict Full Field to some PWM, to allow refreshing of the boost-cap in the FET driver chip


    #define AALT_SCALER_48V        ((((2*4990.0)+221.0) / 221.0) / 50.0)  // Ratio of R19..R22 combined with U9 (INA282) 50x gain followed by resistor dividers.
    #define VALT_SCALER_48V             ((30+19.529)/19.529)      // Scaling values of Voltage Divider resisters on the INA-226 VBat+ sensing line. (R1/R6)
    #define VALT_SCALER_24v                  1.0            // (Note that I included the 'typical' Zin of the INA-226 (830K) into the 'R6' calculation)
    #define AALT_SCALER_24v                  1.0            // (Note also this ratio is 'upside down' from typical - I did this so I could use a * instead of a / in the calc..)
                         

    #define MASTER_RESTORE_DIP_CHECK    0b11110011          //  ONLY do Master Restore if DIP switches call for CPE#5 to be selected and all the rest of the switches are ON.
      


    
    #define  set_PWM_frequency() TCCR1B = (TCCR1B & 0b11111000) | 0x04;                     // Set Timer 1 (Pin 9/10 PWM) to 122Hz (from default 488hz).  This more matches
                                                                                            // optimal Alternator Field requirements, as frequencies above 400Hz seem to send


 
                                                                            
#elif defined(STM32F072xB) || \
      defined(STM32F078xx) 
      
          
    #define SYSTEMCAN                                       // Build flag to select different code for this Stand-along regulator.
    #define CPU_STM32                                       // Utilize the STM I2C hardware (sensors.cpp)
    #include "Portability.h"                                // Pick up all the IDE specifics
 

    #define HARDWARE_VERSION   "1.0.0 ((2018-07-29)"         // Min PCB Hardware version usable with this version of firmware                                                         
    #define PRODUCT_CODE               200                  //  200 is the product code for STM32F072 based Smart Alternator Regulator
 
 
     


    //  -- Note:  All the 'Port assignment' stuff is handled in cubeMX
           
    #define   ADC_OVERSAMPLE            4                   // Oversample each ADC 4x times. (MAX 4x!)
    #define   ADC_CHANNELS             19                   // There are 19 ADC sensors to be sampled    

      
    //---- Board specific port mapping
    #define  Field_PWM_Timer    htim1                       //  TIM1 for its PWM source
    #define  Field_PWM_Channel  TIM_CHANNEL_1
    #define  micros_timer       htim2                       //  TIM2 as micros() timer (for Stator IRQ/RPMs)

                

    #define EEPROM_I2C_ADDR             0x50                // EEPROM address




      
    #define SYSTEM_BAUD                   115200UL          // With no Bluetooth module, run the serial port faster to min time blocked from full buffer.
    #define DEFAULT_BT_CONFIG_CHANGED      true             // On the regulators w/o a Bluetooth module, we do not need to take the extra security caution of locking out any changes
                                                            // via ASCII commands into the user has revised the name and password.  See users guide for details.



    #define FIELD_PWM_MAX                  0xFE             // Maximum alternator PWM value.  
    #define DIP_MASK                       0xFF             // There are 8-bits usable in this version                                        
 
    #define AALT_SCALER            ((10000+220) / 220) / 50 // Ratio of R22/R24 combined with U6 (INA282) 50x gain followed by resistor dividers.
    #define VALT_SCALER                 (10+9.881)/9.881    // Scaling values of Voltage Divider resisters on the INA-226 VBat+ sensing line. (R4/R10)
                                                            // (Note that I add in the 'typical' Zin  of 830K of the INA-226 into the 'R10' calculation)
                                                            // (Note also this ratio is 'upside down' from typical - I did this so I could use a * instead of a / in the calc..)




    #define MASTER_RESTORE_DIP_CHECK    0b11110011          //  ONLY do Master Restore if DIP switches call for CPE#5 to be selected and all the rest of the switches are ON.



                           

    //----  Extern var delcerations      
                                                                       
    extern ADC_HandleTypeDef hadc;                              // (from cubeMX)
    extern DMA_HandleTypeDef hdma_adc;
    extern CAN_HandleTypeDef hcan;
    extern I2C_HandleTypeDef hi2c1;
    // extern SPI_HandleTypeDef hspi1;
    extern TIM_HandleTypeDef htim1;
    extern TIM_HandleTypeDef htim2;

    // extern UART_HandleTypeDef huart4;
    //!! HEY!! extern IWDG_HandleTypeDef hiwdg;

    //!! HEY!!  is this being used for the Alt Reg??  extern uint32_t totalHours;


    

#else
        #error Unsupported Smart Regulator CPU 
#endif 








                                                                
                                                                
//
//
//------------------------------------------------------------------------------------
//
//      The Following are internal parameters and typically will not need to be changed.
//
//




                                // -----   Critical fault values: exceeding these causes system fault and fault handler.
                                //         Note that these values are set for a 'normalized - 12v / 500Ah battery', and will be automatically scaled during runtime
                                //         by the regulator after sampling the battery voltage and setting the variable "systemVoltMult"
                                //
#define FAULT_BAT_VOLTS_CHARGE          16.5                    // This is where we will fault.  Make sure you have sufficient headroom for Over Charge voltage (if being used)
#define FAULT_BAT_VOLTS_EQUALIZE        18.0                    // When doing Equalization, allow a higher limit.
#define FAULT_BAT_VOLTS_LOW              8.0                    // Anything below this means battery is damaged, or sensing wire is missing / failing.
#define FAULT_BAT_TEMP                    60                    //  At or above this, fault out. (Approx 140F)
#define FAULT_ALT_TEMP                   1.1                    //  Fault if Alt Temp exceeds desired set point by 10%   
#define FAULT_FET_TEMP                    70                    // If Field driver FETs are over 80c (Approx 160f), something is wrong with the FETs - fault.

#define ADPT_ACPT_TIME_FACTOR              5                    // If the regulators is operating in Adaptive Acceptance Duration mode (either because EXIT_ACPT_AMPS was set = -1, or
                                                                // if we are unable to measure any amps), the amount of time we spend in Bulk phase will be multiplied by this factor, and
                                                                // we will remain in Acceptance phase no longer then this, or EXIT_ACPT_DURATION - whichever is less.  This is in reality a backup
                                                                // to Amp based decisions, in case the shunt is not installed, or fails.  Or in the case where the install never uses the Amp shunt, and
                                                                // charging is started with a full battery - do not want to boil off the battery.
        


                                // ------ Parameters used to calibrate RPMs as measured from Alternator Stator Pulses
#define RPM_IRQ_AVERAGING_FACTOR           100                  // # sector pulses we count between RPM calculations, to smooth things out.  100 should give 3-6 updates / second as speed.
#define IRQ_TIMEOUT                         10                  // If we do not see pulses every 10mS on average, figure things have stopped.
#define IDLE_SETTLE_PERIOD               10000UL                // While looking for a potential new low for idleRPMs, the engine must maintain this new 'idle' period for at least 10 seconds.





    

                                //----  PWM Field Control values
#define FIELD_PWM_MIN              0x00                 // Minimum alternator PWM value - It is unlikely you will need to change this.
// #define FIELD_PWM_MAX           -------              // Maximum alternator PWM value - (This is now defined in the CPU specific section of xxxx.h, SmartRegulator.h, SmartGen.h, etc...)
#define MAX_TACH_PWM                 75                 // Do not allow MIN PWM (held in 'thresholdPWMvalue') to go above this value when Tach Mode is enabled.  Safety, esp in case Auto tech mode is enabled.
                                                        //  This same value is used to limit the highest value the user is allowed to enter using the $SCT ASCII command.
#define PWM_CHANGE_CAP                2                 // Limits how much UP we will change the Field PWM in each time 'adjusting' it.

#define PWM_CHANGE_RATE            100UL                // Time (in mS) between the 'adjustments' of the PWM.  Allows a settling period before making another move.
#define PWM_RAMP_RATE              400UL                // When ramping, slow the rate of change down  to update only this often (in mS)  
                                                        //    This combined with PWM_CHANGE_CAP will define the ramping time.
                                                        //    (for PWM to reach the FIELD_PWM_MAX value and exit Ramp).

   
                                                        // These are used to count down how many PWM_CHANGE_RATE cycles must pass before we look at the 
                                                        // these slow changing temperature values.  (TA = Alt)
                                                        // If any of these are over value, we will adjust DOWN the PWM once every xx times through adjusting PWM. 



 
                                //---- The following scaling values are used in the PID calculations used in manage_ALT()
                                //     Factors are adjusted in manage_alt() by systemMult as needed.
                                //     Values are represented in 'Gain' format, and MUST be defined as floating values (e.g.  30.0  vs. 30) in order to keep each component of the 
                                //      PID engine working with fractions (each small fractional value is significant when summing up the PID components)
#define KpPWM_V                     20.0
#define KiPWM_V                     10.0
#define KdPWM_V                     75.0

#define KpPWM_A                      0.6              
#define KiPWM_A                      0.3 
#define KdPWM_A                      0.7

#define KpPWM_W                      0.05     
#define KiPWM_W                      0.0                
#define KdPWM_W                      0.02


#define KpPWM_AT                      2                 // (These are OK as int consts, as the AT error calcs are done using ints)
#define KdPWM_AT                     20                 // D for temps are only applied to pullback as we approch target. 
#define TAM_SENSITIVITY             100                 // Pace the updating of the dT to every 100x PWM_CHANGE_RATE (or ~every 10 second)  (Max 126)
                                                        // Note that the D factor IS applied each manage_alt() loop, it is only updated ever 10x - 
                                                        // Which also means once updated it will continue to be applied for all the  loops until the next update.

#define PID_I_WINDUP_CAP            0.9                 // Capping value for the 'I' factor in the PID engines.  I is not allowed to influence the PWM any more then this limit 
                                                        // to prevent 'integrator Runaway' .  

                                                          
                                //---- Load Dump / Raw Overvoltage - over temp -  detection thresholds and actions.
                                //     (These action occur asynchronous to the PID engine -- handled in real time linked to the ADCs sampling rate.)
                                                     
#define LD1_THRESHOLD              0.040                // During a Load Dump situation, VBat can start to rise VERY QUICKLY.   Too quick for the PID engine
#define LD2_THRESHOLD              0.080                // in the case of large (200A+) alternators.  If measured voltage exceeds target voltage by these thresholds 
#define LD3_THRESHOLD              0.100                // the PWM is pulled back.   Once these brute force changes are made, the normal PID engine can start 
                                                        // adjusting things back to the new situation.  PROTECT THE BATTERY IS #1 CRITERIA!!!!
        
#define LD1_PULLBACK               0.95                 // On 1st sign of overvoltage, pull back the field a little.
#define LD2_PULLBACK               0.85                 // On 2nd sign of overvoltage, pull back harder 
                                                        // LD 3 overvoltage holds the Field at 0 until the overvoltage situation clears.

                            

#define AOT_PULLBACK_THRESHOLD          1.03            // If we find we are 3% over alternator temperture goal, the PID engine is not keeping up.  
#define AOT_PULLBACK_RESUME             0.90            //  (And do not resume normal operation until we are at 90% of goal temperature or less)
#define AOT_PULLBACK_FACTOR             0.50            // Do something dramatic, cut field drive by 50% before it continues to rise and we fault out.                                                                         


                                                        // Any time we go over-temp, it seems we have a condition
                                                        // of a fight between that temp value and the amp/watts target - creating an osculation situation.
#define OT_PULLBACK_FACTOR              0.95            // When triggered, we will pull down the Watts target and max PWM limit this ratio to try and self correct.
#define OT_PULLBACK_FACTOR_LIMIT        0.60            // Don't pull back more then this, if we are unable to correct with this much pullback - let the system fault out
                                                        // as install is very very wrong.


#define USE_AMPS_THRESHOLD              5               // If at some time we do not measure AT LEAST this amount of Amps, then we will ASSUME the Amp Shunt is not
                                                        // connected - this will cause Manage_ALT() to ignore any Amps based thresholds when deciding if it is time to
                                                        // transition out of a given charging phase.  It is a way for the regulator to act in Voltage Only mode - just
                                                        // do not connect up the shunt!  (Actually, would be better to put a wire jumper across the Shunt terminals)
                                                                                                      

                                                        // When deciding to change Alternator charge states, and adjust the throttle, we use persistent Watts and Amps.
                                                        // These are averaged values over X periods.  These are used to smooth changes in 
                                                        // Alternator State modes - to allow for short term bumps and dips.
#define AMPS_PERSISTENCE_FACTOR          256 //!! HEY!!  Try smaller number 512            // Amps will be averaged over this number of samples at "PWM_CHANGE_RATE". (512 = a bit less then 1 minute look-back)
                                                        // Set = 1L to disable  (Used to exit Acceptance and Float modes)
#define VOLTS_PERSISTENCE_FACTOR         300            // Volts will be averaged over this number of samples at "PWM_CHANGE_RATE". (300 = ~1/2 min look-back)
                                                        // Set = 1 to disable  (Used to exit post_float mode)

                                                        // Desensitizing parameters for deciding when to initiate a new Alternator Capacity Measurement cycle, 
#define SAMPLE_ALT_CAP_RPM_THRESH         250           // Initiate a new Capacity Sample if we have seen in increase in RPMs / Amps from the prior reading.  
#define SAMPLE_ALT_CAP_AMPS_THRESH_RATIO  1.05          // To keep from being too twitchy ..  





                                // --------  INA226 Registers & configuration
                                
  
#define INA226_VP_A_I2C_ADDR       0x40                 // I2C addresses of INA226 device. Measures Amp shunt, and Vbat+   
#define CONFIG_REG                 0x00                 // Register Pointers in the INA-226
#define SHUNT_V_REG                0x01
#define VOLTAGE_REG                0x02 
//#define POWER_REG                0x03                 // Because I am doing raw voltage reads of the Shunt, no need to access any of the INA-226 calculated Amps/power
//#define SHUNT_A_REG              0x04
//#define CAL_REG                  0x05                 // Nor the Cal reg 
#define STATUS_REG                 0x06
#define LIMIT_REG                  0x07
#define DIE_ID_REG                 0xFF                 // Unique 16-bit ID for the part.

#define INA226_CONFIG            0x4523                 // Configuration: Average 16 samples of 1.1mS A/Ds (17mS conversion time), mode=shunt&volt:triggered
#define INA226_PD_CONFIG         (INA226_CONFIG & 0xFFF8)  // Mask out the power-down bits.
#define I2C_TIMEOUTms              100                  // If any given I2C transaction takes more then 100mS, fault out.
#define INA226_TIMEOUTms      	   100                  // If it takes more then 100mS for the INA226 to complete a sample cycle, fault out.
#define INA226_SAMPLE_TIMEOUTms  2*INA_SAMPLE_PERIOD    // If something prevents us from initating a voltas/amps (ina226) sample cycle, fault out.
  




                                // ------ Values use to read the Feature-in port to handle debouncing.
#define DEBOUNCE_COUNT               5                  // We will do 5 samples of the Feature_in port to handling any de-bouncing.
#define DEBOUNCE_TIME                1                  // And if asked for, we will block the system for 1mS between each of those samples in order to complete a
                                                        // debounce cycle when feature_IN() is called.  Do not make this too large, as it is a raw delay.
                                                        // And remember, there is an external R/C to help filter any bouncing before we even see it.



                                // ------ Values use for the NTC temp sensors.
#define   NTC_RO                10000                   // Nominal resistance of NTC sensors at 25c
#define   NTC_BETA               3950                   // Most common Beta for probes on EBay..
#define   NTC_BETA_FETs          3380                   // Beta of NTC chip used for FET sensing.
#define   NTC_RF                10000                   // Value of 'Feed' resister
#define   NTC_RG                  100                   // Value of 'Ground Isolation' resister - used only on the external NTC probes.



                        
                                // ----- Mainloop timing values, how often do we update the PWM, check for key pressed, etc.
                                //         All times are in mS
#define SENSOR_SAMPLE_RATE           50UL               // If we are not able to synchronize with the stator, force a sample of Volts, Amps, Temperatures, every 50mS min. 
#define ACCUMULATE_SAMPLING_RATE   1000UL               // Update the accumulated AHs and WHs  every 1 second.  
#define SAMPLE_ALT_CAP_DURATION   10000UL               // When we have decided it is time to sample the Alternators capability, run it hard for 10 seconds.  
#define SAMPLE_ALT_CAP_REST       30000UL               // and give a 30 second minute rest period between Sampling Cycles. 
//#define OA_HOLD_DURATION          60000UL               // Hold on to external offset amps (received via $EOA command) for only 60 seconds MAX.
                                                          // REDACTED 2-26-2018 

//----  RTOS controls and flags  (If RTOS is being used)
#define NTC_SAMPLE_PERIOD                 500                  // Sample the NTC sensors every 0.5 seconds
#define INA_SAMPLE_PERIOD                  50                  // Sample the INA226 (Volts/AMps) sensors every 50mS





                                // --------  Global Timouts
#define WDT_PER                 WDTO_8S                 // Set the Watchdog Timer for 8 seconds (Lock step this with CHECK_WDT_mS below)
#define CHECK_WDT_mS               8000                 // Used by validation in pre-compile checks for errors in critical timing.  Make this the # mS the WDT is set for.
#define I2C_TIMEOUT                  20                 // Bail on faulted I2C functions after 20mS

 


                                //-----  Bluetooth values
#define BT_TIMEOUT                1000UL                // Wait up to 1 second for the RN41 module to acknowledge a command.

#define CLEAR_BT_LOCK_AMPS            5                 // In order for systemConfig.BT_CONFIG_CHANGED to be changed to TRUE, we must have received a valid '$SCN:' while the
                                                        // alternator is not actually charging.  This defines the Amps threshold we need to be under to decide we are 'not charging'.
                                                        // Note that you can PERMANENTLY disable the ability to change Bluetooth configuration by setting this to -1




#define RVC_CHARGER_TYPE     RVCDCct_Alternator         // This OSEnergy device is an Alternator





//---  Functions and global vars exported from SmartRegulator.ino
uint8_t readDipSwitch(void);
int     checkStampStack(void);
void    reboot(void);

extern uint8_t  requiredSensorsFlag;


#endif  // _SMARTREG_H_



