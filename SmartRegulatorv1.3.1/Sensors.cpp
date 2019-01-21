//      Sensors.cpp
//
//      Copyright (c) 2016, 2017 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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
#include "Sensors.h"
#include "Alternator.h"
#include "OSEnergy_CAN.h"
#include "Flash.h"


    //-------   CPU specific includes - You may have to manually comment these out with over version of the Arduino IDE
#if defined CPU_AVR 
    #include <I2Cx.h>                                       // Newer I2C lib with improved reliability and error checking.
    
    
#elif defined CPU_AVRCAN         
    #include <SoftI2CMaster.h>                              // http://homepage.hispeed.ch/peterfleury/avr-software.html  
                                                            //   It is too bad there are so many of these with the same name.  Be sure to get the correct one
                                                            
                                                            
#elif defined CPU_STM32          
    ///!   NEED TO FIGURE THIS ONE OUT...   LOOK TO THE MBED LIB??? #include <????> 
    
    
#else
    #error  NO SENSOR INCLUDE SELECTED
#endif
    




   
//----  Public  veriables 


float   measuredAltVolts  = 0;                                          // The regulator takes local measurements, and to start with we ASSUME these sensors are connected to the Alternator.  So, we locally measure
float   measuredAltAmps   = 0;                                          // ALTERNATOR voltage and current.
float   measuredBatVolts  = 0;                                          // However, we need to regulator based on Battery voltage and current.  In stand-along installations, we have to assume that what we measure via 
float   measuredBatAmps   = 0;                                          // the regulator is what we are to 'regulator' to, so the cost will copy over the 'alternator' values into the 'battery' values.
                                                                        // However, in more advanced cases we might receive the actual battery voltage and/or current externally.  Either via the CAN bus of by the $EBA: ASCII command
                                                                        // In those cases the measuredBatteryVolts/amps will utilizes the externally supplied values.
 
bool    shuntAmpsMeasured = false;                                      //  accommodating multi charge sources w/external coordination where the Amp Shunt is installed on the Alternator, not the Battery.
                                                                        //  usingEXTAmps is a signal that we are using externally supplied amps.  It is used to help assess when this external value times out.
                                                                        //  shuntAmpsMeasured is a signal that we have ever been able to measure Amps via the shunt - used to disable amp based decisions in 
                                                                        //  manageAlt() when we are not even able to measure Amps....
int     measuredAltWatts  = 0;


int     measuredFETTemp   = -99;                                        // -99 indicated not present.  Temperature of Field FETs, in degrees C.
int     measuredAltTemp   = -99;                                        // -99 indicated not present.  -100 indicates user has shorted the Alt probe and we should run in half-power mode.
int     measuredAlt2Temp  = -99;                                        // -99 indicated not present.  Value derived from 2nd NTC port if battery temperature is delivered by an external source via the CAN.
int     measuredBatTemp   = -99;                                        // -99 indicates we have not measured this yet, or the sender has failed and we need to use Defaults.
                                                                        // Battery Temperature typically will be measured by the 2nd NTC, the B-NTC port.  However, if the temperature is provided by an external source
                                                                        //  (Specifically, via the CAN bus), then the 2nd NTC port will be considered a 2nd alternator sensor for alternator temperature regulation.
bool    batTempExternal   = false;                                      // Have we received the battery temperature from an external source via the CAN?
                                                                        // Defaults to false for compatibility with non-CAN enabled regulators.

int     measuredFieldAmps  = -99;                                       // What is the current being delivered to the field?  -99 indicated we are not able to measure it.

bool    updatingVAs       = false;                                      // Are we in the process of updating the Volts and Amps?  (Meaning, hold off doing anything critical until we get new data..)


#ifdef SIMULATION
  float   simCAN_Ext_amps = 0.0;                                        // Used to receive in externally simulated amps being pushed into the battery.  Ala, in a dual 'engine' simulation.
  #endif


#ifdef STUFF_SENSE_PORT
  float   VAltScaler = VALT_SCALER_48V;                                // If this hardware has enabled a stuffing option runtime selection, we redefine the constant scalers as variables
  float   AAltScaler = AALT_SCALER_48V;                                //  so they cna be adjusted at startup.
  #endif


//----- Calibration buffer 
//      Future releases may allow for calibration of individual boards.  Either by user, or by some external manufacturing process.
//      Would allow for use of lower-cost resistors in all the dividers.  This structure is saved in the FLASH of each device.
//      For now, will simple prime these with default values.
#ifdef CPU_AVR
  tCAL  ADCCal = {false,                                                 // locked   == Let user do calibration if they wish to.
                  1.0,                                                   // .VBAT_GAIN_ERROR  == Error of VBat ADC + resister dividers
                  1.0,                                                   // .AMP_GAIN_ERROR
                  0,                                                     // .VBAT_OFFSET
                  -522};                                                 // .AMP_OFFSET == ADC Offset error of shunt circuit measured @ 0A

                                    //****************************************************************************************************************************
                                    //i += 522;                                 // VERY BAD!  Original PCB needed to add in a manual offset to accommodate a hardware design error with regard to 
                                    // Default offset is now contained          // how common-mode noise is divided by R22/R24, and causes -14.2A to be displayed when no current
                                    // in this  'dafault' ADCCal structure      // is present in amp shunt.
                                                                                // IF YOU DO BUILD OPTION OF NOT INSTALLING INA282 LEVEL SHIFTER, CHANGE DEFAULT TO = 0
                                    //****************************************************************************************************************************




#else
  tCAL  ADCCal = {false,                                                  // locked   == Let user do calibration if they wish to.
                  1.0,                                                    // .VBAT_GAIN_ERROR  == Error of VBat ADC + resister dividers
                  1.0,                                                    // .AMP_GAIN_ERROR
                  0,                                                      // .VBAT_OFFSET
                  0};                                                     // .AMP_OFFSET == ADC Offset error of shunt circuit measured @ 0A
#endif


     
                
                
 

 
//----  Internal veriables and prototypes
uint32_t sensosrLastSampled;                                        // Used in the main loop to force an sensor (INA226, NTC) sample cycle if alternator isn't running.

uint32_t accumulatedNTC_A      = 0;                                // Accumulated RAW A/D readings from sample_ADCs();  Used to calculate temperatures after averaging NTC_AVERAGING A/D readings.
uint32_t accumulatedNTC_B      = 0;
uint32_t accumulatedNTC_FET    = 0;
uint8_t       accumulatedADCSamples = 0;                                // How many NTC A/D samples have been accumulated?  Not that in actuality, we ping-pong between the two NTC sensors each 'sample' cycle, to
                                                                        // allow for the Atmel A/D to settle after selecting a given A/D port.

uint32_t accumulateUpdated;                                        // Time the Last Run Accumulators were last updated.
uint32_t generatorLrStarted;                                       // At what time (mills) did the Generator start producing power?
uint32_t generatorLrRunTime;                                       // Accumulated time for the last Alternator run (in mills)
int32_t accumulatedASecs;                                                  // Accumulated Amp-Seconds of current charge cycle.   This actually holds Amps @ ACCUMULATED_SAMPLING rate.  Need to divide to get true value.
int32_t accumulatedWSecs;                                                  // Accumulated Watt-Seconds of current charge cycle.  This actually holds Watts @ ACCUMULATED_SAMPLING rate. Need to divide to get true value.
          

int16_t savedShuntRawADC;                                               // Place holder for the last raw Shunt ADC reading during read_INA().  Used by calibrate_ADCs() to determine offset error of board
          
int  normalizeNTCAverage(uint32_t accumalatedSample, int beta, bool hasRG);
int  read_INA226(void);
void sample_ADCs(void);
void resolve_ADCs(void);
void calibrate_ADCs(void);

 
//------------------------------------------------------------------------------------------------------
//
//  Initalize Sensor   
//
//      This function will configure the board for locally attached sensor readings.
//      It will also iniitate a sampling cycle of the local ADCs - for later polling and reading.
//
//
//------------------------------------------------------------------------------------------------------
bool initialize_sensors(void) {


  pinMode(NTC_A_PORT,           INPUT);                                                 // And A/D input pins
  pinMode(NTC_B_PORT,           INPUT); 
  
  
  #ifdef NTC_FET_PORT
    pinMode(NTC_FET_PORT,        INPUT);  
    #endif
    
       
  

  #ifdef CPU_AVR                                                                        // Startup the stand-alone regulators Vbat and Amps sensor.
    I2c.begin();                                                                        // new I2C library
    I2c.timeOut(I2C_TIMEOUT);                                                           // Enable fault timeouts on the I2C bus.
    I2c.pullup(0);                                                                      // Disable internal pull-ups, there are external pull-up resisters.
    #endif

    

    
  #ifdef  CPU_AVRCAN                                                                    // ATmega64M1 needs software simulation
    i2c_init();         
    #endif
    
  
  
  #ifdef  CPU_STM32
                        //!!  SOMETHING GOES HERE
    #endif
    
  

  
  #ifdef SIMULATION
    randomSeed(analogRead(0));                                                          // Prime the random number generator (used during VBat simulation)
    #endif


  calibrate_ADCs();                                                                     // See if user has selected self-calibrate mode (by connection Vbat to +5v on the ICSP pin)
   
  sensosrLastSampled     = millis();                                                    // Prime all the loop counters;
  
  reset_run_summary();
  sample_ALT_VoltAmps();                                                                // Let's get these guys doing a round of sampling for use to decide system voltage.
  
  return(true);
}





//------------------------------------------------------------------------------------------------------
// Calibrate ADCs
//
//      Self calibration attempts to calibrate most of the ADCs / DAC to the tolerance of the band-gap reference in the CPU.
//      User can select this mode during startup if they connect VBat to the +5v connector on the ICSP, and also short out 
//      the amperage shunt.  Device will calibrate voltage and amperage offset values and save them to EEPROM.
//      Doing a Master Clear (via Feature-in at power-on, or by using $MSR:  ASCII command).
//      Calibration is based on self-measuring of Vcc using the Vcc/4 ADC channel internal to the CPU.
//
//------------------------------------------------------------------------------------------------------

void     calibrate_ADCs() {

//!! HEY !!!!!
/**********************   UNDER TRIAL, MAY NOT USE THIS APPROCH  
     CAL buff;
     uint32_t readVccMv();                                                         // Forward declaration, for this functions use only

    if (ADCCal.LOCKED == true)  return;                                                 // Device has been factory calibrated, do not let user override it.


    
    // Is the system Valt connected to Vcc on the regulator PCB?  (Simplest is to connect VBat to ICSP-pin2)

    sample_ALT_VoltAmps();                                                              // Start the ADCs going.
    delay (100);                                                                        // It should have only take 17mS for the INA226 to complete a sample, but let's add a bit of padding..                                   
    if (!read_ALT_VoltAmps()) return;                                                   // Any read errors, just bail on calibration.
    if ((measuredAltVolts < 4.50)  || (measuredAltVolts > 5.50))                        // Are we somewhere near 5v? )
        return;                                                                         // No, user is not asking to perform self-contained field cal.

            
   
Serial.println("CALIBRATION SELECTED"); 

      wdt_reset();                                                                      // Don't let the watch-dog bite us.

    #if (defined CPU_AVR) || (defined CPU_AVRCAN)                                                


                                                   // Calibration procedure for ATmega64M1 CPU.
    
        int i; 
        uint32_t VccAcumMv = 0;                                                    // Accumulator for the VCC ADC readings
        int32_t          AMP_OFFSETAccum = 0;
        float         measuredVcc;                                                      // Accumulated Vcc measurement via VBat ADC.
        float         baselineVBAT_GAIN_ERROR;

        
        
        //----  User is asking for simple field calibration.
        //      1) See what Vcc is compared to the internal bandgap
        //      2) Compare that to what we read on VBat
        //      3) Read the Amp Shunt, and note any built-in offset.
        
        baselineVBAT_GAIN_ERROR = ADCCal.VBAT_GAIN_ERROR;                                             // Will need to back-out the existing gain factor.
        measuredVcc = measuredAltVolts / baselineVBAT_GAIN_ERROR;                              // Prime the accumulator

        
        for (i = 0; i < 64; i++) {                                                      // Do 64x oversampling
            sample_ALT_VoltAmps();                                                      // Start the ADCs going.
            VccAcumMv += readVccMv();                                                   // Do CPU based ADC sampling of Vcc.

            delay (100);                                                                // It should have only take 17mS for the INA226 to complete a sample, but let's add a bit of padding..                                   
            if (!read_ALT_VoltAmps()) return;                                           // Any read errors, just bail on calibration.
        
            measuredVcc += (measuredAltVolts / baselineVBAT_GAIN_ERROR);                       // Average here - backing out the existing calibration factor.
            measuredVcc /= 2.0;
            
            Serial.print('.');
            AMP_OFFSETAccum  +=  savedShuntRawADC;                                       // Pick up the value saved during read_ALT_VoltsAmps()
        }

        wdt_reset();                                                                    // Pat-pat..

        
Serial.println((VccAcumMv/64));
Serial.println(((float)analogRead(A10)*4.0*5.0/1023),3);

        baselineVBAT_GAIN_ERROR =  measuredVcc / ((float)(VccAcumMv/64) /1000.0);             // Convert 

        if ((baselineVBAT_GAIN_ERROR < 0.90)  || (baselineVBAT_GAIN_ERROR > 1.10)) return;           // Only allow up to a 10% error, any more and something is wrong...
        
        ADCCal.VBAT_GAIN_ERROR  =  baselineVBAT_GAIN_ERROR;
        ADCCal.AMP_OFFSET = (AMP_OFFSETAccum / 64);
         
 
        //----- Store these new calibration values into EEPROM for future use.
         write_CAL_EEPROM(&ADCCal);


Serial.println("CALIBRATION COMPLETED");
Serial.println(ADCCal.VBAT_GAIN_ERROR,4);
Serial.println(ADCCal.AMP_OFFSET);
  
    #endif 


*********************************/
    
}


//---- Return locally measured Vcc in mV
 uint32_t readVccMv() {

#ifdef CPU_AVR
      // Read 1.1V reference against AVcc
      // set the reference to Vcc and the measurement to the internal 1.1V reference
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
      delay(2);                                                                   // Some time to settle after MUX change
      ADCSRA |= _BV(ADSC);                                                        // Start conversion
      while (bit_is_set(ADCSRA,ADSC));                                            // Wait for conversion to complete
      
      uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
      uint8_t high = ADCH; // unlocks both

      return( 1125300UL / (uint32_t)((high<<8) | low));                      // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

#elif defined(CPU_AVRCAN)
      return (((uint32_t)analogRead(A10)*5*4000)/1023);                      // A10 is internal Vcc/4 ADC port.

#elif defined(CPU_STM32)

        //!! HEY!!  SOMETHING GOES HERE!!!
        
#endif

    return(0);

}







 
//------------------------------------------------------------------------------------------------------
//
//  read_sensors()   
//
//      This function will gather information from all the sensors and update
//      global variables. 
//
//      If there is a fatal error it will return FALSE
//         Currently, Faulting conditions are read errors on the INA226 sensor  (Volts and Amps)
//
//------------------------------------------------------------------------------------------------------

  
bool  read_sensors(void) {
   

        // Start a sample run if the INAs are ready and the stator just signaled,
        // OR too much time has elapsed and it seems
        // either the INA226 has stalled, or we are not receiving Stator interrupts.

    if ((statorIRQflag  && !updatingVAs) ||                                                     // Synchronized with stator
        ((millis() - sensosrLastSampled) >= SENSOR_SAMPLE_RATE)) {                               // Or just go get them it is has been too long. . . .  (e.g., alt stopped)      
  
        sample_ALT_VoltAmps();                                                                  // Start the INA226 sample cycle.
        sample_ADCs();                                                                          // Use the same pacing rate for the local NTC ADC sampling conversions.

        sensosrLastSampled = millis();                           
        statorIRQflag = false;                                                                  // Clear the Stator Flag for next time.
        }

    resolve_ADCs();
    return(read_ALT_VoltAmps());

  }



  
  
  
  
  
  



//------------------------------------------------------------------------------------------------------
// Sample ALT Volts & Amps 
//      This function will instruct the sensors used for local reading of Volts and Amps to begin a sample cycle.
//        Example:  instruct the INA226's to begin a sample cycle (triggered mode).
//        As a result of the Config reg being written to -  the INA-226 will begin a new sample / averaging cycle.
//        (Note, this is also called during Setup to initially program the INA226 with its configuration values)
// 
//------------------------------------------------------------------------------------------------------

bool sample_ALT_VoltAmps(void) {
    
  #ifdef CPU_AVR
    uint8_t ptr[2];

    ptr[0] = highByte(INA226_CONFIG);                                                   // Config current & Vbat INA226
    ptr[1] = lowByte (INA226_CONFIG);
       I2c.write(INA226_VP_A_I2C_ADDR,CONFIG_REG, ptr, 2);                                // Writing the Config reg also 'triggers' a INA226 sample cycle.
    #endif
   
   
   
   
  #ifdef CPU_AVRCAN
    i2c_start(INA226_VP_A_I2C_ADDR<<1 | I2C_WRITE);
    i2c_write(CONFIG_REG);                                                              // Writing the Config reg also 'triggers' a INA226 sample cycle.
    i2c_write(highByte(INA226_CONFIG));                                                 // MSB always goes 1st
    i2c_write( lowByte(INA226_CONFIG));
    i2c_stop();
    #endif
   

  updatingVAs = true;                                                                   // Let the world know we are working on getting a new Volts and Amps reading 
  return(true);
  
}






 
//------------------------------------------------------------------------------------------------------
//
//  read_ALT_VoltAmps()   
//
//      This function will read the Volts and Amps attached to the regulator's sensor ports and update global alternator variables for Volts, Amps, and Watts.
//      After doing this, the function resolve_BAT_voltsAmps() should be called.
//   
//      Much of the simulation code is here, enabled with the SIMULATION flag.  Scroll down to get to the real code!
//
//      If there is a fatal error it will set the appropriate chargingState and faultCode and return FALSE
//      Fatal error meaning we are not able to determine the battery voltage, and as such need to not be running the alternator.
//
//------------------------------------------------------------------------------------------------------
bool read_ALT_VoltAmps(void) {

#ifdef SIMULATION

     #define SIM_BAT_TYPE_FLA                                                   // Select one to simulate
//   #define SIM_BAT_TYPE_FiLeP04                                               // Select one to simulate

     #define SIM_BAT_AHCAP              1000                                    // Capacity in Amp Hours
     #define SIM_ALT_SIZE                275                                    // What is the size of the alternator?


     #ifdef SIM_BAT_TYPE_FLA
        #define SIM_BAT_NOM_IR           0.003                                  // Nominal (at full charge) Internal Resistance.   Try using:  FLA = 0.002 / 0.005
        #define SIM_BAT_FC_V            14.6                                    // 'At Rest' voltage of fully charged battery
        #define SIM_BAT_FD_V            11.4                                    // 'At Rest' voltage of completely discharged battery
        #endif


     #ifdef SIM_BAT_TYPE_FiLeP04
        #define SIM_BAT_NOM_IR           0.0003                                 // Nominal (at full charge) Internal Resistance   Try using:  FLA = 0.0001 / 0.0004
       #endif





     #define  SIM_MAX_RPM               6000.0                                  //   RPM range limits we will use for simulation
     #define  SIM_MIN_RPM               1000.0

     float static simRPM                = (SIM_MAX_RPM - SIM_MIN_RPM)/2;        // Start by assuming the Alternator is running half speed.
     float static simHouseAmps          = 0;                                    // Simulated house load, used to help in simulating 'load dumps'
     float static simSOC                = 0.65;                                 // Current SOC of battery, in % from 0-100  (0-1)
   
     bool static          simCharging   = true;                                 // Are we charging, or discharging?
     uint32_t static simStateLastCh = 0;                                        // And when did this state last change?  (Used to provide lag  effect in battery voltage)
     uint32_t static simLastCalc    = 0;                                        // Time (in mS) this last simulation was preformed.  Used to adjust battery SOC.



        

        measuredAltAmps  = 1.0 + log( ((float)fieldPWMvalue) / 255.0);                          // Start by deducing a simulate exponential curve of alternator output.
        measuredAltAmps *= 1.0 + log(simRPM/SIM_MAX_RPM) / log(100.0);                          // Overlay an exponential curve simulating output vs. RPMs.
        measuredAltAmps *= SIM_ALT_SIZE;                                                        // Now scale the output from 0..1 to 0..Alt Size.
        measuredAltAmps += simCAN_Ext_amps;                                                     // Add in any external Amps we have recevied.  (Note we are NOT doing a time qualifier, so if the device dies - we keep adding)
        measuredAltAmps += (random(1000) / 1000.0) * 0.01 * SIM_ALT_SIZE;                       // Add in 1% of 'randomness' from the alternator's output.
        measuredAltAmps -= 17.9;                                                                // Simulated 'house' load
        measuredAltAmps  = constrain(measuredAltAmps, -17.9, (float)(SIM_ALT_SIZE+simCAN_Ext_amps));      // Make sure we do not go out of bounds (too large, too small) 


        simSOC      += ((measuredAltAmps - simHouseAmps) * (millis() - simLastCalc)) / ((float)SIM_BAT_AHCAP * 360000.0);
                                                                                                // Calc how much new 'energy' is being placed back into the battery represented as a change in SOC.
                                                                                                // Will ignore recharge efficiency (ala, FLA ~ 90%, LeFeP04 ~ 99%) as it does not really impact this sim.
                                                                                                // The formula has been reduced / shifted to prevent overflow issues in the FP calculations...
        simSOC = constrain(simSOC, 0.0, 1.0);


  


        if ((((measuredAltAmps - simHouseAmps) < 0.0) && (simCharging == true))   ||
            (((measuredAltAmps - simHouseAmps) > 0.0) && (simCharging == false))) {             // There has been a change in state from charging to discharging, or the other...
                simCharging = !simCharging;
                simStateLastCh = millis();
                }



        measuredAltVolts =  SIM_BAT_FD_V;                                                       // Start with ideal fully discharged battery voltage.
        measuredAltVolts += ((float)(SIM_BAT_FC_V - SIM_BAT_FD_V)) * simSOC;                    // Add in an uplift of ideal battery based on SOC
        measuredAltVolts += (measuredAltAmps - simHouseAmps) * SIM_BAT_NOM_IR;                  // Now add in overlay of impact of charge/discharge current via Internal Resistance.

         
        #ifdef SYSTEMCAN 
            if ((CAN_RBM_sourceID != 0)  &&                                                     //  If simulating in a CAN system state
                (ignoringRBM == false)   && 
                (CAN_RBM_desiredChargeState != RVCDCbcm_Undefined)  && (CAN_RBM_desiredChargeState != RVCDCbcm_Unknown))                                                                                                         // RBM is controlling us, we need to make some other checks and adjustments.
                    simHouseAmps = 0.0;                                                         //  Do not allow simHouseAmps or battery SOC to impact anything

            else 
            #endif
            if (simCharging)
               measuredAltVolts  -= (SIM_BAT_FC_V - measuredAltVolts) * (     exp(-(float)(millis() - simStateLastCh) / (60000.0 * 1.0)));    // Charging.  Simulate a bit of initial resistance to raise battery voltage over 1 minutes.
            else
               measuredAltVolts  -= (SIM_BAT_FC_V - measuredAltVolts) * (1 - (exp(-(float)(millis() - simStateLastCh) / (60000.0 * 3.0))));   // Discharging.  Simulate the 'surface charge' dissipating over 3 minutes






        measuredAltVolts  = constrain(measuredAltVolts,10.0,15.75);







        if ((measuredAltAmps - simHouseAmps) < 0.0) {                                           // We are discharging.
             if (simCharging  == true) {
                simCharging    = false;                                                         // And looks like just changed to it.
                simStateLastCh = millis();
                }
             }
        else {
            if (simCharging  == false) {
                simCharging    = true;
                simStateLastCh = millis();
                }
            }










                //----  Adding in simulated external events - loads being placed on and removed.
                //      (Will be reflected in the next time through)
                //

        simHouseAmps = 0.0;                                                                     // All external events are reflected by a simulated Amps being drawn
                                                                                                // from the system.  LetÃ¢â‚¬â„¢s start assuming there is no such external load.

       if ((millis() > (323000UL))   &&
           (millis() < (350000UL))) {                                                           // Simulate a heavy load added well into Acceptance phase.
                simHouseAmps = SIM_ALT_SIZE * 0.60;                                             // 60% of alternator size.
                bool static simHL1 = false;
                if (!simHL1) Serial.println("   SIMULATED Heavy Load #1");
                simHL1 = true;
                }


       if ((millis() > (altModeChanged + 50000UL))   &&
           (millis() < (altModeChanged + 52000UL))) {                                           // Simulate a short (2 second) heavy load added while in float.
                simHouseAmps = SIM_ALT_SIZE * 1.2;                                              // 20% greater than the size of the alternator (Will draw from battery).
                bool static simHL2 = false;
                if (!simHL2) Serial.println("   SIMULATED Heavy Load #2");
                simHL2 = true;
                }
       
        if ((millis() > (altModeChanged + 100000UL))   &&
            (millis() < (altModeChanged + 160000UL))) {                                         // Simulate a longer (1 minute) heavy load added while in float.
                simHouseAmps = SIM_ALT_SIZE * 2.0;                                              // 200% greater than the size of the alternator (Will draw from battery).
                 bool static simHL3 = false;
                if (!simHL3) Serial.println("   SIMULATED Heavy Load #3");
                simHL3 = true;
               }
                         
        updatingVAs     = false;
        shuntAmpsMeasured = true;
        simLastCalc       = millis();

        measuredAltWatts   = (int) (measuredAltVolts * measuredAltAmps);    


        if (measuredAltAmps >= USE_AMPS_THRESHOLD)                                                  // Set flag if it looks like we are able to read current via local shunt.
            shuntAmpsMeasured = true;
     

 

              
        return(true);
    #endif        // SIMULATION option


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    unsigned u = read_INA226();                                                                 // Get Bat Volts, Alt Amps, and update global variables.
                                
    if (u != 0) {                                                                         
        chargingState = FAULTED;                                                                // A non-zero return contains the I2C error code.
        faultCode       = FC_INA226_READ_ERROR + u;                                             // Add in I2C returned error code.
        return (false);                                                                         // And loop back to allow fault handler to stop everything!
        }                                       
        
  
    
    //----  Now that we have the basic Volts and Amps measured, see if some level of adjustments need to be done, and also update any related variables (e.g. AltWatts)
    //
    
     
    measuredAltWatts   = (int) (measuredAltVolts * measuredAltAmps);    


    if (measuredAltAmps >= USE_AMPS_THRESHOLD)                                                  // Set flag if it looks like we are able to read current via local shunt.
        shuntAmpsMeasured = true;
    
    return(true);


}

 
 
//------------------------------------------------------------------------------------------------------
//
//  Resolve_BATT_VoltAmpTemp()   
//
//      This function is called to decide how the voltage and amps measured locally via measure_ALT_VoltsAmps() should be used with regards to the actual battery voltage and current.
//      In simple stand-alone systems they will be the same.  However, if battery voltage and/or amps are communicated to the regulator externally (either via the $EBA: ASCII String,
//      of via the CAN bus)  this functions will apply needed adjustments to locally measured values and calculate the expected battery voltage and amps.
//
//      Externally received battery information will be aged, and after some period of time will be ignored.  In this way if a communications error occurs the regulator will fall back
//      to using locally measured values as a default.  Thereby protecting the battery.  Likewise, if we receive external values that are just not believable, they will be ignored and the
//      ignoringRBM flag will be set.
//
//      Likewise, battery temperature will be looked at to see if a remotely measured value should be used.  Specifically from a RMB or a RAT via the CAN bus.
//
//
//------------------------------------------------------------------------------------------------------
void resolve_BAT_VoltAmpTemp(void) {

    
    measuredBatVolts  = measuredAltVolts;                                                     // Start by assuming we have not received any external communications.
                        
    
    //if ((millis()- EORLastReceived) >= OA_HOLD_DURATION)                                      // Need to resolve if an $EBA: string has delivered the battery amps.  If so, has it gone stale? 
    //        usingEXTAmps = false;                                                              // We have held on to the external value long enough, and the user has not updated it..
    // REDACTED 2-26-2018                                                                                          
  
    
    //-------   Now apply any CAN based adjustments
    //
    
    #ifdef SYSTEMCAN  
       usingEXTAmps = resolve_RI_VoltAmpsTemp(&measuredBatVolts, &measuredBatAmps, &measuredBatTemp);       
                                                                                              // Make appropriate adjustments if we are linked to a RBM and it Remote Insturmentation 
                                                                                              //  ANd note if we were able to do so.
                                                                                              //  (Prog note:  Flow was changed with removal of $EBA: support, if reinstated - will have to swizzel this some.
    #endif
    
    
      if (usingEXTAmps != true)                                                               // Did we get an external battery current value from someone? 
        measuredBatAmps = measuredAltAmps;                                                    // No.  Fall back to isolated mode and assume the battery amps are what we are measuring our self.

}










 




//------------------------------------------------------------------------------------------------------
// Read INA-226
//      This function will check to see if the values in the INA-226 is ready to be read.
//      If so, this will read them and update the global variables for measured ALTERNATOR voltage and amps;
//      an external function needs to decide how these should be used with regards to BATTERY voltage and amps.
//      The Global Variable INA226_ready will also be set to TRUE to indicate they are ready for another 
//      sampling cycle to begin.
//
//      Will return '0' if all is OK, else will return the I2C lib error code.
// 
//------------------------------------------------------------------------------------------------------

int read_INA226(void) {

#ifdef CPU_AVR
  int16_t i;

                //--- Do we have anything to read?   Check the VBat+, Alternator AMPs sensor.

  if ((i = I2c.read(INA226_VP_A_I2C_ADDR, STATUS_REG, 2)) != 0)                           // Read in the Status Register.
        return(i);                                                                      // If I2C read error (non zero return), skip the rest.

  i  = I2c.receive() <<8;
  i |= I2c.receive(); 

  if (i & 0x0008) {                                                                     // Conversion is completed!   Go get them! 
    if ((i = I2c.read(INA226_VP_A_I2C_ADDR, VOLTAGE_REG, 2)) != 0)                        // Check Valt
        return(i);                                                                      // If I2C read error (non zero return), return error and skip the rest.

    i  = I2c.receive() <<8;
    i |= I2c.receive(); 
 
    measuredAltVolts  = i * 0.00125 * VAltScaler * ADCCal.VBAT_GAIN_ERROR;                // Each bit = 1.25mV, and adjust for the pre-scaling resisters.


    
    if ((i =I2c.read(INA226_VP_A_I2C_ADDR, SHUNT_V_REG, 2)) != 0)                         // Now read the Amps, read the raw shunt voltage.
           return(i);                                                                   // If I2C read error (non zero return), return error and skip the rest.


    i  = I2c.receive() <<8;
    i |= I2c.receive(); 
 

    //****************************************************************************************************************************
    // i += 522;                                                                        // VERY BAD!  Adding in a manual offset to accommodate a hardware design error with regard to 
    // Default offset is now contained in the 'dafault' ADCCal structure                // how common-mode noise is divided by R22/R24, and causes -14.2A to be displayed when no current
    //                                                                                  // is present in amp shunt.
    //                                                                                  // IF YOU DO BUILD OPTION OF NOT INSTALLING INA282 LEVEL SHIFTER, REMOVE THIS
    //  Above has been moved to the config structure ADCCal.AMP_OFFSET
    //****************************************************************************************************************************

                                                                                        
    measuredAltAmps  = (i - ADCCal.AMP_OFFSET)  * 0.0000025 * AAltScaler * (float)systemConfig.AMP_SHUNT_RATIO;  // Each bit = 2.5uV Shunt Voltage.  Adjust by Shunt ratio and internal dividers (R22/R24)
    if (systemConfig.REVERSED_SHUNT == true)
      measuredAltAmps *= -1.0;                                                          // If shunt is wired backwards, reverse measured value.

    updatingVAs = false;                                                                //   All done, ready to do another synchronized sample session anytime.
    }


return (0);
#endif          // (Standalone)




#ifdef CPU_AVRCAN
  int16_t i;
                //--- Do we have anything to read from the INA226?   Check the VAlt+, AMPs sensor.

   if (!i2c_start(INA226_VP_A_I2C_ADDR<<1 | I2C_WRITE)) return(1);                            // Initiate an I2C read operation - send out which register we want to read.
   if (!i2c_write(STATUS_REG))                        return(2);                            // Send out the register we want to read; will return if we received an ACK from the IAN226 or not. 
   i2c_stop();
         
   if (!i2c_rep_start(INA226_VP_A_I2C_ADDR<<1 | I2C_READ))  return(3);                        // Now tell the I2C we want to read that register
   i  = i2c_read(false) <<8;
   i |= i2c_read(true);    // Get the LSB, and send out the I2C NACK to indicate we are done.
      i2c_stop();


   if (i & 0x0008) {  



// Conversion is completed!   Go get them! 
     if (!i2c_start(INA226_VP_A_I2C_ADDR<<1 | I2C_WRITE)) return(4);                            // Check alt
     if (!i2c_write(VOLTAGE_REG))                       return(5);
     i2c_stop();
 
     if (!i2c_rep_start(INA226_VP_A_I2C_ADDR<<1 | I2C_READ))  return(6); 
     i  = i2c_read(false) <<8;
     i |= i2c_read(true);  
     i2c_stop();

     measuredAltVolts  = i * 0.00125 * VAltScaler * ADCCal.VBAT_GAIN_ERROR;                    // Each bit = 1.25mV, and adjust for the pre-scaling resisters.
 

 
     if (!i2c_start(INA226_VP_A_I2C_ADDR<<1 | I2C_WRITE)) return(7);                           // Now read the Amps, read the raw shunt voltage.
     if (!i2c_write(SHUNT_V_REG))                       return(8);
     i2c_stop();
 
     if (!i2c_rep_start(INA226_VP_A_I2C_ADDR<<1 | I2C_READ))  return(9);   
     i  = i2c_read(false) <<8;
     i |= i2c_read(true);                                                       
     i2c_stop();

     savedShuntRawADC = i;                                                                   // Tuck this raw value away in case we are doing a auto-calibration procedure (See cal_ADCs())
     measuredAltAmps  = (i - ADCCal.AMP_OFFSET)  * 0.0000025 * AAltScaler * (float)systemConfig.AMP_SHUNT_RATIO;  
                                                                                             // Each bit = 2.5uV Shunt Voltage.  Adjust by Shunt ratio and internal dividers 
      if (systemConfig.REVERSED_SHUNT == true)
        measuredAltAmps *= -1.0;                                                            // If shunt is wired backwards, reverse measured value.

     updatingVAs = false;                                                                   //   All done, ready to do another synchronized sample session.
     }

return (0);

  
#endif        // (SYSTEMCAN)


}





//------------------------------------------------------------------------------------------------------
// Sample NTC's
//      This function will sample the NTC's A/D ports and update the accumulated A/D value.
//
// 
//------------------------------------------------------------------------------------------------------

void sample_ADCs(void) {


    switch (accumulatedADCSamples % 3) {                                                        // Which NTC port do we need to do?
        case 0:     accumulatedNTC_A  += (uint32_t) analogRead(NTC_A_PORT);                // Read the 'A' (Alternator) port
                    break;
                    
        case 1:     accumulatedNTC_B  += (uint32_t) analogRead(NTC_B_PORT);                // Read the 'B' ((Battery, or 2nd alternator) port
                    break;
                   
#ifdef NTC_FET_PORT 
        case 2:       accumulatedNTC_FET  += (uint32_t) analogRead(NTC_FET_PORT);          // Read the Field FET's NTC,
                      break;
#endif 
                     

        default:  break;
        }                                                     
              
              
   accumulatedADCSamples++;

       
}








//------------------------------------------------------------------------------------------------------
// Resolve ADCs
//      This function will check to see if it is time to calculate the temps based on the accumulated NTC Samples.
// 
//------------------------------------------------------------------------------------------------------

void resolve_ADCs(void) {

    if ((accumulatedADCSamples  < NTC_AVERAGING) || ((accumulatedADCSamples % 3) != 0))
        return;                                                                               // Not ready to do calculation yet, or counter  in mid-cycle through the NTC ports (messes up average)!

        
    measuredAltTemp  = normalizeNTCAverage(accumulatedNTC_A, NTC_BETA, true);                 // Convert the A NTC sensor for the alternator.
    
    
    
    
    //---   Port B is most often used to sense the battery temperature.  However, if we are receiving battery temp from an external source
    //      (e.g., the CAN bus), then port B can be repurposed as an 2nd alternator sensor.
    
    if (batTempExternal == true)                                                                                    
        measuredAlt2Temp = normalizeNTCAverage(accumulatedNTC_B, NTC_BETA, true);             // We are receiving the Battery temp externally; B port is used for a 2nd alternator probe.
    else {                                                                                     // (the CAN handler function will deal with updating battery temperature)
        measuredBatTemp  = normalizeNTCAverage(accumulatedNTC_B, NTC_BETA, true);             // Not receiving battery temp, so we will treat the B port as connected to the battery.
        measuredAlt2Temp = -99;
        }


    
    #ifdef NTC_FET_PORT
        measuredFETTemp    = normalizeNTCAverage(accumulatedNTC_FET, NTC_BETA_FETs, false);   // And also convert the FET sensor (Onboard FET NTC does not have a Ground Isolation Resistor)
        #endif

    if ((measuredBatTemp  > 120) || (measuredBatTemp  < -40)) measuredBatTemp  = -99;         // Out of bound A/D reading, indicates something is wrong...
    if ((measuredAlt2Temp > 120) || (measuredAlt2Temp < -40)) measuredAlt2Temp = -99;  
    
    if  (measuredAltTemp  > 160)                              measuredAltTemp  = -100;        // If user has shorted out the Alt sensor - 
    else                                                                                      // that indicates they want to run in 1/2 power mode   
      if((measuredAltTemp > 120) || (measuredAltTemp  < -40)) measuredAltTemp  = -99;  

          
    #ifdef NTC_FET_PORT 
      if ((measuredFETTemp > 120) || (measuredFETTemp < -40)) measuredFETTemp = -99;  
      #endif
     
    accumulatedNTC_A = 0;                                                                     // Reset the accumulators and counter
    accumulatedNTC_B  = 0;
    accumulatedNTC_FET = 0;
    accumulatedADCSamples = 0;




#ifdef SIMULATION
    uint32_t static fieldOTtime = 0;
    uint32_t static fieldUTtime = 0;
    float static simAltTemp = 35.0;

    if (fieldPWMvalue > 160) {
        fieldUTtime = 0;
        if (fieldOTtime == 0) 
            fieldOTtime = millis();
        simAltTemp = simAltTemp + (((float)(millis() - (float)fieldOTtime) / 100000.0) * (0.7  + (0.01 * (fieldPWMvalue - 160)))); 
         
    }
    
    if (fieldPWMvalue < 150) {
        fieldOTtime = 0;
        if (fieldUTtime == 0) 
            fieldUTtime = millis();
         simAltTemp = simAltTemp - (((float)(millis() - (float)fieldOTtime) / 100000.0) * (0.7  + (0.01 * (150 - fieldPWMvalue))));             
    }  
      
    if (simAltTemp < 35.0)
        simAltTemp = 35.0;

        
   if (simAltTemp >= ((systemConfig.ALT_TEMP_SETPOINT  * FAULT_ALT_TEMP) - 1))
        simAltTemp = ((systemConfig.ALT_TEMP_SETPOINT  * FAULT_ALT_TEMP) - 1);

    measuredAltTemp  = simAltTemp;
    measuredAlt2Temp = -99;

    measuredBatTemp  = 28; 
#endif


}


int normalizeNTCAverage(uint32_t accumulatedSample, int beta, bool hasRG)    {     // Helper function, will convert the passed oversampled ADC value into a temperature
    float          resistanceNTC;
    int            adcNTC;

    adcNTC = (accumulatedSample / (accumulatedADCSamples/3));                           // Calculate NTC resistance
    if (adcNTC > 1000)                                                                  //    There must not be any NTC probe attached to this port.
       return(-1000);                                                                   //    Signal that by sending back a very very cold temp...
    resistanceNTC = (1023.0 / (float)adcNTC) - 1.0;
    resistanceNTC = (float)NTC_RF / resistanceNTC;

    if (hasRG)  resistanceNTC -= (float)NTC_RG;                                         // Adjust for the Ground Isolation Resistor, if this sensor has one.
                                                                                        // Use Beta method for calculating dec-C.
    return((int)(1 / (log(resistanceNTC / NTC_RO) / beta + 1 / (25.0+273.15) ) - 273.15));
    
 }




 
 
 

//------------------------------------------------------------------------------------------------------
//
//  update_run_summary()   
//
//      This function will update the global accumulate variables Ah and Wh, as well as Run Time.
//      Used to drive Last Run Summary display screen, and also provide values for exiting Float mode via Ahs.
//
//------------------------------------------------------------------------------------------------------
  
void update_run_summary(void) {

    if ((millis() - accumulateUpdated) < ACCUMULATE_SAMPLING_RATE)                              // Update the runtime accumulators?
        return;                                                                                 //  Not yet.  (Do it every 1 second)

    accumulateUpdated = millis();
  


   if ((chargingState >= pending_R) && (chargingState <= equalize)) {                       //  If the Alternator is running, update the last-run vars.
        generatorLrRunTime = millis() - generatorLrStarted;
        accumulatedASecs    += measuredAltAmps;
        accumulatedWSecs    += measuredAltWatts;
        }

}





//------------------------------------------------------------------------------------------------------
//
//  reset_run_summary()   
//
//      This function will reset the global accumulate variables Ah and Wh, as well as Run Time.
//
//------------------------------------------------------------------------------------------------------

void  reset_run_summary(void){                                                                          // Zero out the accumulated AHs counters as we start this new 'charge cycle'
    accumulateUpdated  = millis();
    generatorLrStarted = millis();                                                          // Reset the 'last ran' counters.
    generatorLrRunTime = 0;
    accumulatedASecs   = 0;
    accumulatedWSecs   = 0;
}




