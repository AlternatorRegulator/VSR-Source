//      LED.cpp
//
//      Copyright (c) 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
//                                                      http://smartdcgenerator.blogspot.com/
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


#include "Config.h"                                               // Pick up the specific structures and their sizes for this program.
#include "LED.h"
#include "OSEnergy_CAN.h"




unsigned        LEDPattern;                                             // Holds the requested bit pattern to send to the LED.  Send LSB 1st.
unsigned        LEDBitMask;                                             // Used to select one bit at a time to 'send to the LED'
int8_t          LEDRepeat;                                              // How many times were we requested to show this pattern?
unsigned        LEDTiming;                                              // How quickly should we send out the Bits?
unsigned        LEDBitCntr;                                             // How many bits to we have left in the LED bit pattern?
uint32_t   LEDUpdated;                                             // When was the LED last changes in accordance to the bit-pattern?
bool            LEDFOMirror;                                            // When Blinking Out to the LED, should we also blink out to the Feature-Out port?  (requires #include FEATURE_OUT_LAMP to be defined)


//----  Some UGLY macros to allow for common code between Arduino and cubeMX  (AVR & STM32 CPUs)
#ifdef  CPU_STM32
    #define  _SET_GREEN_LED(x)       HAL_GPIO_WritePin(LED_Green_GPIO_Port,   LED_Green_Pin,   x)
    #define  _SET_RED_LED(x)         HAL_GPIO_WritePin(LED_Red_GPIO_Port,     LED_Red_Pin,     x)
    #define  _SET_FO_PORT(x)         HAL_GPIO_WritePin(Feature_OUT_GPIO_Port, Feature_OUT_Pin, x)  
 
    #define LED_ON      GPIO_PIN_SET
    #define LED_OFF     GPIO_PIN_RESET
    
    
#elif defined(CPU_AVR) || defined(CPU_AVRCAN)
    #define  _SET_GREEN_LED(x)       digitalWrite(LED_GREEN_PORT,  x)
    #define  _SET_RED_LED(x)         digitalWrite(LED_RED_PORT,    x)
    #define  _SET_FO_PORT(x)         digitalWrite(FEATURE_OUT_PORT,    x)

    #define LED_ON      HIGH
    #define LED_OFF     LOW

#else
    #error Unsupported CPU in LED.CPP 
#endif






//------------------------------------------------------------------------------------------------------
//
// Update LED
//
//      This routing is called from Mainloop and will set the LED to the blinking pattern appropriate to the
//      current Alternator Status and flash it.  A trick to force this utility to refresh the blinking pattern is to 
//      set the global variable "LEDRepeat" = 0;   This will terminate the current blinking pattern smoothly and 
//      let this routine reset the pattern to the new one.  (ala, use when changing states in manage_alternator(); )
//
//      If #include FEATURE_OUT_LAMP is defined, this routine will also update the Feature-Out port to FULL ON
//      when the alternator is in a no-charge state.
//
//
//
//
//------------------------------------------------------------------------------------------------------



void update_LED() {

   if (refresh_LED() == true)                                           // If there is already a pattern blinking out, let it finish before overwriting it.
        return;
 


  switch (chargingState) {
        case ramping:
        case post_ramp:
        case determine_ALT_cap:
        case bulk_charge: 
        case RBM_CVCC:                  blink_LED (LED_BULK,       LED_RATE_NORMAL, -1, false);
                                        break;

        case acceptance_charge:         blink_LED (LED_ACCEPT,     LED_RATE_NORMAL,   -1, false);
                                        break;

        case overcharge_charge:         blink_LED (LED_OC,         LED_RATE_SLOW, -1, false);
                                        break;

        case float_charge:
        case forced_float_charge:
        case post_float:                blink_LED (LED_FLOAT,      LED_RATE_NORMAL, -1, false);
                                        break;

        case equalize:                  blink_LED (LED_EQUALIZE,   LED_RATE_FAST,   -1, true);                  // Include the dash LAMP while equalizing
                                        break;


        default:                        blink_LED (LED_IDLE,       LED_RATE_NORMAL, -1, false);
                                        break;
        }


}





//------------------------------------------------------------------------------------------------------
//
// Blink LED
//
//      This routing will initiate a new 'LED' status update blinking cycle.  It will shift out a pattern based
//      on the pattern given to it.   Note that if a LED flashing pattern is already in effect, calling this 
//      will abort that existing one and replace it with this new request.
//      
//      There is a dependency on calling refresh_LED() every once and a while in order to actually update the LED.
//
//              pattern --> Pattern to send to LED, will be shifted out MSB 1st to the LED every timing period
//              timing  --> Number of mS that should pass between each bit-pattern update to the LED
//              repeat  --> Once the pattern has finished, repeat it this many times.  -1 = repeat until stopped.
//              mirror  --> Should we mirror the blinking on the FEATURE_OUT port (requires #include FEATURE_OUT_LAMP to be defined) 
//
//
//
//------------------------------------------------------------------------------------------------------




void blink_LED (unsigned pattern, unsigned led_time, int8_t led_repeat,  bool mirror) {

        LEDPattern   = pattern;
        LEDTiming    = led_time;
        LEDRepeat    = led_repeat;
        LEDFOMirror  = mirror;

        _SET_RED_LED(LED_OFF);                                           // As we are starting a new pattern, make sure the LED is turned off
        _SET_GREEN_LED(LED_OFF);                                         // in preparation for the new bit pattern to be shifted out.

    

        LEDBitMask = 0;                                                 // We will start with the MSB in the pattern
        LEDUpdated = millis();
      
}










//------------------------------------------------------------------------------------------------------
//
// Refresh LED
//
//      This routine will look to see if sufficient time has passed and change the LED to the next bit in the bit pattern.
//      It will repeat the pattern x times as determined by blink_LED() call and return TRUE.
//      If the pattern (and repeats) has been exhausted, the LED will be turned off and FALSE will be returned.
//
//
//
//------------------------------------------------------------------------------------------------------



bool refresh_LED() {

   if ((LEDBitMask != 0) && ((millis() - LEDUpdated) <= LEDTiming))     // Time to change a bit?
        return (true);                                                  //  Nope, wait some more..  (Unless we are starting a new pattern, then let's get that 1st bit out!)


   if ((LEDBitMask == 0) && (LEDRepeat == 0)) {
        _SET_RED_LED(LED_OFF);                                          //  We are all done, make sure the LED is off . . .
        _SET_GREEN_LED(LED_OFF);   
       
        
        #ifdef FEATURE_OUT_LAMP
          if (LEDFOMirror == true)
                _SET_FO_PORT(LED_OFF);                                  //  And adjust the LAMP if so configured.
        #endif
          
        return(false);                                                  //  . . . and return that there is no more
        }


   if (LEDBitMask == 0) {                                               // Reset the LED mask to start with MSB?
        LEDBitMask = 0x8000;                                            // Yes
        LEDRepeat--;                                                    // and take the Counter down one
        }
     else
        LEDBitMask = LEDBitMask >> 1;                                   // Adjust the bit mask and counter, and let the next time back into
                                                                        // refresh_LED() decide if we are finished.  (This will assure if the MSB is
                                                                        // LED-on, there it will be light for timing mS..)

   if ((LEDPattern & LEDBitMask) == 0) {
        _SET_RED_LED(LED_OFF);                                          //  LED should be off . . .
        _SET_GREEN_LED(LED_OFF); 
        
        }
    else {                                                              // LED should be on, but which color?
        if (chargingState == FAULTED) 
            _SET_RED_LED(LED_ON);                                       // FAULTED status gets RED LED blinking.
        else {
            _SET_GREEN_LED(LED_ON);                                     // Normal status gets the GREEN one
#ifdef SYSTEMCAN
            if ((CAN_RBM_sourceID != 0) && (!ignoringRBM))
                _SET_RED_LED(LED_ON);                                   // But if we are taking orders from a CAN MASTER. change the status LED to yellow.
#endif    
            }
        }



#ifdef FEATURE_OUT_LAMP
    if (LEDFOMirror == true) {                                          //  And adjust the LAMP if so configured.
      if ((LEDPattern & LEDBitMask) == 0)  
        _SET_FO_PORT(LED_OFF);    
      else
        _SET_FO_PORT(LED_ON);  
      }
   else {                                                               //  Lamp not being used for active blinking, lets see if we should just have it on :-) 
      if ((chargingState == unknown)  || (chargingState == disabled) || (chargingState == pending_R) || (requiredSensorsFlag != 0))     
        _SET_FO_PORT(LED_ON);                                           //    . .  set it to ON if we are not charging, missing sensor.  OFF if all is OK.
      else 
        _SET_FO_PORT(LED_OFF);  
     }

#endif
 
    

   LEDUpdated = millis();                                               //  Reset timer, and    
   return (true);                                                       //    say there is More to Come!
   
}





