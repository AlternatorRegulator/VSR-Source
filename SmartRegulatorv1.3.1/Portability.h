//
//      Portability.h
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


#ifndef _PORTABILITY_H_
#define _PORTABILITY_H_


                                                       
/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              PORTABILITY                                             *
 *                                                                                      *
 *                                                                                      *
 *      This include file contains all the 'ugly' definitions and conversions           *
 *      used to allow much of the VSR projects source to be used with the Arduino       * 
 *      IDE for use wiht AVR CPUs, or the STm32 cubeMX / Kiel uVsion environment        *
 *      when targeting the STM32F07 CPU.                                                *
 *                                                                                      *
 *       Note that projecct/board specifics (e.g. port assigments) are NOT contained    *
 *       in this file.  Only compiler / linker stuff                                    *
 *                                                                                      *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/
       
                                
#if defined (CPU_AVR) || \
    defined (CPU_AVRCAN)                                    // Arduino IDE target
    
    #include <Arduino.h>                                    // Pick up Arduino specifics, ala PROGMEM
    #include <avr/wdt.h>     
    #include <MemoryFree.h>
    #include "System.h"

                                             
                                                
     
                                                        
    //-- Some #defines to allow for more common code across different CPUs, w/o a TON of #ifdefs
    #define   sample_feature_IN_port()   digitalRead(FEATURE_IN_PORT)                                                      
    

    #define ASCII_read(v)          Serial.read()
    #define ASCII_RxAvailable(v)  (Serial.available() > 0)
    #define ASCII_write(v)         Serial.write(v)
    #define Serial_flush();        Serial.flush();
    
 
  
                                                                            
#elif defined(CPU_STM32)
      
  
    #include <stdint.h>                             // Not using an 'OS', so need to do all the basic includes.
    #include <stdbool.h>
    #include <stdio.h>
    #include <inttypes.h>
    #include "stm32f0xx_hal.h"                      // Bring in all the cubeMX and RTOS definitions.
    #include "cmsis_os.h"
    #include "usbd_cdc_if.h"                        // Brings in the USB stuff for ASCII_write and ASCII_read #defs
    #include "System.h"

    #include <RVCMessages.h>                        // https://github.com/thomasonw


    
     
    //----  Some macros, to make porting over of Arduino based code simpler
    //      Be carefull with these, as there is no type checking - and the macros will cause re-evalaution of complex values...
    #define snprintf_P(s, f, ...) snprintf((s), (f), __VA_ARGS__)
    #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
    #define min(a,b) ((a)<(b)?(a):(b))
    #define max(a,b) ((a)>(b)?(a):(b))
    #define lowByte(v)   ((unsigned char) (v))
    #define highByte(v)  ((unsigned char) (((unsigned int) (v)) >> 8))
        
 
  
    #define sample_feature_IN_port()    HAL_GPIO_ReadPin(Feature_IN_GPIO_Port, Feature_IN_Pin)
    #define micros()                    __HAL_TIM_GET_COUNTER(&micros_timer) 
    #define analogWrite(a,pwm)          __HAL_TIM_SET_COMPARE(&Field_PWM_Timer, Field_PWM_Channel, pwm);    
                                                                // THE ONLY THING ANALOGWRITE IS USED FOR IS TO ADJUST THE FIELD PWM. 
   
   
   
    //---  Dummy-ups for Watchdog.
    #define WDTO_8S 1                                                                           // Just a dummy value, to allow common code.
    #ifdef DEBUG
      inline void wdt_enable(int x) {}                                                               // No watchdog while debugging, interfers with uVision debugger!
      inline void wdt_disable(void) {}
      inline void wdt_reset(void) {}                                                               
          
    #else
      inline void wdt_enable(int x) {}                                                              //!! HEY!!!  PUT IN WATCHDOG CODE FOR LIVE SYSTEM
      inline void wdt_disable(void) {}
      inline void wdt_reset(void) {IWDG->KR = 0xAAAA;}                                                //Reload IWDG - See more at: http://embedded-lab.com/blog/?p=9662#sthash.6VNxVSn0.dpuf
    #endif 

  
     
    extern "C" {
        char    USB_ASCII_read(void);                                                                  // These functions are located in usbd_cdc_if.c
        uint8_t USB_ASCII_RxAvailable(void);  
        }
    #define ASCII_RxAvailable(v)    USB_ASCII_RxAvailable()
    #define ASCII_read(v)           USB_ASCII_read()
    #define ASCII_write(v)          CDC_Transmit_FS((uint8_t*)v,strlen((char*)v))                   // was --> puts(v)
    inline int Serial_flush(void) { return true;}                                                    // Just a dummy
            
        
   
   

    // -- From the Maple STM32 porting file arduino.h



    #define PROGMEM
    #define PGM_P  const char *
    #define PSTR(str) (str)

    #define _SFR_BYTE(n) (n)

    typedef void prog_void;
    typedef char prog_char;
    typedef unsigned char prog_uchar;
    typedef int8_t prog_int8_t;
    typedef uint8_t prog_uint8_t;
    typedef int16_t prog_int16_t;
    typedef uint16_t prog_uint16_t;
    typedef int32_t prog_int32_t;
    typedef uint32_t prog_uint32_t;

    #define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
    #define strcpy_P(dest, src) strcpy((dest), (src))
    #define strcat_P(dest, src) strcat((dest), (src))
    #define strcmp_P(a, b) strcmp((a), (b))
    #define strstr_P(a, b) strstr((a), (b))
    #define strlen_P(a) strlen((a))
    #define sprintf_P(s, f, ...) sprintf((s), (f), __VA_ARGS__)


    #define pgm_read_float(addr) (*(const float *)(addr))

    #define pgm_read_byte_near(addr) pgm_read_byte(addr)
    #define pgm_read_word_near(addr) pgm_read_word(addr)
    #define pgm_read_dword_near(addr) pgm_read_dword(addr)
    #define pgm_read_float_near(addr) pgm_read_float(addr)
    #define pgm_read_byte_far(addr) pgm_read_byte(addr)
    #define pgm_read_word_far(addr) pgm_read_word(addr)
    #define pgm_read_dword_far(addr) pgm_read_dword(addr)
    #define pgm_read_float_far(addr) pgm_read_float(addr)

                                                  
    extern "C" {
      void setup(void);
      void loop();
      }  


    //----  CAN / NMEA2000 linkages.
    //      Need to do them manually, as the cubeMX does not have auto integration into Timo's lib
    //      Note that the implementation of these is in system.cpp, but am having issues sorting out pre-directives
    //      and #ifdefs's, do the declaration of these lives here.

    #include <NMEA2000.h>
    #include <N2kMsg.h>
    class tNMEA2000_cubeMX : public tNMEA2000
        {
        protected:
            virtual bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent);
            virtual bool CANOpen();
            virtual bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);

        public:
            tNMEA2000_cubeMX();

        };



        //class Serial : public N2kStream {
          class tcubeMXStream : public N2kStream {
        public:
           virtual int read();
           virtual size_t write(const uint8_t* data, size_t size);
        };




    

#else
        #error Unsupported Smart Regulator CPU 
#endif 













#endif  // _PORTABILITY_H_




