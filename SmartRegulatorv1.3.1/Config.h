//
//      Config.h
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


#ifndef _CONFIG_H_
#define _CONFIG_H_


                                                       
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




#define  FIRMWARE_VERSION  "AREG1.3.1"                              // Sent out with SST; status string as well as the CAN product ID 
                                                                    // Use format of xxxxyyyyyy  where xxxx= 4 chars of device type, and yyy= varying chars of version number.
                                                                    // Note the yyy's are to follow the Semantiic Version spec guidelines:  https://semver.org/
                                                                    // Make sure to use `-' for any trailing modifiers after the final version number 'number'

//#define SIMULATION                                                // Enables forcing code to allow for bench testing on stand-alone  card..
//#define DEBUG                                                     // Debug code, Extra information sent via the Serial port for use during mocking-up and/or Debugging
                                                                    //  The Debug switch also allows the RN41 to accept commands externally, and bypasses the need to change 
                                                                    //  the Bluetooth name and PIN before accepting any ASCII change commands.






    


#define  OSE_ALTERNATOR                                             //  Select the ONE device type this is
//#define OSE_GENERATOR
//#define OSE_SOLAR
//#define OSE_BMS
//#define OSE_BATMON
//#define OSE_DISPLAY1                // Type 1 OSEnergy Remote Display (commonly used with DC Generator)






                                                         
#if   defined OSE_ALTERNATOR
    #include "SmartRegulator.h"    
#elif defined OSE_GENERATOR
    #include "DCGen.h"
#elif defined OSE_BATMON
    #include "BatMon.h"
#elif defined OSE_DISPLAY1
    #include "DCRemote.h"           // DC Generator type Display

    
#else
        #error Unknown device type 
#endif 
    



#endif  // _CONFIG_H_



