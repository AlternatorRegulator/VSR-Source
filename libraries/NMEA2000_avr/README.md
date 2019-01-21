# NMEA2000_avr

This library provides support to AVR CPUs which contain an internal CAN controller when used with the [NMEA2000](https://github.com/ttlappalainen/NMEA2000) library.

See https://github.com/thomasonw/NMEA2000_avr  &  https://github.com/thomasonw/avr_can

Currently supported CPUs include:
* ATmega16M1
* ATmega32M1
* ATmega64M1
* ATmega32C1
* ATmega64C1
* AT90CAN64
* AT90CAN128
* 

## Usage

This library is is support of ttlappalainen's NMEA2000 library.  See the [NMEA2000](https://github.com/ttlappalainen/NMEA2000) for more examples.  
Also required is AVR_CAN driver <https://github.com/thomasonw/avr_can>   Make sure you have it installed.



    #include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
    #include <N2kMessages.h>

    
    void setup() {
      NMEA2000.open();
    }

    void loop() {
    }

 All examples using one CAN port in the NMEA2000 library will.  The ATmega M1/C1 & AT90CAN CPUs only have one CAN controller.
 

## Hardware


You will need a CAN transceiver like the MCP2551

|      avr CPU      |       MCP2551       | NMEA2000 Cable |
| ----------------- | ------------------- | -------------- |
|        TX         |         TxD         |                |
|        RX         |         RxD         |                |
|                   |        CANH         |      Blue      |
|                   |        CANL         |     White      |
|        Gnd        |         Gnd         |     Black      |
|        5v         |         VDD         |                |
|        Gnd        |         Rs          |                |
|                   | Vref (disconnected) |                |


See also <https://github.com/thomasonw/ATmegaxxM1-C1> for Arduino IDE support of the ATmegaxxM1 and ATmegaxxC1 CPUs
