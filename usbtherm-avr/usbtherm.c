/*
 * usbtherm.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Firmware for USBTherm2, a simple AVR micro controller based USB thermometer
 * and hygrometer.
 *
 * Just fits on an ATtiny45 with ~ 3900 bytes (95% full)
 *
 * Created on: 15.01.2023
 *     Author: torsten.roemer@luniks.net
 *
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "usbdrv/usbdrv.h"
/* #include "usbdrv/oddebug.h" */

/* The pins for V-USB are set up in usbdrv/usbconfig.h */
#define PIN_TMP         2 // ADC2 (PB4) single ended input
#define PIN_RH          3 // ADC3 (PB3) single ended input
#define PIN_LED         PB0
#define AREF_MV         5000

/* Weight of the exponential weighted moving average as bit shift */
#define EWMA_BS         6

/* Output of the TMP36 is 750 mV @ 25°C, 10 mV per °C */
#define TMP36_MV_0C     500

/* Timer0 interrupts per second */
#define INTS_SEC        F_CPU / (1024UL * 255)

/* Request from the host */
#define CUSTOM_REQ_TMP  0

static volatile uint8_t ints = 0;
static uint32_t mVAvgTmp = -1;
static uint32_t mvAvgRh = -1;

ISR(TIMER0_COMPA_vect) {
    ints++;
}

EMPTY_INTERRUPT(ADC_vect);

/*
 * Sets up the pins.
 */
static void initPins(void) {
	// set LED pin as output pin
	DDRB |= (1 << PB0);
}

/*
 * Sets up the timer.
 */
static void initTimer(void) {
    // timer0 clear timer on compare match mode, TOP OCR0A
    TCCR0A |= (1 << WGM01);
    // timer0 clock prescaler/1024/255 ~ 63 Hz @ 16.5 MHz
    TCCR0B |= (1 << CS02) | (1 << CS00);
    OCR0A = 255;

    // enable timer0 compare match A interrupt
    TIMSK |= (1 << OCIE0A);
}

/*
 * Sets up the ADC.
 */
static void initADC(void) {
    set_sleep_mode(SLEEP_MODE_IDLE);

    // disable digital input on the ADC inputs to reduce power consumption
    // and digital noise
    DIDR0 = 0b00011000;
    // ADC clock prescaler/128 ~ 129 kHz @ 16.5 MHz
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    // enable ADC interrupt
    ADCSRA |= (1 << ADIE);
    // enable ADC
    ADCSRA |= (1 << ADEN);
}

/*
 * Sets up V-USB.
 */
static void initUSB(void) {
    wdt_disable();
    usbInit();
    usbDeviceDisconnect();
    _delay_ms(255);
    usbDeviceConnect();
}

/*
 * Measures the voltage at the given pin with 16x oversampling during
 * idle sleep mode to reduce digital noise, updates the given exponential
 * weighted moving average and returns it.
 */
static uint32_t measure(uint8_t pin, uint32_t mVAvg) {
    ADMUX = (0xf0 & ADMUX) | pin;

    uint32_t overValue = 0;
    for (uint8_t i = 0; i < 16; i++) {
        ADCSRA |= (1 << ADSC);
        sleep_mode();
        overValue += ADC;
    }

    uint32_t mV = ((overValue >> 2) * AREF_MV) >> 12;

    if (mVAvg == -1) {
        // use first measurement as initial value for average
        return mV << EWMA_BS;
    } else {
        // calculate EWMA
        return mV + mVAvg - ((mVAvg - EWMA_BS) >> EWMA_BS);
    }
}

/*
 * Sets up the implemented requests.
 */
usbMsgLen_t usbFunctionSetup(uchar data[8]) {
    usbRequest_t *req = (void *)data;

    /*
     * The only implemented request. It calculates the temperature and humidity
     * from the respective average measurement value and transfers both 
     * multiplied by 10 and separated by the pipe character to the host.
     * Setting usbMsgPtr is a nice alternative to using usbFunctionRead().
     */
    if (req->bRequest == CUSTOM_REQ_TMP) {
        if (mVAvgTmp == -1 || mvAvgRh == -1) {
            // return no data if not yet measured
            return 0;
        }

        // temperature in °C multiplied by 10
        int16_t tmpx10 = (mVAvgTmp >> EWMA_BS) - TMP36_MV_0C;
        // relative humidity in % multiplied by 10
        uint32_t rhx10 = (mvAvgRh * 100 - (75750UL << EWMA_BS)) / (318UL << EWMA_BS);
        // temperature compensation of relative humidity
        rhx10 = (rhx10 * 1000000) / (1054600 - tmpx10 * 216UL);

        static char msg[16];
        snprintf(msg, sizeof(msg), "%d|%ld", tmpx10, rhx10);
        usbMsgPtr = (usbMsgPtr_t)msg;

        return sizeof(msg);
    }

    // return no data for unimplemented requests
    return 0;
}

/*
 * Calibrates the internal oscillator based on measurements from V-USB
 * after device reset, saving a crystal and two capaciators and making the two
 * required ADC input pins available. Thanks to Joonas Pihlajamaa, 
 * https://codeandlife.com/2012/02/22/v-usb-with-attiny45-attiny85-without-a-crystal/
 */
void hadUsbReset() {
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal = 0, bestCal = 0, step = 0, region = 0;

    // do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
    for (region = 0; region <= 1; region++) {
        frameLength = 0;
        trialCal = (region == 0) ? 0 : 128;
        
        for (step = 64; step > 0; step >>= 1) { 
            if (frameLength < targetLength) { // true for initial iteration
                trialCal += step; // frequency too low
            } else {
                trialCal -= step; // frequency too high
            }
                
            OSCCAL = trialCal;
            frameLength = usbMeasureFrameLength();
            
            if (abs(frameLength - targetLength) < bestDeviation) {
                bestCal = trialCal; // new optimum found
                bestDeviation = abs(frameLength -targetLength);
            }
        }
    }

    OSCCAL = bestCal;
}

int main(void) {

    initPins();
    initTimer();
    initADC();
    initUSB();

    // enable global interrupts
    sei();

    while (true) {

        /*
         * Measure temperature and humidity and update the average values
         * and flash the LED once about every second.
         */
        if (ints >= INTS_SEC) {
            ints = 0;
            PORTB |= (1 << PIN_LED);
            mVAvgTmp = measure(PIN_TMP, mVAvgTmp);
            mvAvgRh = measure(PIN_RH, mvAvgRh);
            PORTB &= ~(1 << PIN_LED);
        }
        usbPoll();
    }

    return 0;
}
