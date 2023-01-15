/*
 * usbtherm.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Firmware for USBTherm, a simple AVR micro controller based USB thermometer.
 *
 * Created on: 26.05.2016
 *     Author: dode@luniks.net
 *
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "usbdrv/usbdrv.h"
/* #include "usbdrv/oddebug.h" */

/* The pins for V-USB are set up in usbdrv/usbconfig.h */
#define PIN_TEMP        2 // ADC2 (PB4) single ended input
#define PIN_HUMI        3 // ADC3 (PB3) single ended input
#define AREF_MV         5000

/* Weight of the exponential weighted moving average as bit shift */
#define EWMA_BS         2

/* Output of the TMP36 is 750 mV @ 25°C, 10 mV per °C */
#define TMP36_MV_0C     500

/* Timer0 interrupts per second */
#define INTS_SEC        F_CPU / (1024UL * 255)

/* Request from the kernel driver */
#define CUSTOM_REQ_TEMP 0

static volatile uint8_t ints = 0;
static int32_t mVAvgTmp = -1;

ISR(TIMER0_COMPA_vect) {
    ints++;
}

EMPTY_INTERRUPT(ADC_vect);

/**
 * Sets up the timer.
 */
static void initTimer(void) {
    // timer0 clear timer on compare match mode, TOP OCR0A
    TCCR0A |= (1 << WGM01);
    // timer0 clock prescaler/1024/255 ~ 46 Hz @ 12 MHz ~ 61 Hz @ 16 MHz
    TCCR0B |= (1 << CS02) | (1 << CS00);
    OCR0A = 255;

    // enable timer0 compare match A interrupt
    TIMSK |= (1 << OCIE0A);
}

/**
 * Sets up the ADC.
 */
static void initADC(void) {
    set_sleep_mode(SLEEP_MODE_IDLE);

    // disable digital input on the ADC inputs to reduce digital noise
    DIDR0 = 0b00011000;
    // ADC clock prescaler/128 ~ 129 kHz @ 16.5 MHz
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    // enable ADC interrupt
    ADCSRA |= (1 << ADIE);
    // enable ADC
    ADCSRA |= (1 << ADEN);
}

/**
 * Sets up V-USB.
 */
static void initUSB(void) {
    wdt_disable();
    usbInit();
    usbDeviceDisconnect();
    _delay_ms(255);
    usbDeviceConnect();
}

/**
 * Measures the voltage at the given pin with 16x oversampling during
 * idle sleep mode to reduce digital noise, updates the given exponential
 * weighted moving average and returns it.
 */
static int32_t measure(uint8_t pin, int32_t mVAvg) {
    ADMUX = (0xf0 & ADMUX) | pin;

    uint32_t overValue = 0;
    for (uint8_t i = 0; i < 16; i++) {
        ADCSRA |= (1 << ADSC);
        sleep_mode();
        overValue += ADC;
    }

    int16_t mV = (((overValue >> 2) * AREF_MV) >> 12);

    if (mVAvg == -1) {
        // use first measurement as initial value for average
        return mV << EWMA_BS;
    } else {
        // calculate EWMA
        return mV + mVAvg - ((mVAvg - EWMA_BS) >> EWMA_BS);
    }
}

/**
 * Called by the driver to read the temperature value in °C x10.
 */
uchar usbFunctionRead(uchar *data, uchar len) {

    /**
     * The temperature value is short enough to be read at once.
     * TODO okay like that?
     */
    int16_t tempx10 = (mVAvgTmp >> EWMA_BS) - TMP36_MV_0C;
    snprintf((char *)data, len, "%d", tempx10);

    return len;
}

/**
 * Sets up the implemented requests.
 */
usbMsgLen_t usbFunctionSetup(uchar data[8]) {
    usbRequest_t *req = (void *)data;

    /*
     * The only implemented request - tells the driver to read data
     * (the temperature value) with usbFunctionRead().
     * Also, USB_CFG_IMPLEMENT_FN_READ must be set to 1 in usbdrv/usbconfig.h.
     */
    if (req->bRequest == CUSTOM_REQ_TEMP) {
        return USB_NO_MSG;
    }

    /**
     * Return no data for unimplemented requests.
     */
    return 0;
}

/**
 * From https://codeandlife.com/2012/02/22/v-usb-with-attiny45-attiny85-without-a-crystal/
 */
// Called by V-USB after device reset
void hadUsbReset() {
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal, bestCal = 0, step, region;

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

    initTimer();
    initADC();
    initUSB();

    // enable global interrupts
    sei();

    while (true) {

        /**
         * About every second, turn on the LED, measure the temperature,
         * print the temperature via USART, turn off the LED.
         */
        if (ints >= INTS_SEC) {
            ints = 0;
            mVAvgTmp = measure(PIN_TEMP, mVAvgTmp);
        }
        usbPoll();
    }

    return 0;
}
