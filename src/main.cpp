#ifndef __AVR_ATtiny24A__
    #define __AVR_ATtiny24A__
#endif

#define F_CPU 1843200UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>

volatile unsigned int dataIn = 0;

volatile unsigned int spiDO[] = {0, 0}; // SPI data for output
volatile int spiDOIndex = -1;   // Index of next data. If -1 no data is available

/**
 * PA2 - out   // Chip Available - HIGH = Chip is ready to transfer data / rdy for SPI
 * PA3 - in    // Chip Select - PullUp ON, LOW = Chip Selected
 * PA4 - USCK
 * PA5 - MISO
 * PA6 - MOSI
 */
void initSPI() {
    // USIOIE - Enable Overflow Interrupt (USI_OVF_vect)
    // USIWM0 - Enable Three-wire mode. Uses DO, DI, and USCK pins
    // USICS1 - Set clock source to "External, positive edge"
    USICR |= (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1);

    // DDRA |= (0<<DDA4) -> PA4 (USCK) as input
    DDRA |= (1<<DDA5); // PA5 (DO/MISO) as output

    DDRA |= (1 << DDA2); // Indicator that the chip is ready for SPI

    // Activate PullUp Resistor on Chip Select Pin
    PORTA |= (1<<PA3);
}

/**
 * PA0 - ADC0 // Read Cell Voltage
 */
void initADC() {
    // REFS1 - Internal 1.1V voltage reference
    // MUX[5:0] - 001000 = ADC0 is Positive and ADC1 Negative & Gain 1x
    ADMUX |= (1<<REFS1) | (1<<MUX3);

    // ADEN - Enable the ADC
    // ADPS[2:0] - Set Prescaler -> Optimal ADC Frequency is between 50 kHz and 200 kHz
    ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Prescaler 128
}

void initSleepMode() {
    // SM[1:0] to 10 - Power down mode
    MCUCR |= (1<<SM1);

    // The low level of INT0 generates an interrupt request. (Userd for wake up)
    //MCUCR |= (0<<ISC01) | (0<<ISC00);
}

void init() {
    cli(); // Deactivate Interrupts

    initSPI();
    initADC();
    initSleepMode();

    DDRA |= (1<<DDA7);  // Set Active Mode indicator LED Port as Output
    PORTA |= (1<<PA7);  // Activate the indicator LED

    sei(); // Activate Interrupts
}

/**
 *
 */
void enableSPI() {
    // USIOIE - Enable Overflow Interrupt (USI_OVF_vect)
    // USIWM0 - Enable Three-wire mode. Uses DO, DI, and USCK pins
    // USICS1 - Set clock source to "External, positive edge"
    USICR |= (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1);
}

/**
 *
 */
void disableSPI() {
    USICR |= (0<<USIOIE) | (0<<USIWM0) | (0<<USICS1);
}

/**
 *
 */
void measureVoltage(bool discard = false) {
    // With current configuration volage is 1,1*1024/ADCVal

    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC));  // wait till conversion complete

    if (!discard) {
        // ADCL Must be read first, then ADCH
        //adcL = ADCL;
        //adcH = ADCH;
        //adcData = (adcH<<8) | adcL;
        uint16_t adcData = ADCL|(ADCH<<8);

        // factor = gain * referenceVoltage / 1024 * deviderFactor * milliVoltFactor
        // factor = 1.0  * 1.1              / 1024 * 5.285698488   * 1000
        uint16_t milliVolts = adcData * 5.678; // Factor = 1.07421875 * Devider Factor of 5.285698488

        spiDO[0] = milliVolts;
        spiDO[1] = (milliVolts>>8);

        spiDOIndex = 1;
    }
}

/**
 * Initialize the sleep mode
 */
void sleep() {
    // Prepare for Sleep
    cli();
    GIMSK |= (1<<INT0); // Enable the interrupt on INT0 for wake up (only low level interrupt will work for wake up)
    PORTA &= ~(1<<PA7); // Deactivate active mode indicator LED
    ADCSRA &= ~(1<<ADEN); // Disable adc
    sleep_enable();
    sei();

    // Sleep
    sleep_cpu();

    // After wake up
    cli();
    sleep_disable();
    GIMSK &= ~(1<<INT0); // Disable INT0 Interrupt after wake up
    ADCSRA |= (1<<ADEN); // Enable ADC
    PORTA |= (1<<PA7); // Activate active mode indicator LED
    sei();
}

/**
 * If PA3 is HIGH, chip is not selected
 * If PA3 is LOW, chip is selected
 */
bool isChipSelected() {
    return !(PINA & (1 << PINA3));
}

int main(void) {
    init();
    sleep();

    while(1) {
        measureVoltage(true); // Discard first measurement (Reccomended on Page 145)
        measureVoltage();
        USIDR = spiDO[spiDOIndex]; // USIDR is data which should be transmitted
        enableSPI();

        PORTA |= (1<<PA2); // turn Chip is Ready indicator ON

        while(spiDOIndex >= 0); // Wait until all data is transmitted

        PORTA &= ~(1<<PA2); // turn Chip is Ready indicator OFF
        disableSPI();

        sleep();

        //if (!isChipSelected()) {
        //    disableSPI();
        //} else {
        //    if (voltageBufferPos < 0) measureVoltage();
        //    enableSPI();
        //}

        //if (dataIn == 1) {
        //    PORTB |= (1<<PB0);
        //} else {
        //    PORTB &= ~(1<<PB0);
        //}
        //PORTB |= (1<<PB0);
		//_delay_ms(500);

		//PORTB &= ~(1<<PB0);
		//_delay_ms(500);
    }

    return 0;
}

ISR(EXT_INT0_vect) {
    // Triggered on wake up
}

ISR(USI_OVF_vect) {
    USISR = (1<<USIOIF);            // Flag löschen
    dataIn = USIDR;                 // Empfangene Daten lesen

    --spiDOIndex; // Decrement index after data is transmitted
    if (spiDOIndex < 0) {
        USIDR = 0; // Setze 0 wenn keine weiteren Daten mehr vorhanden sind
    } else {
        USIDR = spiDO[spiDOIndex]; // Nächsten Datensatz bereitstellen
    }
}
