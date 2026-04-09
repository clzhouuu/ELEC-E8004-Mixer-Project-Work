#include <msp432p401r.h>
#include <bolt.h>
#include <board_config.h>
#include <MSP432P4xx/gpio.h>
#include <MSP432P4xx/spi.h>


// set high or low
static void pin_set(uint_fast8_t port, uint_fast16_t pin, uint8_t value) {
    if (value) {
        GPIO_setOutputHighOnPin(port, pin);
    } else {
        GPIO_setOutputLowOnPin(port, pin);
    }
}


// read pin
static uint8_t pin_read(uint_fast8_t port, uint_fast16_t pin) {
    return GPIO_getInputPinValue(port, pin);
}

// BOLT inialization
uint8_t bolt_init(void) {

    // MODE, starts LOW reading
    GPIO_setAsOutputPin(BOLT_MODE_PORT, BOLT_MODE_PIN);
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);

    // REQ, starts LOW no request
    GPIO_setAsOutputPin(BOLT_REQ_PORT, BOLT_REQ_PIN);
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);

    // ACK pin
    GPIO_setAsInputPin(BOLT_ACK_PORT, BOLT_ACK_PIN);

    // IND pin
    GPIO_setAsInputPin(BOLT_IND_PORT, BOLT_IND_PIN);

    // SPI pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        BOLT_SCK_PORT,
        BOLT_SCK_PIN | BOLT_MOSI_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
        BOLT_MISO_PORT,
        BOLT_MISO_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // configure SPI module
    eUSCI_SPI_MasterConfig spi_config = {
        .selectClockSource     = EUSCI_SPI_CLOCKSOURCE_SMCLK,
        .clockSourceFrequency  = 12000000,
        .desiredSpiClock       = 4000000,
        .msbFirst              = EUSCI_SPI_MSB_FIRST,
        .clockPhase            = EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
        .clockPolarity         = EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        .spiMode               = EUSCI_SPI_3PIN
    };
    SPI_initMaster(EUSCI_B0_BASE, &spi_config);
    SPI_enableModule(EUSCI_B0_BASE);

    // return 1 if BOLT is ready and ACK should be 0
    return (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 0);
}

// chekcs IND for if data is available
uint8_t bolt_data_available(void) {
    return pin_read(BOLT_IND_PORT, BOLT_IND_PIN);
}

// APP to CP write 
static uint8_t bolt_write(uint8_t* data, uint8_t len) {

    // MODE HIGH
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 1);

    // REQ HIGH, requesting transfer
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 1);

    // queue full
    uint32_t timeout = 10000;
    while (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 0) {
        timeout--;

        if (timeout == 0) {
            pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
            pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);
            return 0; 
        }
    }

    // send bytes over SPI
    uint8_t i;
    for (i = 0; i < len; i++) {
        while (!EUSCI_B_SPI_getInterruptStatus(
            EUSCI_B0_BASE,
            EUSCI_B_SPI_TRANSMIT_INTERRUPT));
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, data[i]);
    }

    // wait for last byte to finish
    while (EUSCI_B_SPI_isBusy(EUSCI_B0_BASE));

    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);

    return 1;  
}

// BOLT READ reading from CP to APP
static uint8_t bolt_read(uint8_t* buf, uint8_t* len) {

    // MODE LOW, read
    pin_set(BOLT_MODE_PORT, BOLT_MODE_PIN, 0);

    // REQ HIGH, request data
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 1);

    // wait for ACK to go high
    uint32_t timeout = 10000;
    while (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 0) {
        timeout--;

        if (timeout == 0) {
            pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);
            return 0;
        }
    }

    // receive bytes until ACK drops LOW
    *len = 0;
    while (pin_read(BOLT_ACK_PORT, BOLT_ACK_PIN) == 1) {
        // send dummy byte to clock in data
        EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, 0x00);
        while (!EUSCI_B_SPI_getInterruptStatus(
            EUSCI_B0_BASE,
            EUSCI_B_SPI_RECEIVE_INTERRUPT));
        buf[*len] = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
        (*len)++;
    }

    // set REQ LOW
    pin_set(BOLT_REQ_PORT, BOLT_REQ_PIN, 0);

    return 1; 
}


// send a framed message
uint8_t bolt_send(uint8_t channel, const void* payload, uint8_t length) {
    if (length == 0 || length > BOLT_MAX_PAYLOAD) {
        return 0;
    }

    // frame structure
    uint8_t frame[3 + BOLT_MAX_PAYLOAD + 1];
    uint8_t i;

    frame[0] = BOLT_MAGIC;
    frame[1] = channel;
    frame[2] = length;

    // payload conversion
    const uint8_t* p = (const uint8_t*)payload;

    //
    uint8_t checksum = channel ^ length;

    // copy payload and compute checksum
    for (i = 0; i < length; i++) {
        frame[3 + i] = p[i];
        checksum ^= p[i];
    }
    frame[3 + length] = checksum;

    return bolt_write(frame, 3 + length + 1);
}

// receive one framed message
uint8_t bolt_recv(uint8_t* channel_out, uint8_t* buf, uint8_t* len_out) {

    uint8_t raw[3 + BOLT_MAX_PAYLOAD + 1];
    uint8_t raw_len = 0;
    uint8_t i;

    // error if no data or too short or bad magic byte
    if (!bolt_read(raw, &raw_len) || raw_len < 4 || raw[0] != BOLT_MAGIC ) {
        return 0;
    }
    
    uint8_t channel = raw[1];
    uint8_t length = raw[2];

    if (length > BOLT_MAX_PAYLOAD) {
        return 0;
    }

    if (raw_len < (uint8_t)(3 + length + 1)) {
        return 0;
    }

    // validate checksum
    uint8_t frame = channel ^ length;
    for (i = 0; i < length; i++) {
        frame ^= raw[3 + i];
    }

    if (raw[3 + length] != frame) {
        return 0;
    }

    // copy out
    *channel_out = channel;
    *len_out = length;

    for (i = 0; i < length; i++) {
        buf[i] = raw[3 + i];
    }

    return 1;
}