#include "LS7166.h"

LS7166::LS7166(uint8_t cs1_pin, uint8_t cs2_pin, bool enc1_reverse, bool enc2_reverse) {
    _cs1_pin = cs1_pin;
    _cs2_pin = cs2_pin;
    _enc1_reverse = enc1_reverse;
    _enc2_reverse = enc2_reverse;
}

void LS7166::begin() {
    // Initialize SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // Slower clock for stability
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    // Configure CS pins
    pinMode(_cs1_pin, OUTPUT);
    pinMode(_cs2_pin, OUTPUT);
    digitalWrite(_cs1_pin, HIGH);
    digitalWrite(_cs2_pin, HIGH);

    // Small delay for CS pins to stabilize
    delay(10);

    // Initialize both encoders
    init_encoders();

    // Small delay after initialization
    delay(10);
}

void LS7166::write_LS7166(uint8_t cs_pin, uint8_t reg, uint8_t data) {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(data);
    digitalWrite(cs_pin, HIGH);
}

uint8_t LS7166::read_LS7166(uint8_t cs_pin, uint8_t reg) {
    uint8_t data;
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    data = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    return data;
}

void LS7166::init_encoder(uint8_t cs_pin) {
    // Configure MDR0: 4X quadrature, free running mode (same as working example)
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x88); // Write to MDR0
    SPI.transfer(0x03); // Configure to 4 byte mode
    digitalWrite(cs_pin, HIGH);
}

void LS7166::init_encoder1() {
    init_encoder(_cs1_pin);
}

void LS7166::init_encoder2() {
    init_encoder(_cs2_pin);
}

void LS7166::init_encoders() {
    init_encoder1();
    init_encoder2();
}

int32_t LS7166::read_encoder1() {
    unsigned int count_1, count_2, count_3, count_4;
    int32_t count_value;

    digitalWrite(_cs1_pin, LOW);
    SPI.transfer(0x60); // Request count (same as working example)
    count_1 = SPI.transfer(0x00); // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00); // Read lowest order byte
    digitalWrite(_cs1_pin, HIGH);

    // Calculate encoder count (highest byte first)
    count_value = (((int32_t)count_1 << 24) + ((int32_t)count_2 << 16) + ((int32_t)count_3 << 8) + (int32_t)count_4);

    // Reverse direction if needed
    if (_enc1_reverse) {
        count_value = -count_value;
    }

    return count_value;
}

int32_t LS7166::read_encoder2() {
    unsigned int count_1, count_2, count_3, count_4;
    int32_t count_value;

    digitalWrite(_cs2_pin, LOW);
    SPI.transfer(0x60); // Request count (same as working example)
    count_1 = SPI.transfer(0x00); // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00); // Read lowest order byte
    digitalWrite(_cs2_pin, HIGH);

    // Calculate encoder count (highest byte first)
    count_value = (((int32_t)count_1 << 24) + ((int32_t)count_2 << 16) + ((int32_t)count_3 << 8) + (int32_t)count_4);

    // Reverse direction if needed
    if (_enc2_reverse) {
        count_value = -count_value;
    }

    return count_value;
}

void LS7166::reset_encoder1() {
    // Set encoder's data register to 0 (same as working example)
    digitalWrite(_cs1_pin, LOW);
    SPI.transfer(0x98); // Write to DTR
    SPI.transfer(0x00); // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00); // Lowest order byte
    digitalWrite(_cs1_pin, HIGH);

    delayMicroseconds(100);

    // Transfer DTR to CNTR
    digitalWrite(_cs1_pin, LOW);
    SPI.transfer(0xE0);
    digitalWrite(_cs1_pin, HIGH);
}

void LS7166::reset_encoder2() {
    // Set encoder's data register to 0 (same as working example)
    digitalWrite(_cs2_pin, LOW);
    SPI.transfer(0x98); // Write to DTR
    SPI.transfer(0x00); // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00); // Lowest order byte
    digitalWrite(_cs2_pin, HIGH);

    delayMicroseconds(100);

    // Transfer DTR to CNTR
    digitalWrite(_cs2_pin, LOW);
    SPI.transfer(0xE0);
    digitalWrite(_cs2_pin, HIGH);
}

void LS7166::reset_encoders() {
    reset_encoder1();
    reset_encoder2();
}

int32_t LS7166::get_encoder_difference() {
    int32_t enc1 = read_encoder1();
    int32_t enc2 = read_encoder2();
    return enc1 - enc2;
}

int32_t LS7166::get_encoder_average() {
    int32_t enc1 = read_encoder1();
    int32_t enc2 = read_encoder2();
    return (enc1 + enc2) / 2;
}