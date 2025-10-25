#ifndef LS7166_H
#define LS7166_H

#include <Arduino.h>
#include <SPI.h>

// LS7166 Register addresses
#define MDR0 0x08
#define MDR1 0x10
#define DTR  0x18
#define CNTR 0x20
#define OTR  0x28
#define STR  0x30
#define CMD  0x38

// LS7166 Commands
#define CLR_CNTR 0x20
#define RLD_CNTR 0x40
#define RLD_OTR  0x50
#define RESET_BP 0x01
#define RESET_E  0x02
#define LATCH_CNTR 0x03
#define SET_DTR_TO_CNTR 0x04
#define MCR_MDR0 0x88
#define MCR_MDR1 0x90

class LS7166 {
private:
    uint8_t _cs1_pin;
    uint8_t _cs2_pin;
    bool _enc1_reverse;
    bool _enc2_reverse;

    void write_LS7166(uint8_t cs_pin, uint8_t reg, uint8_t data);
    uint8_t read_LS7166(uint8_t cs_pin, uint8_t reg);
    void init_encoder(uint8_t cs_pin);

public:
    LS7166(uint8_t cs1_pin, uint8_t cs2_pin, bool enc1_reverse = false, bool enc2_reverse = false);

    void begin();

    // Individual encoder functions
    void init_encoder1();
    void init_encoder2();
    void init_encoders();

    int32_t read_encoder1();
    int32_t read_encoder2();

    void reset_encoder1();
    void reset_encoder2();
    void reset_encoders();

    // Differential calculation functions
    int32_t get_encoder_difference();
    int32_t get_encoder_average();
};

#endif