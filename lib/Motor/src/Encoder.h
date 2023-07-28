#pragma once
#include <Arduino.h>
#include <driver/pcnt.h>
#include <driver/gpio.h>

#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX
#define _INT16_MAX 32766
#define _INT16_MIN -32766
#define ISR_CORE_USE_DEFAULT (0xffffffff)

typedef void (*enc_isr_cb_t)(void*);

enum puType {
    UP,
    DOWN,
    NONE
};

enum enc_type {
    both_edge,
    single_edge
};

class Encoder
{
private:
    static bool attached_interrupt;
    void attach(int pinE, int pin_dir, enum enc_type et);
    bool working = false;
    bool direction = false;//false forward, true reverse
    bool attached = false;

public:
    int64_t get_raw_count();
    int pin = 0;
    volatile int64_t count = 0;
    volatile uint64_t micro_last = 0;
    volatile uint64_t micro_between_ticks = 0;
    int64_t read();
    int64_t read_and_clear();
    int64_t pause();
    int64_t resume();
    int64_t clear();
    void attach_both_edge(int pin_enc, int pin_dir);
    void attach_single_edge(int pin_enc, int pin_dir);
    void set_filter(uint16_t value);
    bool is_attached(){return attached_interrupt;}
    void detach();
    // bool always_interrupt; //not used
    gpio_num_t pin_encoder;
    gpio_num_t pin_direction;
    pcnt_unit_t unit;
    pcnt_config_t enc_config;
    static uint32_t isrServiceCpuCore;
    static enum puType use_internal_weak_pull_resistors;
    static Encoder *encoders[MAX_ESP32_ENCODERS];
    enc_isr_cb_t _enc_isr_cb;
    void* _enc_isr_cb_data;
    Encoder(enc_isr_cb_t enc_isr_cb = nullptr, void* enc_isr_cb_data = nullptr);
    virtual ~Encoder();
};

#pragma once