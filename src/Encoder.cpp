#include "Encoder.h"
#include <soc/soc_caps.h>
#include <soc/pcnt_struct.h>
#include <esp_log.h>
#include <esp_ipc.h>

static const char *TAG = "Encoder";

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

enum puType Encoder::use_internal_weak_pull_resistors = DOWN;
uint32_t Encoder::isrServiceCpuCore = ISR_CORE_USE_DEFAULT;
Encoder *Encoder::encoders[MAX_ESP32_ENCODERS] = {NULL, };

bool Encoder::attached_interrupt = false;

Encoder::Encoder(enc_isr_cb_t enc_isr_cb, void* enc_isr_cb_data):
    pin_encoder{(gpio_num_t) 0},
    pin_direction{(gpio_num_t) 0},
    unit{(pcnt_unit_t) -1},
    count{0},
    enc_config{},
    _enc_isr_cb{enc_isr_cb},
    _enc_isr_cb_data{enc_isr_cb_data},
    attached{false},
    direction{false},
    working{false}
{}

Encoder::~Encoder(){}

#define COUNTER_H_LIM h_lim_lat
#define COUNTER_L_LIM l_lim_lat

static void encoder_pcnt_intr_handler(void* arg){
    Encoder *encoder_copy = static_cast<Encoder *>(arg);
    pcnt_unit_t unit = encoder_copy->enc_config.unit;
    _ENTER_CRITICAL();
    if(PCNT.status_unit[unit].COUNTER_H_LIM){
        encoder_copy->count += encoder_copy->enc_config.counter_h_lim;
        pcnt_counter_clear(unit);
    } else if (PCNT.status_unit[unit].COUNTER_L_LIM){
        encoder_copy->count += encoder_copy->enc_config.counter_l_lim;
        pcnt_counter_clear(unit);
    } 
    // not going to use always interrupt
    // else if(encoder_copy->always_interrupt && (PCNT.status_unit[unit].thres0_lat || PCNT.status_unit[unit].thres1_lat)){
    //     int16_t c;
    //     pcnt_get_counter_value(unit, &c);
    //     encoder_copy->count += c;
    //     pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
    //     pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
    //     pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    //     pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    //     pcnt_counter_clear(unit);
    //     if(encoder_copy->_enc_isr_cb){
    //         encoder_copy->_enc_isr_cb(encoder_copy->_enc_isr_cb_data);
    //     }
    // }
    _EXIT_CRITICAL();
}

static IRAM_ATTR void ipc_install_isr_on_core(void *arg){
    esp_err_t *result = (esp_err_t*) arg;
    *result = pcnt_isr_service_install(0);
}

void Encoder::attach(int pin_encoder_, int pin_direction_, enum enc_type et){
    if(attached){
        ESP_LOGE(TAG, "Encoder already attached!");
        return;
    }
    int index = 0;
    for(; index < MAX_ESP32_ENCODERS; index++){
        if(Encoder::encoders[index] == NULL){
            encoders[index] = this;
            break;
        }
    }
    if(index == MAX_ESP32_ENCODERS){
        ESP_LOGE(TAG, "Can't attach new encoders: Encoders full");
        return;
    }

    // Set data now that pin attach checks are done
    unit = (pcnt_unit_t) index;
    this->pin_encoder = (gpio_num_t) pin_encoder_;
    this->pin_direction = (gpio_num_t) pin_direction_;

    // Set up the IO state of the pin
    gpio_pad_select_gpio(pin_encoder);
    gpio_pad_select_gpio(pin_direction);
    gpio_set_direction(pin_encoder, GPIO_MODE_INPUT);
    gpio_set_direction(pin_direction, GPIO_MODE_OUTPUT);
    if(use_internal_weak_pull_resistors == DOWN){
        gpio_pulldown_en(pin_encoder);
    }
    if(use_internal_weak_pull_resistors == UP){
        gpio_pullup_en(pin_encoder);
    }

    //Setup encoder PCNT Config
    enc_config.pulse_gpio_num = pin_encoder; //Tachometer
    enc_config.ctrl_gpio_num = pin_direction; //Direction gpio

    enc_config.unit = unit;
    enc_config.channel = PCNT_CHANNEL_0;

    enc_config.pos_mode = PCNT_COUNT_INC;
    enc_config.neg_mode = et == both_edge ? PCNT_COUNT_INC : PCNT_COUNT_DIS; //Optionally switch between using both or single edge

    enc_config.lctrl_mode = PCNT_MODE_KEEP;
    enc_config.hctrl_mode = PCNT_MODE_REVERSE;

    enc_config.counter_h_lim = INT16_MAX;
    enc_config.counter_l_lim = INT16_MIN;

    pcnt_unit_config(&enc_config);

    //Channel 1 will be ignored
    enc_config.channel = PCNT_CHANNEL_1;
    enc_config.pos_mode = PCNT_COUNT_DIS;
    enc_config.neg_mode = PCNT_COUNT_DIS;
    enc_config.lctrl_mode = PCNT_MODE_DISABLE;
    enc_config.hctrl_mode = PCNT_MODE_DISABLE;

    pcnt_unit_config(&enc_config);

    set_filter(250); // Tested value for my case only. Better to calibrate on different settings

    //Enable events when reaching counters max and min values
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    pcnt_counter_pause(unit);

    if(!attached_interrupt){
        if(isrServiceCpuCore == ISR_CORE_USE_DEFAULT || isrServiceCpuCore == xPortGetCoreID()){
            if(pcnt_isr_service_install(0) != ESP_OK){
                ESP_LOGE(TAG, "Encoder can't get installed on the same core...");
            } else {
                esp_err_t ipc_ret_code = ESP_FAIL;
                if(esp_ipc_call_blocking(isrServiceCpuCore, ipc_install_isr_on_core, &ipc_ret_code) != ESP_OK){
                    ESP_LOGE(TAG, "IPC Call to install isr service on core %d failed", isrServiceCpuCore);
                }
                if(ipc_ret_code != ESP_OK){
                    ESP_LOGE(TAG, "Encoder install isr service on core %d failed", isrServiceCpuCore);
                }
            }
            attached_interrupt = true;
        }
    }

    if(pcnt_isr_handler_add(unit, encoder_pcnt_intr_handler, this) != ESP_OK){
        ESP_LOGE(TAG, "Encoder install interrupt handler for unit %d failed", isrServiceCpuCore);
    }

    pcnt_counter_clear(unit);
    pcnt_intr_enable(unit);
    pcnt_counter_resume(unit);

    attached = true;
}

int64_t Encoder::read(){
    _ENTER_CRITICAL();
    int64_t result = count + get_raw_count();
    _EXIT_CRITICAL();
    return result;
}

int64_t Encoder::read_and_clear(){
    _ENTER_CRITICAL();
    int64_t temp = count + get_raw_count();
    pcnt_counter_clear(unit);
    count = 0;
    _EXIT_CRITICAL();
    return temp;
}

int64_t Encoder::pause(){
    return pcnt_counter_pause(unit);
}

int64_t Encoder::resume(){
    return pcnt_counter_resume(unit);
}

int64_t Encoder::get_raw_count(){
    int16_t c;
    int64_t compensate;
    _ENTER_CRITICAL();
    pcnt_get_counter_value(unit, &c);

    //Check if counter is overflowed, if so re-read and compensate
    //see https://github.com/espressif/esp-idf/blob/v4.4.1/tools/unit-test-app/components/test_utils/ref_clock_impl_rmt_pcnt.c#L168-L172
    // if (PCNT.int_st.val & BIT(unit)) {
    //     pcnt_get_counter_value(unit, &c);
	// 	if(PCNT.status_unit[unit].COUNTER_H_LIM){
	// 		compensate = enc_config.counter_h_lim;
	// 	} else if (PCNT.status_unit[unit].COUNTER_L_LIM) {
	// 		compensate = enc_config.counter_l_lim;
	// 	}
	// }

	_EXIT_CRITICAL();
	// return compensate + c;
    return c;
}

int64_t Encoder::clear(){
    _ENTER_CRITICAL();
    count = 0;
    _EXIT_CRITICAL();
    return pcnt_counter_clear(unit);
}

void Encoder::attach_both_edge(int pin_enc, int pin_dir){
    attach(pin_enc, pin_dir, both_edge);
    pcnt_counter_clear(unit);
}

void Encoder::attach_single_edge(int pin_enc, int pin_dir){
    attach(pin_enc, pin_dir, single_edge);
    pcnt_counter_clear(unit);
}

void Encoder::detach(){
    pcnt_counter_pause(unit);
    pcnt_isr_handler_remove(this->enc_config.unit);
    Encoder::encoders[unit]==NULL;
}

void Encoder::set_filter(uint16_t value){
    if(value > 1023) value = 1023;
    if(value == 0){
        pcnt_filter_disable(unit);
    } else {
        pcnt_set_filter_value(unit, value);
        pcnt_filter_enable(unit);
    }
}