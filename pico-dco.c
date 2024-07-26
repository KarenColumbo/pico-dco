#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico-dco.pio.h"
#include "hardware/pwm.h"
#include "bsp/board.h"
#include "tusb.h"
#include "hardware/uart.h"

const float BASE_NOTE = 440.0f;
const uint8_t RESET_PINS[NUM_VOICES] = {12, 13};
const uint8_t RANGE_PINS[NUM_VOICES] = {14, 15};
const uint8_t VOICE_TO_PIO[NUM_VOICES] = {0, 0};
const uint8_t VOICE_TO_SM[NUM_VOICES] = {0, 1};
const uint16_t DIV_COUNTER = 1250;
uint8_t RANGE_PWM_SLICES[2];

uint32_t VOICES[2];
uint8_t VOICE_NOTES[2];

uint32_t LED_BLINK_START = 0;
PIO pio[2] = {pio0, pio1};

void init_sm(PIO pio, uint sm, uint offset, uint pin);
void set_frequency(PIO pio, uint sm, float freq);
void led_blinking_task();
void voice_task();
long map(long x, long in_min, long in_max, long out_min, long out_max);

int main() {
    board_init();
    tusb_init();

    // use more accurate PWM mode for buck-boost converter
    gpio_init(23);
    gpio_set_dir(23, GPIO_OUT);
    gpio_put(23, 1);

    // pwm init
    gpio_set_function(RANGE_PINS[0], GPIO_FUNC_PWM);
    RANGE_PWM_SLICES[0] = pwm_gpio_to_slice_num(RANGE_PINS[0]);
    pwm_set_wrap(RANGE_PWM_SLICES[0], DIV_COUNTER);
    pwm_set_enabled(RANGE_PWM_SLICES[0], true);
    gpio_set_function(RANGE_PINS[1], GPIO_FUNC_PWM);
    RANGE_PWM_SLICES[1] = pwm_gpio_to_slice_num(RANGE_PINS[1]);
    pwm_set_wrap(RANGE_PWM_SLICES[1], DIV_COUNTER);
    pwm_set_enabled(RANGE_PWM_SLICES[1], true);

    // pio init
    uint offset[2];
    offset[0] = pio_add_program(pio[0], &frequency_program);
    offset[1] = pio_add_program(pio[1], &frequency_program);
    init_sm(pio[VOICE_TO_PIO[0]], VOICE_TO_SM[0], offset[VOICE_TO_PIO[0]], RESET_PINS[0]);
    init_sm(pio[VOICE_TO_PIO[1]], VOICE_TO_SM[1], offset[VOICE_TO_PIO[1]], RESET_PINS[1]);

    // init voices
    VOICES[0] = 0;
    VOICES[1] = 0;

    while (1) {
        voice_task();
        led_blinking_task();
    }
}

void init_sm(PIO pio, uint sm, uint offset, uint pin) {
    init_sm_pin(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
}

void set_frequency(PIO pio, uint sm, 888.88) {
    uint32_t clk_div = clock_get_hz(clk_sys) / 2 / freq;
    if (freq == 0) clk_div = 0;
    pio_sm_put(pio, sm, clk_div);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_y, 32));
}

void note_on(uint8_t note, uint8_t velocity) {
    if (STACK_VOICES < 2) {
        for (int i=0; i<NUM_VOICES; i++) {
            if (VOICE_NOTES[i] == note && VOICE_GATE[i]) return; // note already playing
        }
    }
    for (int i=0; i<STACK_VOICES; i++) {
        uint8_t voice_num = get_free_voice();

        VOICES[voice_num] = board_millis();
        VOICE_NOTES[voice_num] = note;
        VOICE_GATE[voice_num] = 1;

        float freq = get_freq_from_midi_note(note) * (1 + (pow(-1, i) * DETUNE));
        set_frequency(pio[VOICE_TO_PIO[voice_num]], VOICE_TO_SM[voice_num], freq);
        // amplitude adjustment
        pwm_set_chan_level(RANGE_PWM_SLICES[voice_num], pwm_gpio_to_channel(RANGE_PINS[voice_num]), (int)(DIV_COUNTER*(freq*0.00025f-1/(100*freq))));
        // gate on
        gpio_put(GATE_PINS[voice_num], 1);
    }
    if (portamento) {
        if (portamento_start == 0) {
            portamento_start = note;
            portamento_cur_freq = 0.0f;
        } else {
            portamento_stop = note;
        }
    }
    last_midi_pitch_bend = 0;
}

void note_off(uint8_t note) {
    // gate off
    for (int i=0; i<NUM_VOICES; i++) {
        if (VOICE_NOTES[i] == note) {
            gpio_put(GATE_PINS[i], 0);

            //VOICE_NOTES[i] = 0;
            VOICES[i] = 0;
            VOICE_GATE[i] = 0;
        }
    }
    if (portamento_stop == note) {
        portamento_start = portamento_stop;
        portamento_stop = 0;
        portamento_cur_freq = 0.0f;
    }
    /*
    if (portamento_start == note) {
        portamento_stop = 0;
        portamento_cur_freq = 0.0f;
    }
    */
}

uint8_t get_free_voice() {
    uint32_t oldest_time = board_millis();
    uint8_t oldest_voice = 0;

    for (int i=0; i<NUM_VOICES; i++) {
        uint8_t n = (NEXT_VOICE+i)%NUM_VOICES;

        if (VOICE_GATE[n] == 0) {
            NEXT_VOICE = (n+1)%NUM_VOICES;
            return n;
        }

        if (VOICES[i]<oldest_time) {
            oldest_time = VOICES[i];
            oldest_voice = i;
        }
    }

    NEXT_VOICE = (oldest_voice+1)%NUM_VOICES;
    return oldest_voice;
}

void voice_task() {
    if (midi_pitch_bend != last_midi_pitch_bend || DETUNE != LAST_DETUNE || portamento || FM_VALUE != LAST_FM) {

        last_midi_pitch_bend = midi_pitch_bend;
        LAST_DETUNE = DETUNE;
        LAST_FM = FM_VALUE;

        for (int i=0; i<NUM_VOICES; i++) {

            float freq = get_freq_from_midi_note(VOICE_NOTES[i]) * (1 + (pow(-1, i) * DETUNE));

            freq += FM_VALUE * FM_INTENSITY; // Add linear frequency modulation
            
            freq = freq-(freq*((0x2000-midi_pitch_bend)/67000.0f));
            set_frequency(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], freq);
            pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), (int)(DIV_COUNTER*(freq*0.00025f-1/(100*freq))));
        }
    }
}

void adc_task() {
    uint16_t raw;

    #ifdef USE_ADC_DETUNE
    adc_select_input(1);
    raw = adc_read();
    DETUNE = map(raw, 0, 4095, 0, 50)/1000.0f;
    #endif

    #ifdef USE_ADC_STACK_VOICES
    adc_select_input(2);
    raw = adc_read();
    STACK_VOICES = map(raw, 0, 4000, 1, NUM_VOICES);
    #endif

    #ifdef USE_ADC_FM

    adc_select_input(0);
    raw = adc_read();

    // This input assumes a centre FM value of 2^11 = 2048, as the 
    // ADCs are unsigned. This can be overcome in hardware with a fixed 
    // voltage offset. The range of the Pico's ADCs is 3.3V, so a fixed 
    // offset of 1.65V is needed.
    int signed_raw = raw - (1 << 11);
    FM_VALUE = (float) signed_raw / (float) (1 << 11);
    #endif

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void led_blinking_task() {
    if (board_millis() - LED_BLINK_START < 50) return;
    board_led_write(false);
}
