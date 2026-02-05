#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ble.h"
#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"
#include "tuning.h"


//=========================== defines =========================================

// If true, sweep through all fine codes.
#define BLE_TX_SWEEP_FINE true
//  #define BLE_TX_SWEEP_FINE false
#define BLE_CALIBRATE_LC true

// BLE TX period in milliseconds.
#define BLE_TX_PERIOD_MS    1000  // milliseconds
// Period for rf_timer meaure the LC count
#define SAMPLE_TIMER_PERIOD        500       ///< 500 = 1ms@500kHz

#define BLE_SEND_CHANNEL 0

#define NUM_SAMPLES 5
#define STEPS_PER_CONFIG 32

//=========================== variables =======================================

// BLE TX tuning code.
static tuning_code_t g_ble_tx_tuning_code = {
    .coarse = 22,
    .mid = 28,
    .fine = 19,
};

typedef struct {
    uint32_t avg_sample;
    uint32_t samples[NUM_SAMPLES];
    uint8_t sample_index;
    bool    count_sample_after_tx;
} app_vars_t;
static volatile app_vars_t app_vars;

// BLE TX trigger.
static bool g_ble_tx_trigger = true;


//=========================== prototypes ======================================
extern optical_vars_t optical_vars;
void   cb_timer(void);
uint32_t     average_sample(void);
void   count_sample_after_ble_tx(void);

//=========================== functions =======================================

// Transmit BLE packets.
static inline void ble_tx_trigger(void) {
    uint32_t delay;
#ifdef BLE_TX_SWEEP_FINE
    // app_vars.count_sample_after_tx = false;
    for (uint8_t tx_fine_code = TUNING_MIN_CODE;
         tx_fine_code <= TUNING_MAX_CODE; ++tx_fine_code) {
        gpio_7_toggle();
        app_vars.count_sample_after_tx = false;
        app_vars.sample_index = 0;
        g_ble_tx_tuning_code.fine = tx_fine_code;
        tuning_tune_radio(&g_ble_tx_tuning_code);
        printf("packet on %u.%u.%u.\n",
               g_ble_tx_tuning_code.coarse, g_ble_tx_tuning_code.mid,
               g_ble_tx_tuning_code.fine);
        ble_transmit();
        count_sample_after_ble_tx();
        // Wait for some time
        for (delay = 0; delay < 5000; ++delay);
        
    }
#else    // !BLE_TX_SWEEP_FINE
    tuning_tune_radio(&g_ble_tx_tuning_code);
    printf("Setting %u.%u.%u.\n",
           g_ble_tx_tuning_code.coarse, g_ble_tx_tuning_code.mid,
           g_ble_tx_tuning_code.fine);

    // Wait for frequency to settle.
    for (uint32_t t = 0; t < 5000; ++t);

    ble_transmit();
#endif  // BLE_TX_SWEEP_FINE
}

static void ble_tx_rftimer_callback(void) {
    // Trigger a BLE TX.
    g_ble_tx_trigger = true;
    // After BLE tx, begin LC count compensation
    
}

int main(void) {
    initialize_mote();

    // Initialize BLE TX.
    printf("Initializing BLE TX.\n");
    ble_init();
    ble_init_tx();
		//ble_set_channel(BLE_SEND_CHANNEL);
		//const char* my_data = "bcum";
    //ble_set_data((const uint8_t*)my_data);

    // Configure the RF timer.
    rftimer_set_callback_by_id(ble_tx_rftimer_callback, 7);
    rftimer_enable_interrupts();
    rftimer_enable_interrupts_by_id(7);

    // Try to configure one another RF timer =
    rftimer_set_callback_by_id(cb_timer, 6);
    rftimer_enable_interrupts();
    rftimer_enable_interrupts_by_id(6);

    analog_scan_chain_write();
    analog_scan_chain_load();

    crc_check();
    perform_calibration();

#ifdef BLE_CALIBRATE_LC
		optical_vars.optical_cal_finished = false;
    optical_enableLCCalibration();

    // Turn on LO, DIV, PA, and IF
    ANALOG_CFG_REG__10 = 0x78;

    // Turn off polyphase and disable mixer
    ANALOG_CFG_REG__16 = 0x6;

    // For TX, LC target freq = 2.402G - 0.25M = 2.40175 GHz.
    optical_setLCTarget(250182);
#endif

    // Enable optical SFD interrupt for optical calibration
    optical_enable();

    // Wait for optical cal to finish
    while (!optical_getCalibrationFinished());

    printf("Cal complete\r\n");

    // Disable static divider to save power
    // divProgram(480, 0, 0);

    // Configure coarse, mid, and fine codes for TX.
#ifdef BLE_CALIBRATE_LC
    g_ble_tx_tuning_code.coarse = optical_getLCCoarse();
    g_ble_tx_tuning_code.mid = optical_getLCMid();
    g_ble_tx_tuning_code.fine = optical_getLCFine();
#else
    // CHANGE THESE VALUES AFTER LC CALIBRATION.
    app_vars.tx_coarse = 22;
    app_vars.tx_mid = 28;
    app_vars.tx_fine = 15;
#endif
		
    // Generate a BLE packet.
    ble_generate_packet();
		//ble_generate_test_packet();

    while (true) {
        if (g_ble_tx_trigger) {
            printf("Triggering BLE TX.\r\n");
            ble_tx_trigger();
            g_ble_tx_trigger = false;
            delay_milliseconds_asynchronous(10000, 7);
        }
    }
}


uint32_t     average_sample(void){
    uint8_t i;
    uint32_t avg;
    
    avg = 0;
    for (i=0;i<NUM_SAMPLES;i++) {
        avg += app_vars.samples[i];
    }
    avg = avg/NUM_SAMPLES;
    return avg;
}

void    cb_timer(void) {
    
    uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;
   
    read_counters_3B(&count_2M,&count_LC,&count_adc);
    gpio_6_toggle();
    if (app_vars.sample_index < NUM_SAMPLES) {
        rftimer_setCompareIn_by_id(rftimer_readCounter()+SAMPLE_TIMER_PERIOD, 6);
        app_vars.samples[app_vars.sample_index] = count_LC;
        app_vars.sample_index++;
    } else {
        app_vars.count_sample_after_tx = true;
    }
}

void    count_sample_after_ble_tx(void) {
    app_vars.avg_sample = average_sample();
    gpio_5_toggle();

    // Enable all counters
    ANALOG_CFG_REG__0 = 0x3FFF;

    rftimer_setCompareIn_by_id(rftimer_readCounter() + SAMPLE_TIMER_PERIOD, 6);
    while (app_vars.count_sample_after_tx==false);
    
    // Disable all counters
    ANALOG_CFG_REG__0 = 0x007F;
    // Reset all counters
    ANALOG_CFG_REG__0 = 0x0000;

    printf(
        "->%d\r\n",
        app_vars.avg_sample
    );
}