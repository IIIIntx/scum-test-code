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

// If true, sweep through all fine codes.
#define BLE_TX_SWEEP_FINE true
//  #define BLE_TX_SWEEP_FINE false
#define BLE_CALIBRATE_LC true

// BLE TX period in milliseconds.
#define BLE_TX_PERIOD_MS 1000  // milliseconds

#define BLE_SEND_CHANNEL 0

// BLE TX tuning code.
static tuning_code_t g_ble_tx_tuning_code = {
    .coarse = 22,
    .mid = 28,
    .fine = 19,
};

// BLE TX trigger.
static bool g_ble_tx_trigger = true;

extern optical_vars_t optical_vars;

// Transmit BLE packets.
static inline void ble_tx_trigger(void) {
#if BLE_TX_SWEEP_FINE
    for (uint8_t tx_fine_code = TUNING_MIN_CODE;
         tx_fine_code <= TUNING_MAX_CODE; ++tx_fine_code) {
        g_ble_tx_tuning_code.fine = tx_fine_code;
        tuning_tune_radio(&g_ble_tx_tuning_code);
        printf("Transmitting BLE packet on %u.%u.%u.\n",
               g_ble_tx_tuning_code.coarse, g_ble_tx_tuning_code.mid,
               g_ble_tx_tuning_code.fine);

        // Wait for the frequency to settle.
        for (uint32_t t = 0; t < 5000; ++t);

        ble_transmit();
    }
#else   // !BLE_TX_SWEEP_FINE
    tuning_tune_radio(&g_ble_tx_tuning_code);
    printf("Transmitting BLE packet on %u.%u.%u.\n",
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

    analog_scan_chain_write();
    analog_scan_chain_load();

    crc_check();
    perform_calibration();

#if BLE_CALIBRATE_LC
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
    divProgram(480, 0, 0);

    // Configure coarse, mid, and fine codes for TX.
#if BLE_CALIBRATE_LC
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
            printf("Triggering BLE TX.\n");
            ble_tx_trigger();
            g_ble_tx_trigger = false;
            delay_milliseconds_asynchronous(BLE_TX_PERIOD_MS, 7);
        }
    }
}
