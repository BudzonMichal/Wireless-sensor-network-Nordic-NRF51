/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_temp_receiver_main main.c
 * @{
 * @ingroup ble_sdk_app_temp_receiver
 * @brief Main file for Wireless Sensors System Application for nRF51822 evaluation board
 *
 * This file contains the source code for a sample application using the advertising packets
 * to send data from sensor, for the nRF51822 evaluation board (PCA10001). This applications is
 * compatible with Android/iOS application nRF Temp 2.0
 */

#include <stdint.h>
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "led.h"
#include "battery.h"
#include "device_manager.h"
#include "app_gpiote.h"
#include "pstorage.h"
#include "temp.h"
#include "main.h"

static ble_gap_adv_params_t   m_adv_params;                                        /**< Parameters to be passed to the stack when starting advertising. */
static app_timer_id_t         m_update_timer_id;                                   /**< Battery timer. */
static bool                   m_memory_access_in_progress = false;                 /**< Flag to keep track of ongoing operations on persistent memory. */

static void                   ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void                   sys_evt_dispatch(uint32_t sys_evt);
static void                   advertising_init(void);                              /**< Advertising initialization function. */
static void                   update_advertising_packet(uint32_t temp,
                                                        uint8_t  battery_level);   /**< Update advertising packet data */

/*****************************************************************************
* Error Handling Functions
*****************************************************************************/


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    // This function should NOT be used in a final product.
    // It is intended STRICTLY for development/debugging purposes.
    // The flash write will happen EVEN if the radio is active, thus interrupting
    // any communication.
    // Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/*****************************************************************************
* Static Timeout Handling Functions
*****************************************************************************/

/**@brief Function for handling an advertising packet update timer timeout.
 *
 * @details This function will be called each time the advertise packet update timer expires.
 *          This function will update temperature and battery level measurement.
 *
 * @param[in] p_context Pointer used for passing some arbitrary information (context) from the
 *                      app_start_timer() call to the timeout handler.
 */
static void adv_packet_update_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    uint32_t temp;
    uint8_t  batt;

    if (take_temperature(&temp) != NRF_SUCCESS)
    {
        temp = FAULTY_TEMPERATURE+1;
    }

    if (take_battery_level(&batt) != NRF_SUCCESS)
    {
        batt = FAULTY_BATT_LEVEL;
    }

    update_advertising_packet(temp, batt);
}


/*****************************************************************************
* Static Initialization Functions
*****************************************************************************/

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_update_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                adv_packet_update_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t                   err_code;
    int32_t                    init_temp = FAULTY_TEMPERATURE; // send improper value indications for the first time
    uint8_t                    init_bl   = FAULTY_BATT_LEVEL;
    ble_advdata_t              advdata;
    ble_advdata_t              srdata;
    uint8_t                    flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_advdata_service_data_t service_data[SERVICES_COUNT];

    // preparing advertising services data
    service_data[0].data.size    = sizeof (init_temp);
    service_data[0].data.p_data  = (uint8_t *)&init_temp;
    service_data[0].service_uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;

    service_data[1].data.size    = sizeof (init_bl);
    service_data[1].data.p_data  = &init_bl;
    service_data[1].service_uuid = BLE_UUID_BATTERY_SERVICE;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof (advdata));

    advdata.name_type            = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance   = false;
    advdata.flags.size           = sizeof (flags);
    advdata.flags.p_data         = &flags;
    advdata.p_service_data_array = service_data;
    advdata.service_data_count   = SERVICES_COUNT;

    // Build and set scan respone data.
    memset(&srdata, 0, sizeof (srdata));

    srdata.p_service_data_array = service_data;
    srdata.service_data_count   = SERVICES_COUNT;


    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof (m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL; // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}


/**@brief Function for updating the Advertising Packets data.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when updating advertising.
 */
static void update_advertising_packet(uint32_t temp, uint8_t battery_level)
{
    uint32_t                   err_code;
    ble_advdata_t              advdata;
    ble_advdata_t              srdata;
    uint8_t                    flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_service_data_t service_data[SERVICES_COUNT];

    // preparing advertising services data
    service_data[0].data.size    = sizeof (temp);
    service_data[0].data.p_data  = (uint8_t *)(&temp);
    service_data[0].service_uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;

    service_data[1].data.size    = sizeof (battery_level);
    service_data[1].data.p_data  = &battery_level;
    service_data[1].service_uuid = BLE_UUID_BATTERY_SERVICE;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof (advdata));

    advdata.name_type            = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance   = false;
    advdata.flags.size           = sizeof (flags);
    advdata.flags.p_data         = &flags;
    advdata.p_service_data_array = service_data;
    advdata.service_data_count   = SERVICES_COUNT;

    // Build and set scan response data.
    memset(&srdata, 0, sizeof (srdata));

    srdata.p_service_data_array = service_data;
    srdata.service_data_count   = SERVICES_COUNT;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t        err_code;
    dm_init_param_t init_data;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof (ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GPIOTE module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/*****************************************************************************
* Static Start Functions
*****************************************************************************/

/**@brief Function for starting the application timers. Required by GPIOTE module.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers
    err_code = app_timer_start(m_update_timer_id, APP_TIMER_TICKS(UPDATE_PERIOD, 0), NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    led_start();
}


/**@brief Function for putting the chip in System OFF Mode
 */
static void system_off_mode_enter(void)
{
    uint32_t err_code;
    uint32_t count;

    // Verify if there is any flash access pending, if yes delay starting advertising until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/*****************************************************************************
* Static Event Handling Functions
*****************************************************************************/

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_TIMEOUT:

            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                led_stop();

                system_off_mode_enter();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in] sys_evt system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                system_off_mode_enter();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/*****************************************************************************
* Main Function
*****************************************************************************/

/**@brief Function for the application main entry.
 */
int main(void)
{
    uint32_t err_code;

    timers_init();
    gpiote_init();
    ble_stack_init();
    device_manager_init();
    battery_init();

    // Initialize Bluetooth Stack parameters.
    advertising_init();

    // Start advertising.
    advertising_start();

    // Enter main loop.
    application_timers_start();

    for (;;)
    {
        // Switch to a low power state until an event is available for the application
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


/**
 * @}
 */
