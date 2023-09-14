/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_spi.h"
#include "app_pwm.h"
#include "nrf_delay.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "BEGAL_BLE"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                32                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};




volatile bool BLEData_ready= false, stream_flag = false;
const uint8_t start_bit=0xA0, end_bit=0xC0, no_data=0x00;
volatile uint8_t sample_no ;

volatile char bci_input[10];
volatile uint8_t input_count = 0;




/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


volatile uint8_t BLEconnected = 0;
volatile uint8_t BLEadvertisingstarted = 0;



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            BLEconnected = 1;
            BLEadvertisingstarted = 0;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            BLEconnected = 0;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);

        }
                
          BLEconnected = 0;
         break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            BLEconnected = 0;
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            BLEconnected = 0;
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            BLEconnected = 0;
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            BLEconnected = 0;
            break;

        default:
            // No implementation needed.
           // BLEconnected = 0;
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            bci_input[input_count] = data_array[index];
            input_count++;
              index++;

            

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/*************ADDING SPI CODE*****************************/

#define NRF_DRV_SPI_DEFAULT_CONFIG1                           \
{                                                            \
    .sck_pin      = NRF_DRV_SPI_PIN_NOT_USED,                \
    .mosi_pin     = NRF_DRV_SPI_PIN_NOT_USED,                \
    .miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,                \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_SPI_FREQ_2M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}



#ifndef SPI_SCK_PIN
#define SPI_SCK_PIN 26
#endif
#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN 30
#endif
#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN 29
#endif
#ifndef SPI_SS_PIN
#define SPI_SS_PIN 31
#endif
#ifndef SPI_IRQ_PRIORITY
#define SPI_IRQ_PRIORITY 6
#endif

//#define SPI_INSTANCE  0 /**< SPI instance index. */
//static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);  /**< SPI instance. */

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");

}

/**************************************************************/

/*************ADDING mcp3912 CODE*****************************/

#define MCP3912_CH0     0x00
#define MCP3912_CH1     0x01
#define MCP3912_CH2     0x02
#define MCP3912_CH3     0x03
#define MCP3912_MOD     0x08
#define MCP3912_PHASE   0x0A
#define MCP3912_GAIN    0x0B
#define MCP3912_STATCOM 0x0C
#define MCP3912_CONFIG0 0x0D
#define MCP3912_CONFIG1 0x0E
#define MCP3912_OFF0    0x0F
#define MCP3912_GC0     0x10
#define MCP3912_OFF1    0x11
#define MCP3912_GC1     0x12
#define MCP3912_OFF2    0x13
#define MCP3912_GC2     0x14
#define MCP3912_OFF3    0x15
#define MCP3912_GC3     0x16
#define MCP3912_LOCK    0x1F



#define No_Of_SAMPLES 248

#define ADC_CHANNELS                4
#define ADC_BUF24_SIZE             ( No_Of_SAMPLES+1)                           
#define ADC_BUF8_SIZE              (ADC_BUF24_SIZE * 3) 



long adc_data_buf24[ADC_CHANNELS][ADC_BUF24_SIZE];
uint8_t adc_data_buf8[ADC_BUF8_SIZE];

uint16_t ADC_BUF24_COUNT = 0;
uint16_t BLE_BUF_COUNT = 0;
uint8_t  ADC_SAMPLING_COMPLETE = 0;
uint8_t  BLE_BUFF_READY = 0;

uint16_t lenght;
uint16_t start;



void mcp3912_write_reg(uint8_t reg, long val24)
{
  uint8_t tx_data[4];
  uint8_t rx_data[4];
  
  
  
  tx_data[0] = ((0b01)<<6) | (reg<<1) | 0;
  tx_data[1] = ((val24>>16)&0xFF);
  tx_data[2] = ((val24>>8)&0xFF);
  tx_data[3] = (val24&0xFF);
  memset(rx_data, 0, 4);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, rx_data, 4));
    while (!spi_xfer_done)
        {
            __WFE();
        }

  }

  long mcp3912_read_reg(uint8_t reg)
{
  uint8_t tx_data[4];
  uint8_t rx_data[4];
  
  memset(tx_data, 0, 4);

  tx_data[0] =  (reg<<1);
  tx_data[0] |=  0b01000001;

  memset(rx_data, 0, 4);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data, 4, rx_data, 4));
   while (!spi_xfer_done)
        {
            __WFE();
        }
  return ( (rx_data[1]<<16) | (rx_data[2]<<8) |rx_data[3]  );
}


void mcp3912_init ()
{
  //nrf_gpio_pin_set(CS_PIN);
  uint32_t statcom, conf0, conf1, lock;
  uint8_t dtst;
  printf("inside the mcp3912 init\n\n");
 

  mcp3912_write_reg(MCP3912_STATCOM, statcom);
 

  conf0 = 0b000000110000000000000000; 
  mcp3912_write_reg(MCP3912_CONFIG0, conf0);
 

 
  conf1 = 0;
  lock = 0xA5<<16;
  //UINT8 *vp = (UINT8 *)&statcom;
  //a[0] = statcom[0];
  //a[1] = statcom[1];
  //a[2] = statcom[2];
  //a[3] = statcom[3];
  // SPI.transfer(statcom);
  //SPI.transfer((statcom>>16)&0xFF);
  //SPI.transfer((statcom>>8)&0xFF);
  //SPI.transfer(statcom&0xFF);

  //mcp3912_write_reg(MCP3912_LOCK, lock);
  mcp3912_write_reg(MCP3912_CONFIG1, conf1);
}


uint8_t mcp_data_ready()
 {
  long retval =  mcp3912_read_reg(MCP3912_STATCOM);
  nrf_delay_us(50);
  long str = retval & 0x0000000f;
  return (str == 0);
 }

 void mcp3912_sleep ()
{
  uint32_t conf1;
  conf1 =(0b00000000 << 16) | (0b00001111 << 8) | 0x00;
  mcp3912_write_reg(MCP3912_CONFIG1, conf1);

}

uint8_t read_adc_data()
  {
  //uint8_t adc_data_buf8[ADC_BUF8_SIZE];


    if(mcp_data_ready())
      {
      adc_data_buf24[0][ADC_BUF24_COUNT] = mcp3912_read_reg(MCP3912_CH0);
      
      adc_data_buf24[1][ADC_BUF24_COUNT] = mcp3912_read_reg(MCP3912_CH1);
      
      adc_data_buf24[2][ADC_BUF24_COUNT] = mcp3912_read_reg(MCP3912_CH2);
      
      adc_data_buf24[3][ADC_BUF24_COUNT] = mcp3912_read_reg(MCP3912_CH3);
      
      ADC_BUF24_COUNT++;

      if(ADC_BUF24_COUNT == No_Of_SAMPLES)
        {
        ADC_SAMPLING_COMPLETE = 1;
        }

      return 1;
      }
    
    return 0;
    }


    void Update_ble_buffer()
  {
  uint16_t i,j = 0;



for(i=0;i<No_Of_SAMPLES;i++)
  {
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[0][i] >> 16 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[0][i] >> 8 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[0][i]  ) & 0xff;
  j++;


    adc_data_buf8[j] = (uint8_t)( adc_data_buf24[1][i] >> 16 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[1][i] >> 8 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[1][i]  ) & 0xff;
  j++;


    adc_data_buf8[j] = (uint8_t)( adc_data_buf24[2][i] >> 16 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[2][i] >> 8 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[2][i]  ) & 0xff;
  j++;


    adc_data_buf8[j] = (uint8_t)( adc_data_buf24[3][i] >> 16 ) & 0xff;
  j++;
  adc_data_buf8[j] = (uint8_t)( adc_data_buf24[3][i] >> 8 ) & 0xff;
  j++;
  adc_data_buf8[j] =(uint8_t) ( adc_data_buf24[3][i]  ) & 0xff;
  j++;

  }


  ADC_BUF24_COUNT =0;
  ADC_SAMPLING_COMPLETE = 0;
  BLE_BUFF_READY =1;
  BLE_BUF_COUNT = j;
  }


/**********************************************************/


/*********************pwm generation using ppi*************/

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.


void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

void ppi_pwm_generation_init()
  {
  //nrf_gpio_cfg_output(13);
 //   NRF_CLOCK->TASKS_LFCLKSTART = 1;
 //   while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
 //    {
 //   //Wait for HFCLK to start
 //    }
 //   NRF_CLOCK->EVENTS_LFCLKSTARTED = 0; //Clear event
 ////Configure GPIOTE to toggle pin 13 
 //   NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
 //                        GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
 //                        13 << GPIOTE_CONFIG_PSEL_Pos | 
 //                        GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;
 ////Configure timer
 //   NRF_TIMER1->PRESCALER = 0;
 //   NRF_TIMER1->CC[0] = 2;  // Adjust the output frequency by adjusting the CC.
 //   NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
 //   NRF_TIMER1->TASKS_START = 1;
 ////Configure PPI
 //NRF_PPI->CH[0].EEP = (uint32_t) &NRF_TIMER1->EVENTS_COMPARE[0];
 //NRF_PPI->CH[0].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[0];
  //NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos;
  }


void pwm_generation_init(){

    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(4, BSP_LED_0);
    
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_LOW;
    /* Initialize and enable PWM. */
    APP_ERROR_CHECK(app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback));
   // APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
    while (app_pwm_channel_duty_set(&PWM1, 0, 50) == NRF_ERROR_BUSY);


}

/**********************************************************/



/**@brief Function for handling the idle state ( loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}



/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{

    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
 
}



/**@brief Function for initializing the nrf log module.
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
  ret_code_t err_code;

  switch(pin_no)
  {
   case BUTTON_2:
     if(button_action == APP_BUTTON_PUSH) {
       
        //#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("button2 pressed.");
        //#endif
     }
     else if(button_action == APP_BUTTON_RELEASE) {
        //#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("button2 released.");
        //#endif
        //mcp3912_sleep();
        //nrf_delay_ms(1000);
        sleep_mode_enter();
     }
     break;
     default:
        return; // no implementation needed
  }

}


static const app_button_cfg_t app_buttons[1] = 
{
// BUTTONS_NUMBER,
// BUTTON_PIN,
// BUTTONS_ACTIVE_STATE,
// BUTTON_PULL,
// are all declared in your components/boards/Ppca10056.h file

    {BUTTON_2, BUTTONS_ACTIVE_STATE, BUTTON_PULL, button_event_handler}
}; 

void buttons_init()
{

    ret_code_t  err_code;                        
// BUTTONS_NUMBER  is declared in your components/boards/Ppca10056.h file
    err_code = app_button_init((app_button_cfg_t *)app_buttons, 1, BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

}


uint8_t Send_data(void)
  {
     
     app_uart_put(start_bit);
     app_uart_put(sample_no);
     
     app_uart_put(adc_data_buf8[0]);
     app_uart_put(adc_data_buf8[1]);
     app_uart_put(adc_data_buf8[2]);
     
     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);

     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);

     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);

     
     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);

     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);

     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);

     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);


      
     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);
     app_uart_put(0);
     
     app_uart_put(end_bit);
 
     if(sample_no <255)
     sample_no ++;
     else
     sample_no =1;
     return 1;
      
  }


  void UART_IP_READ (void)
{
    if(bci_input[0] == 'v')
    {
        printf("0xA0");
        printf("0xB0");
        printf("$$$");
        input_count = 0;
    }
    else if (bci_input[0] == 'd')
    {
        printf("updating channel settings to default$$$");
       
        input_count = 0;
    }
    else if(bci_input[0] == 'c')
    {
        printf("no daisy to attach!8$$$");
       
        input_count = 0;
    }
    if(bci_input[0] == 'b')
    {
        stream_flag = true;
       
        input_count = 0;
      
    }   
    else if(bci_input[0] == 's')
    {
        stream_flag = false;
        sample_no =0;
        input_count = 0;
        
    }
    else if(bci_input[0] == 'x')
    {
            printf("Success: Channel set for "); // + rx_buff[1] + "$$$";
            printf("%c",bci_input[1] - 48); 
            printf("$$$");
           input_count = 0;
        
        
    }   
    
}



/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
  // Initialize.
    uart_init();
    log_init();
    timers_init();
    buttons_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    conn_params_init();
    advertising_init();

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG1;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    // Start execution.

    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();
    BLEadvertisingstarted = 1;
    BLEconnected = 0;
    pwm_generation_init();
    mcp3912_init();  
    printf("\nmcp3912 INIT over\n\n");

    input_count = 0;
    
 //while(1)
 //   {
 //   //adc_data_buf24[0][ADC_BUF24_COUNT] = mcp3912_read_reg(MCP3912_CH0);
 //   printf("%ld\n",mcp3912_read_reg(MCP3912_CH0));
 //   nrf_delay_ms(4);
 //   }



    //while(1)
    //  {

    //  if(stream_flag == true)
    //    {
       
    //    if(mcp_data_ready())
    //    {
    //    adc_data_buf24[0][0] = mcp3912_read_reg(MCP3912_CH0);
    //    //adc_data_buf24[0][0] = adc_data_buf24[0][0] / 10;
    //    int32_t signed_adc_data =  adc_data_buf24[0][0] - 3000000;

    //    signed_adc_data =   signed_adc_data / 10;

    //    adc_data_buf8[0] = ( signed_adc_data >> 16 ) & 0xff;
    //    adc_data_buf8[1] = ( signed_adc_data >> 8 ) & 0xff;
    //    adc_data_buf8[2] =( signed_adc_data  ) & 0xff;
    //    Send_data();
    //    // nrf_delay_ms(100);
    //    }
       
    //   }
      
    //  if(input_count > 0 )
    //    {
    //    nrf_delay_ms(10);
    //    UART_IP_READ();
    //    }
      
    //  }
    //while(1)
    //{
    ////adc_data_buf24[0][ADC_BUF24_COUNT] = mcp3912_read_reg(MCP3912_CH0);
    //printf("%ld\n",mcp3912_read_reg(MCP3912_CH0));
    //nrf_delay_ms(4);
    //}

    
    //Enter main loop.
    //while(1)
    //  {
    //  idle_state_handle();
    //  }
    BLEadvertisingstarted = 1;
    for (;;)
    {
        // idle_state_handle();
        if(BLEconnected==0)
          {
          if(BLEadvertisingstarted == 0)
            {
             printf("RESET NRF\n");
             NVIC_SystemReset();
             while(1);
            }
          }

       if(BLEconnected==1)
         {
         BLEadvertisingstarted = 0;
        if( read_adc_data()  )
        {
        if(ADC_SAMPLING_COMPLETE)
          {
          //future filter
          Update_ble_buffer();
          start = 0;
          }
        else if(BLE_BUFF_READY)
          {
          if(BLE_BUF_COUNT >= 240)
            { 
            lenght = 240;
            uint32_t err_code = ble_nus_data_send(&m_nus, &adc_data_buf8[start], &lenght , m_conn_handle);    
            //printf("BLENUS1:%d,%d,%d,%d\n",err_code,BLE_BUF_COUNT,start,lenght);
            //nrf_delay_us(2000);
            if(err_code == 0)
              {
              BLE_BUF_COUNT = BLE_BUF_COUNT - 240;
              start = start + 240;
              }
            }
          else if(BLE_BUF_COUNT > 0 )
            {
            lenght = BLE_BUF_COUNT;
            uint32_t err_code1 = ble_nus_data_send(&m_nus, &adc_data_buf8[start], &lenght , m_conn_handle);    
            //printf("BLENUS2:%d,%d,%d,%d\n",err_code1,BLE_BUF_COUNT,start,lenght);
            //nrf_delay_us(2000);
            if(err_code1 == 0)
              {
               BLE_BUF_COUNT = 0;
               BLE_BUFF_READY =0;
              //while(1);
              }
            }
          else
            { 
            BLE_BUF_COUNT = 0;
            BLE_BUFF_READY =0;
            }
          }
      } //if( read_adc_data())
    }//if(BLEconnected==1)
    }
  }


/**
 * @}
 */
