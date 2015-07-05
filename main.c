#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "ble_srv_common.h"
#include "ble_hts.h"
#include "htu21.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "app_uart.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(10000, UNIT_0_625_MS)


#define MEASURE_INTERVAL                MSEC_TO_UNITS(20000, 100)

#define APP_TIMER_PRESCALER             0
#define APP_TIMER_MAX_TIMERS            (2+BSP_APP_TIMERS_NUMBER)
#define APP_TIMER_OP_QUEUE_SIZE         4


#define DEVICE_NAME                     "iX"
#define BLE_UUID_TEMPERATURE_SERVICE    0x1809
#define BLE_UUID_HUMIDITY_SERVICE       0x2A6F


#define TX_BUF_SIZE   256u
#define RX_BUF_SIZE   1u

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    for (;;) {
        bsp_indication_set(BSP_INDICATE_FATAL_ERROR);
        nrf_delay_ms(500);
        bsp_indication_set(BSP_INDICATE_IDLE);
        nrf_delay_ms(500);
    }
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void set_advertisment(int32_t temperature, uint8_t temperature_exponent, uint8_t battery, uint16_t humidity)
{

    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    // device name
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

    // battery level
    ble_advdata_service_data_t  srv_data[3];

    uint8_array_t               data_array;
    uint8_t                     data;
    data = 99;
    data_array.p_data        = &data;
    data_array.size          = 1;
    srv_data[0].service_uuid = BLE_UUID_BATTERY_SERVICE;
    srv_data[0].data         = data_array;


    // temperature
    uint8_array_t               data_array2;
    uint8_t                     data2[4];
    data2[0] = ((uint8_t *)&temperature)[0];
    data2[1] = ((uint8_t *)&temperature)[1];
    data2[2] = ((uint8_t *)&temperature)[2];
    data2[3] = temperature_exponent; //exponent
    data_array2.p_data = data2;
    data_array2.size = sizeof(data2);
    srv_data[1].service_uuid = BLE_UUID_TEMPERATURE_SERVICE;
    srv_data[1].data = data_array2;

    // humidity
    uint8_array_t               data_array3;
    uint8_t                     data3[2];
    data3[0] = ((uint8_t *)&humidity)[0];
    data3[1] = ((uint8_t *)&humidity)[1];
    data_array3.p_data        = data3;;
    data_array3.size          = 2;
    srv_data[2].service_uuid = BLE_UUID_HUMIDITY_SERVICE;
    srv_data[2].data         = data_array3;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.p_service_data_array    = srv_data;
    advdata.service_data_count      = 3;

    advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    advdata.flags                 = flags;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


void start_advertising()
{
    uint32_t err_code;
    ble_gap_adv_params_t m_adv_params;
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL; // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}

static void update_sensor(void *p)
{
    LEDS_INVERT(BSP_LED_5_MASK);
    int16_t temperature = -65;
    if (!htu21_read_temperature(&temperature)) {
        temperature = -66;
    }

    uint8_t humid = -2;
    if (!htu21_read_humidity(&humid)) {
        humid = -1;
    }
    set_advertisment(temperature, -2, 98, humid);
    LEDS_INVERT(BSP_LED_5_MASK);
}


static void ble_stack_init(void)
{
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack
    uint32_t err_code;
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


static app_timer_id_t  m_messure_timer_id;
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_messure_timer_id,
            APP_TIMER_MODE_REPEATED,
            update_sensor);
    APP_ERROR_CHECK(err_code);
}

static void timers_start(void)
{
    uint32_t err_code;
    err_code = app_timer_start(m_messure_timer_id, MEASURE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}



void uart_error_handle(app_uart_evt_t * p_event)
{
    return;
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

static void uart_init(void)
{
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        0,
        0,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    uint32_t err_code;
    APP_UART_FIFO_INIT(&comm_params,
            RX_BUF_SIZE,
            TX_BUF_SIZE,
            uart_error_handle,
            APP_IRQ_PRIORITY_LOW,
            err_code);

    APP_ERROR_CHECK(err_code);

    printf("\r\nbooting\r\n");

}



int main(void)
{
    uint32_t err_code;
    // Initialize.
    timers_init();
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

    uart_init();
    ble_stack_init();

    twi_master_init();

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    nrf_delay_ms(200);
    bsp_indication_set(BSP_INDICATE_IDLE);
    nrf_delay_ms(200);
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);

    update_sensor(0);
    start_advertising();
    timers_start();

    // Enter main loop.
    for (;; )
    {
        power_manage();
    }
}
