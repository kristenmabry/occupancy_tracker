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
 * @defgroup ble_sdk_app_hts_main main.c
 * @{
 * @ingroup ble_sdk_app_hts
 * @brief Health Thermometer Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Health Thermometer service
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 *
 *
 *
 *
 *
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "vl53l5cx_api.h"

#include "vl53l5cx_plugin_detection_thresholds.h"
#include "vl53l5cx_plugin_motion_indicator.h"

#include <nrfx_saadc.h>
#include "core_cm4.h"
#include "nrf_clock.h"
#include "nrf_rtc.h"
#include "bsp.h"
#include "nrf52.h"
//#include <nrfx_saadc_v2.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "bsp.h"

#include "nrf_delay.h"

#include "nrf_clock.h"
#include "nrf_rtc.h"
#include "nrf_gpio.h"
#include "ble_cus.h"
#include "nrf_ble_bms.h"

#include <nrf_sdm.h>


#include "nrf52.h"
#include "core_cm4.h"



#define DEVICE_NAME                     "Lidar"                                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "NS-HTS-EXAMPLE"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 0x1122334455                                /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788                                    /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                40                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(500)                       /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define TEMP_TYPE_AS_CHARACTERISTIC     0                                           /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

#define MIN_CELCIUS_DEGREES             3688                                        /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS             3972                                        /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT       36                                          /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define READ_VL53L5CX_INTERVAL          APP_TIMER_TICKS(100)                        /**< Perform VL53L5CX read and update occupancy variable - 100ms*/
#define SYSTEM_RESET_INTERVAL           APP_TIMER_TICKS(1000)                       // Attempt system reset after 1000ms of inaction in ranging loop
#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(10000)  

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define TWI_INSTANCE_ID     0     // twi instance
#define CEILING_HEIGHT      2100  // height from sensor to floor
#define RANGING_FREQUENCY   10     // frequency (Hz) of new ranging data (1-15 for 8x8) (1-60 for 4x4)
#define MOTION_MINIMUM      400   // minimum distance for motion indication (at least <400mm && 1500mm from maximum)
#define MOTION_MAXIMUM      1800  // maximum distance for motion indication (max 4000mm && 1500mm from minimum
#define PERSON_MIN_HEIGHT   1500  // minimum height to increase occupancy
#define INTEGRATION_TIME    10    // 
#define MEM_BUFF_SIZE                   512

APP_TIMER_DEF(m_notification_timer_id);                                                  /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                                 /**< Structure used to identify the battery service. */
BLE_HTS_DEF(m_hts);                                                                 /**< Structure used to identify the health thermometer service. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
APP_TIMER_DEF(m_vl53l5cx_timer_id);
BLE_CUS_DEF(m_cus);                                                             /**< Context for the Queued Write module.*/
APP_TIMER_DEF(m_system_reset_timer_id);                                              // timeout
NRF_BLE_BMS_DEF(m_bms);                                                         //!< Structure used to identify the Bond Management service.
//APP_TIMER_DEF(m_notification_timer_id);


static uint16_t          m_conn_handle = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static bool              m_hts_meas_ind_conf_pending = false;                       /**< Flag to keep track of when an indication confirmation is pending. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_temp_celcius_sim_cfg;                                    /**< Temperature simulator configuration. */
static sensorsim_state_t m_temp_celcius_sim_state;                                  /**< Temperature simulator state. */
static ble_conn_state_user_flag_id_t m_bms_bonds_to_delete;                     //!< Flags used to identify bonds that should be deleted.
static uint8_t                       m_qwr_mem[MEM_BUFF_SIZE];      

static ble_uuid_t m_adv_uuids[] =                                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_BMS_SERVICE,  BLE_UUID_TYPE_BLE},
    {CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

static uint8_t count, isReady;
static uint16_t ceiling_height = CEILING_HEIGHT;  // 16 since 4m is 12bits, initially default but can be changed by user over ble

static uint8_t person_entry = 0, check = 1, person_exit = 0, person_entry_2, person_exit_2;

static void advertising_start(bool erase_bonds);
//static void temperature_measurement_send(void);

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);  // give device twi id

static  VL53L5CX_Configuration sensor_config = {
      /* Platform, filled by customer into the 'platform.h' file */
        .platform = {
          .address = 0x29,
          .m_twi = m_twi,
        },
	/* Results streamcount, value auto-incremented at each range */
	//.streamcount = 0,
	/* Size of data read though I2C */
	//uint32_t	        data_read_size;
	/* Offset buffer */
	//uint8_t		        offset_data[VL53L5CX_OFFSET_BUFFER_SIZE];
	/* Xtalk buffer */
	//uint8_t		        xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];
	/* Temporary buffer used for internal driver processing */
	 //uint8_t	        temp_buffer[VL53L5CX_TEMPORARY_BUFFER_SIZE];
  };

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    bool       is_indication_enabled;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            //// Send a single temperature measurement if indication is enabled.
            //// NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            //// pm_evt_handler() in ble_evt_dispatch().
            //err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            //APP_ERROR_CHECK(err_code);
            //if (is_indication_enabled)
            //{
                //temperature_measurement_send();
            //}
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

/** @brief Attempts to read new Vl53l5cx data, then check if occupancy should change 
*
*   @param
*/
void read_lidar_data() {
  VL53L5CX_ResultsData 	Results;

  vl53l5cx_check_data_ready(&sensor_config, &isReady);
  if(isReady == 1) 
  {
    vl53l5cx_get_ranging_data(&sensor_config, &Results);

    /* As the sensor is set in 4x4 mode by default, we have a total
     * of 16 zones to print. For this example, only the data of first zone are
     * print */
    // NRF_LOG_INFO("Print data no : %3u\n", sensor_config.streamcount);
    //uint8_t i;
    //for(i = 0; i < 16; i++)
    //{                  //"Zone : %3d, Status : %3u, Distance : %4d mm\n" Results.target_status
    //                   //"Zone : %3d, Motion : %3d, Distance : %4d mm\n" Results.motion_indicator.motion
    //        NRF_LOG_INFO("Zone : %3d, Status : %3u, Distance : %4d mm\n",
    //                i,
    //                 Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
    //                (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]));
    //}

    /* Update local Ceiling height to same as bluetooth*/
    ceiling_height = m_cus.current_value_2;

    /* Entry - not pin side to pin side */
    // person just in area 1
    if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) > PERSON_MIN_HEIGHT || 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) > PERSON_MIN_HEIGHT)
      { // not in area 3
        if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT)
          {
            person_entry = 1;
          }
      }
    // person just in area 3                       
    if(((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) > PERSON_MIN_HEIGHT || 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) > PERSON_MIN_HEIGHT)
      && person_entry == 1) 
      { // not in area 1 or 2
        if( /* 1 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT) &&

           /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

          ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
          {
            person_exit = 1;
          }
      }

    // No person present - clear flags
    if(/* 3 */ 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT && 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT &&

      /* 1 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT) &&

       /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

      ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
      {
        if(person_exit){
         m_cus.current_value++; // person passed through and area is clear
         ble_cus_custom_value_update(&m_cus, m_cus.current_value);
         }
        person_entry = 0;
        person_exit = 0;
      }

      /* Exit - pin side to non pin side */
    // person just in area 3
    if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) > PERSON_MIN_HEIGHT || 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) > PERSON_MIN_HEIGHT)
      { // not in area 1
        if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT)
          {
            person_entry_2 = 1;
          }
      }
    // person just in area 1                       
    if(((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) > PERSON_MIN_HEIGHT || 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) > PERSON_MIN_HEIGHT ||
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) > PERSON_MIN_HEIGHT)
      && person_entry_2 == 1) 
      { // not in area 3 or 2
        if( /* 3 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT) &&

           /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

          ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
          {
            person_exit_2 = 1;
          }
      }

    // No person present - clear flags
    if(/* 3 */ 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT && 
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT &&

      /* 1 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT) &&

       /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

      ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
      (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
      {
        if(person_exit_2) {
         m_cus.current_value--; // person passed through and area is clear
         ble_cus_custom_value_update(&m_cus, m_cus.current_value);
        person_entry_2 = 0;
        person_exit_2 = 0;

        }
      }
  
  }
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    //uint8_t  battery_level;

    //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_cus_custom_value_update(&m_cus, m_cus.current_value);
    //err_code = ble_bas_battery_level_update(&m_bas, count, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}



/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();

}

static void occupancy_meas_timeout_handler()
{}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the vl53l5cx_read timer expires
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void vl53l5cx_read_handler(void * p_context)
{
    read_lidar_data();

}

/**@brief Function for handling the system reset after inaction for a certain time
 *
 * @details This function will be called when system_reset timer expires
 *
 *
 */
static void reset_system_handler()
{
  sd_nvic_SystemReset();


}

/**@brief Function for populating simulated health thermometer measurement.
 */
static void hts_sim_measurement(ble_hts_meas_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };

    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

    celciusX100 = sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    // update simulated time stamp
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    // Increment the value of m_custom_value before nortifing it.
    
    
    err_code = ble_cus_custom_value_update(&m_cus, m_cus.current_value);
    //err_code = ble_cus_ceiling_value_update(&m_cus, m_cus.current_value_2);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    //err_code = app_timer_create(&m_battery_timer_id,
    //                            APP_TIMER_MODE_REPEATED,
    //                            battery_level_meas_timeout_handler);
    //APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_notification_timer_id, APP_TIMER_MODE_REPEATED, notification_timeout_handler);


    err_code = app_timer_create(&m_vl53l5cx_timer_id, APP_TIMER_MODE_REPEATED, vl53l5cx_read_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_system_reset_timer_id, APP_TIMER_MODE_SINGLE_SHOT, reset_system_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                           strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from bond management service. from @main.c in nrf_ble_bms
 */
void bms_evt_handler(nrf_ble_bms_t * p_ess, nrf_ble_bms_evt_t * p_evt)
{
    ret_code_t err_code;
    bool is_authorized = true;

    switch (p_evt->evt_type)
    {
        case NRF_BLE_BMS_EVT_AUTH:
            NRF_LOG_DEBUG("Authorization request.");
#if USE_AUTHORIZATION_CODE
            if ((p_evt->auth_code.len != m_auth_code_len) ||
                (memcmp(m_auth_code, p_evt->auth_code.code, m_auth_code_len) != 0))
            {
                is_authorized = false;
            }
#endif
            err_code = nrf_ble_bms_auth_response(&m_bms, is_authorized);
            APP_ERROR_CHECK(err_code);
    }
}





/**@brief Function for simulating and sending one Temperature Measurement.
 */
//static void temperature_measurement_send(void)
//{
//    ble_hts_meas_t simulated_meas;
//    ret_code_t     err_code;

//    if (!m_hts_meas_ind_conf_pending)
//    {
//        hts_sim_measurement(&simulated_meas);

//        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);

//        switch (err_code)
//        {
//            case NRF_SUCCESS:
//                // Measurement was successfully sent, wait for confirmation.
//                m_hts_meas_ind_conf_pending = true;
//                break;

//            case NRF_ERROR_INVALID_STATE:
//                // Ignore error.
//                break;

//            default:
//                APP_ERROR_HANDLER(err_code);
//                break;
//        }
//    }
//}


/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in] p_hts  Health Thermometer Service structure.
 * @param[in] p_evt  Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            //temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
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

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_cus_evt(ble_cus_t     * p_cus_service,
                       ble_cus_evt_t * p_evt)
{
    ret_code_t err_code;
    
    switch(p_evt->evt_type)
    {
        case BLE_CUS_EVT_NOTIFICATION_ENABLED:
            
             err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
             APP_ERROR_CHECK(err_code);
             break;

        case BLE_CUS_EVT_NOTIFICATION_DISABLED:

            err_code = app_timer_stop(m_notification_timer_id);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CUS_EVT_CONNECTED:
            break;

        case BLE_CUS_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for deleting a single bond if it does not belong to a connected peer.
 *
 * This will mark the bond for deferred deletion if the peer is connected.
 */
static void bond_delete(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t   err_code;
    pm_peer_id_t peer_id;

    if (ble_conn_state_status(conn_handle) == BLE_CONN_STATUS_CONNECTED)
    {
        ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, true);
    }
    else
    {
        NRF_LOG_DEBUG("Attempting to delete bond.");
        err_code = pm_peer_id_get(conn_handle, &peer_id);
        APP_ERROR_CHECK(err_code);
        if (peer_id != PM_PEER_ID_INVALID)
        {
            err_code = pm_peer_delete(peer_id);
            APP_ERROR_CHECK(err_code);
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
        }
    }
}


/**@brief Function for performing deferred deletions.
*/
static void delete_disconnected_bonds(void)
{
    uint32_t n_calls = ble_conn_state_for_each_set_user_flag(m_bms_bonds_to_delete, bond_delete, NULL);
    UNUSED_RETURN_VALUE(n_calls);
}


/**@brief Function for marking the requester's bond for deletion.
*/
static void delete_requesting_bond(nrf_ble_bms_t const * p_bms)
{
    NRF_LOG_INFO("Client requested that bond to current device deleted");
    ble_conn_state_user_flag_set(p_bms->conn_handle, m_bms_bonds_to_delete, true);
}


/**@brief Function for deleting all bonds
*/
static void delete_all_bonds(nrf_ble_bms_t const * p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        bond_delete(conn_handle, NULL);

        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Function for deleting all bet requesting device bonds
*/
static void delete_all_except_requesting_bond(nrf_ble_bms_t const * p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds except current bond be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        /* Do nothing if this is our own bond. */
        if (conn_handle != p_bms->conn_handle)
        {
            bond_delete(conn_handle, NULL);
        }

        peer_id = pm_next_peer_id_get(peer_id);
    }
}

uint16_t qwr_evt_handler(nrf_ble_qwr_t * p_qwr, nrf_ble_qwr_evt_t * p_evt)
{
    return nrf_ble_bms_on_qwr_evt(&m_bms, p_qwr, p_evt);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hts_init_t     hts_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dis_sys_id_t   sys_id;
    ble_cus_init_t      cus_init = {0};
    nrf_ble_bms_init_t   bms_init;

    // Initialize Queued Write Module
    memset(&qwr_init, 0, sizeof(qwr_init));
    qwr_init.mem_buffer.len   = MEM_BUFF_SIZE;
    qwr_init.mem_buffer.p_mem = m_qwr_mem;
    qwr_init.callback         = qwr_evt_handler;
    qwr_init.error_handler    = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Occupancy Service
     // Initialize CUS Service init structure to zero.
    cus_init.evt_handler                = on_cus_evt;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);    



    // Initialize Bond Management Service
    memset(&bms_init, 0, sizeof(bms_init));

    m_bms_bonds_to_delete        = ble_conn_state_user_flag_acquire();
    bms_init.evt_handler         = bms_evt_handler;
    bms_init.error_handler       = service_error_handler;
#if USE_AUTHORIZATION_CODE
    bms_init.feature.delete_requesting_auth         = true;
    bms_init.feature.delete_all_auth                = true;
    bms_init.feature.delete_all_but_requesting_auth = true;
#else
    bms_init.feature.delete_requesting              = true;
    bms_init.feature.delete_all                     = true;
    bms_init.feature.delete_all_but_requesting      = true;
#endif
    bms_init.bms_feature_sec_req = SEC_JUST_WORKS;
    bms_init.bms_ctrlpt_sec_req  = SEC_JUST_WORKS;

    bms_init.p_qwr                                       = &m_qwr;
    bms_init.bond_callbacks.delete_requesting            = delete_requesting_bond;
    bms_init.bond_callbacks.delete_all                   = delete_all_bonds;
    bms_init.bond_callbacks.delete_all_except_requesting = delete_all_except_requesting_bond;

    err_code = nrf_ble_bms_init(&m_bms, &bms_init);
    APP_ERROR_CHECK(err_code);



    //// Initialize Battery Service.
    //memset(&bas_init, 0, sizeof(bas_init));

    //// Here the sec level for the Battery Service can be changed/increased.
    //bas_init.bl_rd_sec        = SEC_OPEN;
    //bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    //bas_init.bl_report_rd_sec = SEC_OPEN;

    //bas_init.evt_handler          = NULL;
    //bas_init.support_notification = true;
    //bas_init.p_report_ref         = NULL;
    //bas_init.initial_batt_level   = 100;

    //err_code = ble_bas_init(&m_bas, &bas_init);
    //APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    // Temperature is in celcius (it is multiplied by 100 to avoid floating point arithmetic).
    m_temp_celcius_sim_cfg.min          = MIN_CELCIUS_DEGREES;
    m_temp_celcius_sim_cfg.max          = MAX_CELCIUS_DEGRESS;
    m_temp_celcius_sim_cfg.incr         = CELCIUS_DEGREES_INCREMENT;
    m_temp_celcius_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code = 0;

    // Start application timers.
    //err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    err_code |= app_timer_start(m_vl53l5cx_timer_id, READ_VL53L5CX_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
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
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
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
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
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


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            //m_hts_meas_ind_conf_pending = false;
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
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                //temperature_measurement_send();
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

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
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
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


/**@brief Function for handling the idle state (main loop).
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
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

void twi_init(void)
{
  ret_code_t err_code;

  // create some configurations, save in stuct, pass into init function
  const nrf_drv_twi_config_t twi_config = {
    .scl                = 17,                     // pin 17 to scl
    .sda                = 16,                     // pin 16 to sda
    .frequency          = NRF_DRV_TWI_FREQ_400K,  // 400kHz
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH  // not using softdevice so priority is not changed (would have to be changed if using one)
    //.clear_bus_init     = false                   // 

  };

  // initialize
  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL); // pass address of instance and config, null event handler and something else
  APP_ERROR_CHECK(err_code);  // check err_code
  
  // enable
  nrf_drv_twi_enable(&m_twi);  
}

void vl53l5cx_sensor_init(void)
{
  //uint8_t streamcount = 0;
  sensor_config.platform.address = 0x29;
  sensor_config.platform.m_twi; 
 //     /* Platform, filled by customer into the 'platform.h' file */
 //       .platform = {
 //         .address = 0x29,
 //         .m_twi = m_twi,
 //       },
	///* Results streamcount, value auto-incremented at each range */
	////.streamcount = 0,
	///* Size of data read though I2C */
	////uint32_t	        data_read_size;
	///* Offset buffer */
	////uint8_t		        offset_data[VL53L5CX_OFFSET_BUFFER_SIZE];
	///* Xtalk buffer */
	////uint8_t		        xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];
	///* Temporary buffer used for internal driver processing */
	// //uint8_t	        temp_buffer[VL53L5CX_TEMPORARY_BUFFER_SIZE];
 // };

  VL53L5CX_ResultsData sensor_data = {
    
  };
  
  /*********************************/
  /*      Initialization           */
  /*********************************/

  uint8_t isAlive;
  uint8_t status;
  uint8_t ranging_started;
  VL53L5CX_Motion_Configuration 	motion_config;	/* Motion configuration*/
 
  status |= vl53l5cx_is_alive(&sensor_config, &isAlive);
  if (status) printf("Sensor failed alive test with: %u\n", status);

  status |= vl53l5cx_init(&sensor_config);
  //if (status) printf("Initialization failed with: %u\n", status);
  
  /**** Steps and order? ****/
  /* Resolution
  /* Ranging Mode
  /* Ranging frequency
  /* Target order
  /* Integration time */
  //status |= vl53l5cx_stop_ranging(&sensor_config);

  //status |= vl53l5cx_start_ranging(&sensor_config);   /* Test ranging with defaults */
  //if (status) printf("Start Ranging Failed with: %u\n", status);

  //status |= vl53l5cx_get_ranging_data(&sensor_config, &sensor_data);

  //status |= vl53l5cx_stop_ranging(&sensor_config);

  /*********************************/
  /*   Program motion indicator    */
  /*********************************/

  //status = vl53l5cx_motion_indicator_init(&sensor_config, &motion_config, VL53L5CX_RESOLUTION_4X4);
  //  if(status)
  //    printf("Motion indicator init failed with status : %u\n", status);

  ///* (Optional) Change the min and max distance used to detect motions. The
  // * difference between min and max must never be >1500mm, and minimum never be <400mm,
  // * otherwise the function below returns error 127 */
  //status = vl53l5cx_motion_indicator_set_distance_motion(&sensor_config, &motion_config, MOTION_MINIMUM, MOTION_MAXIMUM);
  //  if(status)
  //    printf("Motion indicator set distance motion failed with status : %u\n", status);

  /* If user want to change the resolution, he also needs to update the motion indicator resolution */
  //status = vl53l5cx_set_resolution(&sensor_config, VL53L5CX_RESOLUTION_4X4);
  //status = vl53l5cx_motion_indicator_set_resolution(&sensor_config, &motion_config, VL53L5CX_RESOLUTION_4X4);

  ///* Increase ranging frequency for the example */
  status = vl53l5cx_set_ranging_frequency_hz(&sensor_config, RANGING_FREQUENCY); // Set ranging frequency
    if(status)
      printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);

  uint8_t temp_number; // stores return data from commands
  vl53l5cx_get_ranging_mode(&sensor_config,  &temp_number);

  if(temp_number != VL53L5CX_RANGING_MODE_AUTONOMOUS) {
    status |= vl53l5cx_set_ranging_mode(&sensor_config, VL53L5CX_RANGING_MODE_AUTONOMOUS);  // PROBLEM AREA
  }
    if (status)
      printf("Mode not changed: %u\n", status);
 
  status |= vl53l5cx_get_ranging_frequency_hz(&sensor_config, &temp_number);
    if (status)
      printf("Frequency get failed: %u\n", status);

  /*********************************/
  /*  Program detection thresholds */
  /*********************************/

  // set up detection threshold
  VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];
  memset(&thresholds, 0, sizeof(thresholds));

//// 4x4
//  for(int i = 0; i < 16; i++){
//     /* The first wanted thresholds is GREATER_THAN mode. Please note that the
//     * first one must always be set with a mathematic_operation
//     * VL53L5CX_OPERATION_NONE.
//     * For this example, the signal thresholds is set to 150 kcps/spads
//     * (the format is automatically updated inside driver)
//     */
//    //thresholds[2*i].zone_num = i;
//    //thresholds[2*i].measurement = VL53L5CX_SIGNAL_PER_SPAD_KCPS;
//    //thresholds[2*i].type = VL53L5CX_GREATER_THAN_MAX_CHECKER;
//    //thresholds[2*i].mathematic_operation = VL53L5CX_OPERATION_NONE;
//    //thresholds[2*i].param_low_thresh = 10;
//    //thresholds[2*i].param_high_thresh = 10;

//    /* The second wanted checker is IN_WINDOW mode. We will set a
//     * mathematical thresholds VL53L5CX_OPERATION_OR, to add the previous
//     * checker to this one.
//     * For this example, distance thresholds are set between 200mm and
//     * 400mm (the format is automatically updated inside driver).
//     */
//    thresholds[2*i+1].zone_num = i;
//    thresholds[2*i+1].measurement = VL53L5CX_DISTANCE_MM;
//    thresholds[2*i+1].type = VL53L5CX_IN_WINDOW;
//    thresholds[2*i+1].mathematic_operation = VL53L5CX_OPERATION_OR;
//    thresholds[2*i+1].param_low_thresh = 100;
//    thresholds[2*i+1].param_high_thresh = 500;
//    }

//    /* The last thresholds must be clearly indicated. As we have 32
//    * checkers (16 zones x 2), the last one is the 31 */
//    thresholds[31].zone_num = VL53L5CX_LAST_THRESHOLD | thresholds[16].zone_num;

//    /* Send array of thresholds to the sensor */
//    status |= vl53l5cx_set_detection_thresholds(&sensor_config, thresholds);

//    /* Enable detection thresholds */
//    status |= vl53l5cx_set_detection_thresholds_enable(&sensor_config, 1);


  /*********************************/
  /*         Ranging loop          */
  /*********************************/
  /* Start a ranging session */
  status = vl53l5cx_set_integration_time_ms(&sensor_config, INTEGRATION_TIME);  // 10ms
  status = vl53l5cx_start_ranging(&sensor_config);
  NRF_LOG_INFO("Start ranging loop\n");

  app_timer_start(m_system_reset_timer_id, SYSTEM_RESET_INTERVAL, NULL);  // reset system after 1000ms


  uint8_t loop, i = 0;
  VL53L5CX_ResultsData 	Results;
  //uint8_t person_entry = 0, check = 1, person_exit = 0, person_entry_2, person_exit_2;
  loop = 0;
  while(loop == 0)
   	{
   		status = vl53l5cx_check_data_ready(&sensor_config, &isReady);
   		if(isReady == 1)
   		{
   			vl53l5cx_get_ranging_data(&sensor_config, &Results);

   			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
   			// NRF_LOG_INFO("Print data no : %3u\n", sensor_config.streamcount);
   			for(i = 0; i < 16; i++)
   			{                  //"Zone : %3d, Status : %3u, Distance : %4d mm\n" Results.target_status
                                           //"Zone : %3d, Motion : %3d, Distance : %4d mm\n" Results.motion_indicator.motion
				NRF_LOG_INFO("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					 Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
					(ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]));
   			}
                        /* Entry - not pin side to pin side */
                        // person just in area 1
                        if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) > PERSON_MIN_HEIGHT || 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) > PERSON_MIN_HEIGHT)
                          { // not in area 3
                            if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT)
                              {
                                person_entry = 1;
                              }
                          }
                        // person just in area 3                       
                        if(((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) > PERSON_MIN_HEIGHT || 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) > PERSON_MIN_HEIGHT)
                          && person_entry == 1) 
                          { // not in area 1 or 2
                            if( /* 1 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT) &&

                               /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

                              ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
                              {
                                person_exit = 1;
                              }
                          }

                        // No person present - clear flags
                        if(/* 3 */ 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT && 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT &&

                          /* 1 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT) &&

                           /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

                          ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
                          {
                            if(person_exit) m_cus.current_value++; // person passed through and area is clear
                            person_entry = 0;
                            person_exit = 0;
                          }

                          /* Exit - pin side to non pin side */
                        // person just in area 3
                        if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) > PERSON_MIN_HEIGHT || 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) > PERSON_MIN_HEIGHT)
                          { // not in area 1
                            if((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT)
                              {
                                person_entry_2 = 1;
                              }
                          }
                        // person just in area 1                       
                        if(((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) > PERSON_MIN_HEIGHT || 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) > PERSON_MIN_HEIGHT ||
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) > PERSON_MIN_HEIGHT)
                          && person_entry_2 == 1) 
                          { // not in area 3 or 2
                            if( /* 3 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT) &&

                               /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

                              ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
                              (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
                              {
                                person_exit_2 = 1;
                              }
                          }

                        // No person present - clear flags
                        if(/* 3 */ 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*3]) < PERSON_MIN_HEIGHT && 
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*7]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*11]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*15]) < PERSON_MIN_HEIGHT &&

                          /* 1 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*0]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*4]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*8]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*12]) < PERSON_MIN_HEIGHT) &&

                           /* 2 */ ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*1]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*5]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*9]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*13]) < PERSON_MIN_HEIGHT) &&

                          ((ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*2]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*6]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*10]) < PERSON_MIN_HEIGHT &&
                          (ceiling_height-Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*14]) < PERSON_MIN_HEIGHT) )
                          {
                            if(person_exit_2)
                             m_cus.current_value--; // person passed through and area is clear
                            person_entry_2 = 0;
                            person_exit_2 = 0;
                          }

   			// NRF_LOG_INFO("");
   			loop++;
   		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
   		WaitMs(&(sensor_config.platform), 10);
   	}
  printf("done");
  app_timer_stop(m_system_reset_timer_id);  // stop system reset

}





// For INT PIN interupts
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) // pin is which pin triggered it, actions is if it was a high to low or low to high interupt
{
  //if(action == GPIOTE_CONFIG_POLARITY_LoToHi) {
        printf("interupt occurred");
      //}
}

static void gpio_init()
{
  // static pins
    nrf_gpio_cfg_output(15); // LPn
    nrf_gpio_pin_set(15);
    nrf_gpio_cfg_output(14); // PwrEn
    nrf_gpio_pin_set(14);
    nrf_gpio_cfg_output(13); // I2C_Rst
    nrf_gpio_pin_clear(13);

 //interupt pin 12 = INT but int doesn't set currently
  //ret_code_t err_code;

  //err_code = nrf_drv_gpiote_init();
  //APP_ERROR_CHECK(err_code);

  //nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  //in_config.pull = NRF_GPIO_PIN_PULLDOWN;

  //err_code = nrf_drv_gpiote_in_init(12, &in_config, in_pin_handler);  // INT on pin 12
  //APP_ERROR_CHECK(err_code);

  //nrf_drv_gpiote_in_event_enable(12, true);
}

// FROM: https://github.com/NordicPlayground/nrf51-TIMER-examples/blob/master/timer_example_timer_mode/main.c
//  https://devzone.nordicsemi.com/f/nordic-q-a/35230/timer-interrupts-nrf
//void start_timer(void)
//{		
//    NRF_TIMER0->TASKS_STOP = 1; // Stop timer
//    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // taken from Nordic dev zone
//    NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
//    NRF_TIMER0->PRESCALER = 8; // 1us resolution
//    NRF_TIMER0->TASKS_CLEAR = 1; // Clear timer
//    NRF_TIMER0->CC[0] = 62500;
//    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos; // taken from Nordic dev zone
//    NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
//    //attachInterrupt(TIMER1_IRQn, TIMER1_Interrupt); // also used in variant.cpp to configure the RTC1
//    NVIC_EnableIRQ(TIMER0_IRQn);
//    NRF_TIMER0->TASKS_START = 1; // Start TIMER
//}

void start_timer(void)
{		
    //// Irq setup
    //NVIC_SetPriority(RTC0_IRQn, 15); // Lowest priority
    //NVIC_ClearPendingIRQ(RTC0_IRQn);
    //NVIC_EnableIRQ(RTC0_IRQn);

    //// Start LFCLK clock
    //nrf_clock_lf_src_set(NRF_CLOCK_LF_SRC_RC); // 32KHz RC
    //nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);

    //// Set prescaler to the max value (12-bit)
    //// -> 8Hz counter frequency
    //nrf_rtc_prescaler_set(NRF_RTC0,(1<<12) -1);
    //nrf_rtc_event_enable(NRF_RTC0, NRF_RTC_INT_TICK_MASK); /* yes INT mask must be used here */
    //nrf_rtc_int_enable(NRF_RTC0,NRF_RTC_INT_TICK_MASK);
    //nrf_rtc_task_trigger(NRF_RTC0,NRF_RTC_TASK_START);
    
}


		
/** RTC0 Interupt Handler run when RTC interupt is triggered
 */
void RTC0_IRQHandler(void) {
    nrf_rtc_event_clear(NRF_RTC0,NRF_RTC_EVENT_TICK);
    uint8_t isReady;
    vl53l5cx_check_data_ready(&sensor_config, &isReady);
    if(isReady) 
    {
      read_lidar_data();
      
      
    }
}




  /*********************************/
  /*                 ADC           */
  /*********************************/
//static void adc_start(uint32_t cc_value)
//{
//    ret_code_t err_code;

//    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
//    saadc_adv_config.internal_timer_cc = cc_value;
//    saadc_adv_config.start_on_end = true;

//    err_code = nrfx_saadc_advanced_mode_set((1<<0),
//                                            NRF_SAADC_RESOLUTION_10BIT,
//                                            &saadc_adv_config,
//                                            adc_handler);
//    APP_ERROR_CHECK(err_code);
                                            
//    // Configure two buffers to ensure double buffering of samples, to avoid data loss when the sampling frequency is high
//    err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrfx_saadc_mode_trigger();
//    APP_ERROR_CHECK(err_code);
//}

//static void adc_handler(nrfx_saadc_evt_t const * p_event)
//{
//    ret_code_t err_code;
//    switch (p_event->type)
//    {
//        case NRFX_SAADC_EVT_DONE:
//            NRF_LOG_INFO("DONE. Sample[0] = %i", p_event->data.done.p_buffer[0]);

//            // Add code here to process the input
//            // If the processing is time consuming execution should be deferred to the main context
//            break;

//        case NRFX_SAADC_EVT_BUF_REQ:
//            // Set up the next available buffer
//            err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
//            APP_ERROR_CHECK(err_code);
//            break;
//    }
//}





/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // BLE init
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("Health Thermometer example started.");
    application_timers_start();
    advertising_start(erase_bonds);

    /* Sensor init */
    twi_init();

    gpio_init();
        NRF_LOG_INFO("GPIO initilized");
        NRF_LOG_FLUSH();

    vl53l5cx_sensor_init();
    //start_timer();

    // Enter main loop.
    while (1)
    {
        idle_state_handle();
        
    }
}


/**
 * @}
 */