#include "sdk_common.h"
#include "ble_cus.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "read_write_memory.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_cus->custom_value_handles.value_handle)
    {
        
        //nrf_gpio_pin_toggle(LED_4);
        ble_cus_custom_value_update(p_cus, p_evt_write->data);
        p_cus->current_value = (*p_evt_write->data << 8) + *(p_evt_write->data + 1);
        /*
        if(*p_evt_write->data == 0x01)
        {
            nrf_gpio_pin_clear(20); 
        }
        else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
          //Do nothing
        }
        */
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_cus->custom_value_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }

  /* Ceiling */
    // Ceiling Value Characteristic Written to.
    if (p_evt_write->handle == p_cus->ceiling_value_handles.value_handle)
    {
        
        //nrf_gpio_pin_toggle(LED_4);
        //uint8_t temp_array[2] = {(*p_evt_write->data << 8) , *(p_evt_write->data + 1)};
        ble_cus_ceiling_value_update(p_cus, p_evt_write->data);
        //p_cus->current_value_2 = (*p_evt_write->data << 8) + *(p_evt_write->data + 1);
        int ceiling_height = p_cus->current_value_2;


        uint8_t error = 0;
    vl53l5cx_stop_ranging(&p_cus->sensor_config);

    error |= vl53l5cx_set_detection_thresholds_enable(&p_cus->sensor_config, 0);
    memset(&p_cus->thresholds, 0, sizeof(p_cus->thresholds));
    /* Add thresholds for all zones (16 zones in resolution 4x4, or 64 in 8x8) */
    for(int i = 0; i < 16; i++)
    {
          p_cus->thresholds[2*i+1].zone_num = i;
          p_cus->thresholds[2*i+1].measurement = VL53L5CX_DISTANCE_MM;
          p_cus->thresholds[2*i+1].type = VL53L5CX_IN_WINDOW;
          p_cus->thresholds[2*i+1].mathematic_operation = VL53L5CX_OPERATION_OR;
          p_cus->thresholds[2*i+1].param_low_thresh = 10;
          p_cus->thresholds[2*i+1].param_high_thresh = ceiling_height-1500;
    }
    p_cus->thresholds[31].zone_num = VL53L5CX_LAST_THRESHOLD | p_cus->thresholds[31].zone_num;
    error |= vl53l5cx_set_detection_thresholds(&p_cus->sensor_config, p_cus->thresholds);
    error |= vl53l5cx_set_detection_thresholds_enable(&p_cus->sensor_config, 1);

    vl53l5cx_start_ranging(&p_cus->sensor_config);




        /*
        if(*p_evt_write->data == 0x01)
        {
            nrf_gpio_pin_clear(20); 
        }
        else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
          //Do nothing
        }
        */
    }

    // Check if the Ceiling value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    //if ((p_evt_write->handle == p_cus->ceiling_value_handles.cccd_handle)
    //    && (p_evt_write->len == 2)
    //   )
    //{
    //    // CCCD written, call application event handler
    //    if (p_cus->evt_handler != NULL)
    //    {
    //        ble_cus_evt_t evt;

    //        if (ble_srv_is_notification_enabled(p_evt_write->data))
    //        {
    //            evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
    //        }
    //        else
    //        {
    //            evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
    //        }
    //        // Call the application event handler.
    //        p_cus->evt_handler(p_cus, &evt);
    //    }
    //}

}

void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cus_t * p_cus = (ble_cus_t *) p_context;
    
    NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cus, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t)*2;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t)*2;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

//* Same as custom but with changed uuid
static uint32_t ceiling_value_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CEILING_VALUE_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t)*2;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t)*2;

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->ceiling_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


static uint32_t ble_value_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init, uint16_t UUID)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = BATTERY_VALUE_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->battery_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    // Add Custom Value characteristic
    err_code = ceiling_value_char_add(p_cus, p_cus_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    err_code = custom_value_char_add(p_cus, p_cus_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 
    err_code = ble_value_char_add(p_cus, p_cus_init, BATTERY_VALUE_UUID);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 

    //uint16_t i = 2000;
    //ble_cus_custom_value_update(p_cus, &i);
    //p_cus->current_value_2 = 0x2001;
    return NRF_SUCCESS;
}

uint32_t ble_cus_custom_value_update(ble_cus_t * p_cus, uint8_t * custom_value)
{
    NRF_LOG_INFO("In ble_cus_custom_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    __ALIGN(4) uint8_t temp_array[4] = {(*custom_value) , *(custom_value + 1)};
    p_cus->current_value = (*custom_value << 8) + *(custom_value + 1);

    err_code = kls_fds_find_and_delete(OCCU_FILE_ID_FDS, OCCU_REC_KEY_FDS);
    APP_ERROR_CHECK(err_code);

    err_code = kls_fds_write(OCCU_FILE_ID_FDS, OCCU_REC_KEY_FDS, temp_array);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Flash Write: %d", (*custom_value << 8) + *(custom_value + 1));

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->custom_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->custom_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }
    



    return err_code;
}



uint32_t ble_cus_ceiling_value_update(ble_cus_t * p_cus, uint8_t * ceiling_value)
{
    NRF_LOG_INFO("In ble_cus_ceiling_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    __ALIGN(4) uint8_t temp_array[4] = {(*ceiling_value) , *(ceiling_value + 1), 0x00, 0x00};
    p_cus->current_value_2 = (*ceiling_value << 8) + *(ceiling_value + 1);

    err_code = kls_fds_find_and_delete(CEIL_FILE_ID_FDS, CEIL_REC_KEY_FDS);
    APP_ERROR_CHECK(err_code);

    err_code = kls_fds_write(CEIL_FILE_ID_FDS, CEIL_REC_KEY_FDS, temp_array);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Flash Write: %d", (*ceiling_value << 8) + *(ceiling_value + 1));


    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = ceiling_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->ceiling_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->ceiling_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }




    return err_code;
}

// not changable with others since uses p_cus->current_value
uint32_t ble_cus_battery_value_update(ble_cus_t * p_cus, uint8_t * current_value, uint8_t len, ble_gatts_char_handles_t value_handles)
{
    //NRF_LOG_INFO("In ble_cus_batter_value_update. \r\n"); 
    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = current_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      p_cus->battery_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cus->battery_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        //NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        //NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }
    p_cus->current_value_battery = *(current_value + 1);



    return err_code;
}

