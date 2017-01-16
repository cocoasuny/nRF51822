/**
  ****************************************************************************************
  * @file    ble_top_implementation.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   ble communication top level implementation
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
#include "ble_top_implementation.h"



/* peripheral related. */
#define DEVICE_NAME                      "NordicLESCApp"                            /**< Name of device used for advertising. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 40                                        /**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       60                                        /**< The advertising timeout in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */




/* private variables define */
static BLE_SCAN_LIST_T    m_targetConnectDevInfo;       //需要连接设备信息


/* private function declare */
static void ble_stack_init(void);
static uint32_t ble_new_event_handler(void);
static void sys_evt_dispatch(uint32_t sys_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void db_discovery_init(void);
static void db_disc_handler(ble_db_discovery_evt_t * p_evt);
static void gap_params_init(void);
static void conn_params_init(void);
static void conn_params_error_handler(uint32_t nrf_error);
static void advertising_init(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static ret_code_t start_ble_scan(void);
static ret_code_t stop_ble_scan(void);
static void services_init(void);
static void vTimerStopBleScanCB(xTimerHandle pxTimer);
static void reset_scan_list(void);
static void reset_target_Connnect_DevInfo(void);

/**
  * @brief  ble_top_implementation_thread 
  * @note   the top level implementation of ble task 
  * @param  void * arg
  * @retval None
  */
void ble_top_implementation_thread(void * arg)
{
	UNUSED_PARAMETER(arg);
	
	/* ble stack init */
	ble_stack_init();	
	db_discovery_init();
	gap_params_init();
	conn_params_init();
	services_init();
	advertising_init();
	
    while (1)
    {
        /* Wait for event from SoftDevice */
        while (pdFALSE == xSemaphoreTake(g_semaphore_ble_event_ready, portMAX_DELAY))
        {
            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
        }

        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();
    }	
}
/**
  * @brief  ble_event_handler_thread 
  * @note   the ble event handler
  * @param  void * arg
  * @retval None
  */
void ble_event_handler_thread(void * arg)
{
    BLE_MSG_T               bleEventMsgValue;
    const TickType_t 		xMaxBlockTime = pdMS_TO_TICKS(300); /* 设置最大等待时间为 300ms */
    TimerHandle_t           scanCTL_Timer = NULL;
    uint8_t                 i=0;
    
    UNUSED_PARAMETER(arg);
    
    /* creat event queue for ble event */
    g_bleEventQueue = xQueueCreate(BLE_EVENT_QUEUE_SIZE,sizeof(BLE_MSG_T));
    #ifdef DEBUG_BLE_EVENT
        if(g_bleEventQueue == NULL)
        {
            printf("ble event queue creat fail\r\n");
        }
    #endif
        
    /* ble event queue init */
    bleEventMsgValue.eventID = EVENT_APP_BLE_DEFAULT;
    
    while(1)
    {
        // [code block]: queue message handler
        {
            if(pdPASS == xQueueReceive(g_bleEventQueue,(void *)&bleEventMsgValue,xMaxBlockTime))
            {
                /* 接收到消息，对消息事件进行处理 */
                switch(bleEventMsgValue.eventID)
                {
                    case EVENT_APP_BLE_START_SCAN:
                    {
                        #ifdef DEBUG_BLE_SCAN
                            printf("start scan\r\n");
                        #endif
                        /* start ble scan */
                        start_ble_scan();
                        
                        /* 清空扫描列表 */
                        reset_scan_list();
                        
                        /* 分配timer控制扫描时间 */
                        scanCTL_Timer = xTimerCreate("scanTime",STOP_SCAN_TIME,pdFALSE,(void *)0,vTimerStopBleScanCB);
                        if(scanCTL_Timer == NULL)
                        {
                            APP_ERROR_CHECK(pdFALSE);
                        }
                        if(xTimerStart(scanCTL_Timer,xMaxBlockTime) != pdPASS)
						{
							APP_ERROR_CHECK(pdFALSE);
						}   
                    }
                    break;
                    case EVENT_APP_BLE_STOP_SCAN:
                    {
                        #ifdef DEBUG_BLE_SCAN
                            printf("stop scan\r\n");
                        #endif
                        /* stop ble scan */
                        stop_ble_scan();
                        
                        /* reset the target connnect device info */
                        reset_target_Connnect_DevInfo();
                        
                        /* 找出扫描列表中最大信号质量强度对应的手环 */
                        for(i=0;i<MAX_SCAN_LIST_NUM;i++)
                        {
                            if(gScanList[i].isValid == false)  //说明在扫描事件中已将填充对应的扫描列表位置
                            {
                                if(m_targetConnectDevInfo.rssi < gScanList[i].rssi)
                                {
                                    memcpy(&m_targetConnectDevInfo,&gScanList[i],sizeof(BLE_SCAN_LIST_T));
                                }
                            }                                
                        }
                        uint32_t SN = 0;
                        SN = (uint32_t)((m_targetConnectDevInfo.sn[0]<<24)+(m_targetConnectDevInfo.sn[1]<<16)
                                        +(m_targetConnectDevInfo.sn[2]<<8)+m_targetConnectDevInfo.sn[3]<<0);
                        printf("target SN:%d\r\n",SN);
                    }
                    break;
                    default:break;
                }
            }
        }
        //[code block]: queue message handler
    }
}

/**@brief Function for start ble scanning.
 */
static ret_code_t start_ble_scan(void)
{
    ret_code_t      err_code = NRF_SUCCESS;
    uint32_t        count = 0;

    //check if there are no flash operations in progress
    err_code = fs_queued_op_count_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count == 0)
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise Heart Rate or Running speed and cadence UUIDs.
        scan_start();  
    }
    
    return err_code;
}
/**
  * @brief  Function for stopping ble scan. 
  * @param  None
  * @retval ret_code_t
  */
static ret_code_t stop_ble_scan(void)
{
    ret_code_t      err_code = NRF_SUCCESS;  

    err_code = scan_stop();
    
    return err_code;
}

/**
  * @brief  Function for initializing the BLE stack. 
  * @note   Initializes the SoftDevice and the BLE event interrupts.
  * @param  None
  * @retval None
  */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            bsp_led_toggle(LED2);
            break;

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code;
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

/**
  * @brief  Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
  * @note   This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
  * @param  p_ble_evt   Bluetooth stack event.
  * @retval None
  */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	uint16_t conn_handle;
    uint16_t role;
	
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
	
	// The connection handle should really be retrievable for any event type.
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    role        = ble_conn_state_role(conn_handle);
	
    // Based on the role this device plays in the connection, dispatch to the right applications.
    if (role == BLE_GAP_ROLE_PERIPH)
    {
		ble_advertising_on_ble_evt(p_ble_evt);
		ble_conn_params_on_ble_evt(p_ble_evt);
	}
	else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
	{
        /** on_ble_central_evt will update the connection handles, so we want to execute it
         * after dispatching to the central applications upon disconnection. */
        if (p_ble_evt->header.evt_id != BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }		
	}  
}
/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
//    // Dispatch the system event to the fstorage module, where it will be
//    // dispatched to the Flash Data Storage (FDS) module.
//    fs_sys_event_handler(sys_evt);

//    // Dispatch to the Advertising module last, since it will check if there are any
//    // pending flash operations in fstorage. Let fstorage process system events first,
//    // so that it can report correctly to the Advertising module.
//    ble_advertising_on_sys_evt(sys_evt);
}

/**
  * @brief  Event handler for new BLE events
  * @note   This function is called from the SoftDevice handler. 
  *         It is called from interrupt level.
  * @param  None
  * @retval The returned value is checked in the softdevice_handler module,
  *         using the APP_ERROR_CHECK macro.
  */
static uint32_t ble_new_event_handler(void)
{
    BaseType_t yield_req = pdFALSE;

    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    UNUSED_VARIABLE(xSemaphoreGiveFromISR(g_semaphore_ble_event_ready, &yield_req));
    portYIELD_FROM_ISR(yield_req);
    return NRF_SUCCESS;
}

/**
  * @brief  Function for the GAP initialization.
  * @note   This function sets up all the necessary GAP (Generic Access Profile) parameters of the
  *         device including the device name, appearance, and the preferred connection parameters.
  * @param  None
  * @retval None
  */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
  * @brief  Function for initializing the Connection Parameters module.
  * @note   Connection Parameters
  * @param  None
  * @retval None
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
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling database discovery events.
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 * @param[in] p_event  Pointer to the database discovery event.
 * @retval None
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
//    ble_hrs_on_db_disc_evt(&m_ble_hrs_c, p_evt);
}

/**
  * @brief  Database discovery initialization.
  * @note   Database discovery initialization. 
  * @param  None
  * @retval None
  */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**
  * @brief  Function for initializing the Advertising functionality.
  * @note   Function for initializing the Advertising functionality.
  * @param  None
  * @retval None
  */
static void advertising_init(void)
{
    uint32_t               				err_code;
    ble_advdata_t          				advdata;
	ble_advdata_t              			scanrsp;
    ble_adv_modes_config_t 				options;
	ble_advdata_manuf_data_t    		manuf;
	uint8_array_t              			adv_manuf_data_array;
	uint8_t                             manufdata[2];
	ble_uuid_t 							scanrsp_uuids[] = {BLE_UUID_PASSKEY_AUTH_SERVICE,BLE_UUID_TYPE_BLE};
    ble_uuid_t							adv_uuids[] = {CHECK_UP_UUID_SERVICE,BLE_UUID_TYPE_BLE};

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

	/* advertise manufactor data */
	manufdata[0]                    = g_DeviceInformation.SeriNum[2];
	manufdata[1]                    = g_DeviceInformation.SeriNum[3];
	adv_manuf_data_array.p_data     = manufdata;
	adv_manuf_data_array.size       = 2;
	manuf.company_identifier        = g_DeviceInformation.SeriNum[1]*256 + g_DeviceInformation.SeriNum[0];
	manuf.data                      = adv_manuf_data_array;
	advdata.p_manuf_specific_data   = &manuf;
	
	memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(scanrsp_uuids) / sizeof(scanrsp_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = scanrsp_uuids;
	
    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

}
/**
  * @brief  Function for initializing the service
  * @note   Function for initializing the service
  * @param  None
  * @retval None
  */
static void services_init(void)
{
	uint32_t						err_code;
	ble_checkup_service_init_t		checkup_service_init;
	ble_passkey_service_init_t		passkey_service_init;
	
	/* Add immediately measure service */
	memset(&checkup_service_init,0,sizeof(ble_checkup_service_init_t));
	err_code = ble_checkup_service_init(&gBleServiceCheckUp,&checkup_service_init);
	if(err_code != NRF_SUCCESS)
	{
		APP_ERROR_CHECK(err_code);
	}
	
	
	/* Add passkey confirm service */
	memset(&passkey_service_init,0,sizeof(ble_passkey_service_init_t));
	err_code = ble_passkey_service_init(&gBleServicePasskey,&passkey_service_init);
	if(err_code != NRF_SUCCESS)
	{
		APP_ERROR_CHECK(err_code);
	}
}
/**
  * @brief  vTimerStopBleScanCB
  * @note   stop ble scan timer call back
  * @param  pxTimer
  * @retval None
  */
static void vTimerStopBleScanCB(xTimerHandle pxTimer)
{
    BLE_MSG_T               bleEventMsgValue;
    const TickType_t        xTicksToWait = 100 / portTICK_PERIOD_MS; /* 最大等待时间100ms */
    
    bleEventMsgValue.eventID = EVENT_APP_BLE_STOP_SCAN;
    if(xQueueSend(g_bleEventQueue,(void *)&bleEventMsgValue,xTicksToWait) != pdPASS)
    {
        APP_ERROR_CHECK(pdFAIL);
    }
}
/**
  * @brief  reset_scan_list
  * @param  None
  * @retval None
  */
static void reset_scan_list(void)
{
    uint8_t i = 0;
    
    for(i=0;i<MAX_SCAN_LIST_NUM;i++)
    {
        memset(&gScanList[i],0,sizeof(BLE_SCAN_LIST_T));
        gScanList[i].isValid = true;
        gScanList[i].rssi = DEFAULT_REF_RSSI;
    }
}
/**
  * @brief  reset_target_Connnect_DevInfo
  * @param  None
  * @retval None
  */
static void reset_target_Connnect_DevInfo(void)
{
    memset(&m_targetConnectDevInfo,0,sizeof(BLE_SCAN_LIST_T));
    m_targetConnectDevInfo.isValid = true;
    m_targetConnectDevInfo.rssi = DEFAULT_REF_RSSI;
}

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

