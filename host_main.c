/******************************************************************************
* File Name: host_main.c
*
*
* Description:
*
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_syslib.h"
#include "cy_sysint.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "stdio.h"
#include "common.h"


/******************************************************************************
* Macros
*******************************************************************************/
#define ENABLE      (1u)
#define DISABLE     (0u)

#define DEBUG_BLE_ENABLE    ENABLE

#if DEBUG_BLE_ENABLE
#define DEBUG_BLE       printf
#else
#define DEBUG_BLE(...)
#endif

typedef struct
{
  int8_t  exponent;
  int32_t mantissa;
} ieee_float32_t;

#define CUSTOM_ACC_X_DECL_HANDLE		cy_ble_customsConfig.attrInfo->customServInfo->customServCharDesc[0]
#define CUSTOM_ACC_X_CHAR_HANDLE		cy_ble_customsConfig.attrInfo->customServInfo[0].customServCharHandle

#define CUSTOM_ACC_Y_DECL_HANDLE		cy_ble_customsConfig.attrInfo->customServInfo->customServCharDesc[1]
#define CUSTOM_ACC_Y_CHAR_HANDLE		cy_ble_customsConfig.attrInfo->customServInfo[1].customServCharHandle

#define CUSTOM_ACC_Z_DECL_HANDLE		cy_ble_customsConfig.attrInfo->customServInfo->customServCharDesc[2]
#define CUSTOM_ACC_Z_CHAR_HANDLE		cy_ble_customsConfig.attrInfo->customServInfo[2].customServCharHandle

#define CUSTOM_BATT_DECL_HANDLE			cy_ble_customsConfig.attrInfo->customServInfo->customServCharDesc[3]
#define CUSTOM_BATT_CHAR_HANDLE			cy_ble_customsConfig.attrInfo->customServInfo[3].customServCharHandle

#define CUSTOM_TEMP_DECL_HANDLE			cy_ble_customsConfig.attrInfo->customServInfo->customServCharDesc[4]
#define CUSTOM_TEMP_CHAR_HANDLE			cy_ble_customsConfig.attrInfo->customServInfo[4].customServCharHandle

#define CUSTOM_PRES_DECL_HANDLE			cy_ble_customsConfig.attrInfo->customServInfo->customServCharDesc[5]
#define CUSTOM_PRES_CHAR_HANDLE			cy_ble_customsConfig.attrInfo->customServInfo[5].customServCharHandle

/*******************************************************************************
* Variables
*******************************************************************************/
bool AccXNotificationEnabled = false;
bool AccYNotificationEnabled = false;
bool AccZNotificationEnabled = false;
bool BattNotificationEnabled = false;
bool TempNotificationEnabled = false;
bool PresIndicationEnabled = false;
bool PresIndicationReceived = false;

cy_stc_ble_gatts_handle_value_ntf_t AccxNotificationPacket;
cy_stc_ble_gatts_handle_value_ntf_t AccyNotificationPacket;
cy_stc_ble_gatts_handle_value_ntf_t AcczNotificationPacket;
cy_stc_ble_gatts_handle_value_ntf_t BattNotificationPacket;
cy_stc_ble_gatts_handle_value_ntf_t TempNotificationPacket;
cy_stc_ble_gatts_handle_value_ind_t PressIndicationPacket;

cy_stc_ble_conn_handle_t appConnHandle;
/* BLESS interrupt configuration.
 * It is used when BLE middleware operates in BLE Single CM4 Core mode. */
const cy_stc_sysint_t blessIsrCfg =
{
    /* The BLESS interrupt */
    .intrSrc      = bless_interrupt_IRQn,

    /* The interrupt priority number */
    .intrPriority = 1u
};

/*SensorHub local variables*/
static int accx_value = 0;
static int accy_value = 0;
static int accz_value = 0;
static unsigned char batt_value = 0;
static int temp_formated = 0;
static int press_value = 0;

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
extern void ProcessSensors(void);
void Ble_Init(void);
void BlessInterrupt(void);
void IasEventHandler(uint32_t event, void *eventParam);
void StackEventHandler(uint32 event, void* eventParam);

void AccXNotification(void);
void AccYNotification(void);
void AccZNotification(void);
void BattNotification(void);
void TempNotification(void);
void PressIndication(void);

ieee_float32_t c_float32_to_ieee_11073_float32(float input);
int float32_to_ieee_11073_int(float input);

extern sensor_data_t sensor_data_storage;

/*******************************************************************************
* Function Name: HostMain()
********************************************************************************
* Summary:
*  Main function for the BLE Host.
*
*******************************************************************************/
int HostMain(void)
{
    __enable_irq(); /* Enable global interrupts. */
    uint32_t cnt = 0;
    
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    
    printf("RDK3 BLE Throughput Measurement\r\n");
    printf("Role : Server\r\n");
    
    /* Initialize BLE */
    Ble_Init();

    for(;;)
    {
        /* Cy_Ble_ProcessEvents() allows BLE stack to process pending events */
        Cy_BLE_ProcessEvents();

        if(cnt == 50000)
        {
        	cnt = 0;

            /*Process SensorFusion Data*/
            ProcessSensors();

            /*Update data for client read access*/
            accx_value = (int)(sensor_data_storage.bmi_acc_x + 32767);
            accy_value = (int)(sensor_data_storage.bmi_acc_y + 32767);
            accz_value = (int)(sensor_data_storage.bmi_acc_z + 32767);
            batt_value = (unsigned char)(sensor_data_storage.battery_lvl);
            temp_formated = float32_to_ieee_11073_int((float)(sensor_data_storage.sht_temperature/1000.0));
            press_value = (int)(sensor_data_storage.dps_pressure);
        	cy_ble_config.gattDB[CY_BLE_SENSORHUB_ACC_X_DECL_HANDLE].attValue.attFormatValue.attGenericValLen->attGenericVal = (void*)&accx_value;
        	cy_ble_config.gattDB[CY_BLE_SENSORHUB_ACC_Y_DECL_HANDLE].attValue.attFormatValue.attGenericValLen->attGenericVal = (void*)&accy_value;
        	cy_ble_config.gattDB[CY_BLE_SENSORHUB_ACC_Z_DECL_HANDLE].attValue.attFormatValue.attGenericValLen->attGenericVal = (void*)&accz_value;
        	cy_ble_config.gattDB[CY_BLE_SENSORHUB_BAT_LEVEL_DECL_HANDLE].attValue.attFormatValue.attGenericValLen->attGenericVal = (void*)&batt_value;
        	cy_ble_config.gattDB[CY_BLE_SENSORHUB_TEMP_DECL_HANDLE].attValue.attFormatValue.attGenericValLen->attGenericVal = (void*)&temp_formated;
        	cy_ble_config.gattDB[CY_BLE_SENSORHUB_PRESS_DECL_HANDLE].attValue.attFormatValue.attGenericValLen->attGenericVal = (void*)&press_value;

        	/*Notify/Indicate*/
            if(AccXNotificationEnabled == true)
            {
                if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
                {
                    /* Send notification data to the GATT Client*/
                    AccXNotification();
                }
            }

            if(AccYNotificationEnabled == true)
            {
                if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
                {
                    /* Send notification data to the GATT Client*/
                    AccYNotification();
                }
            }

            if(AccZNotificationEnabled == true)
            {
                if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
                {
                    /* Send notification data to the GATT Client*/
                    AccZNotification();
                }
            }

            if(BattNotificationEnabled == true)
            {
                if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
                {
                    /* Send notification data to the GATT Client*/
                    BattNotification();
                }
            }

            if(TempNotificationEnabled == true)
            {
                if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
                {
                    /* Send notification data to the GATT Client*/
                    TempNotification();
                }
            }

            if(PresIndicationEnabled == true)
            {
                if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
                {
                    /* Send indication data to the GATT Client*/
                    PressIndication();
                }
            }
        }
        cnt++;
    }

    return 0;
}

/*******************************************************************************
* Function Name: Ble_Init()
********************************************************************************
*
* Summary:
*   This function initializes BLE.
*
* Return:
*   None
*
*******************************************************************************/
void Ble_Init(void)
{
    cy_en_ble_api_result_t apiResult;
    cy_stc_ble_stack_lib_version_t stackVersion;
    
    /* Initialize the BLESS interrupt */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;
    Cy_SysInt_Init(cy_ble_config.hw->blessIsrConfig, BlessInterrupt);

    /* Register the generic event handler */
    Cy_BLE_RegisterEventCallback(StackEventHandler);

    /* Initialize the BLE */
    Cy_BLE_Init(&cy_ble_config);

    /* Enable BLE */
    Cy_BLE_Enable();

    /* Register the IAS CallBack */
    Cy_BLE_IAS_RegisterAttrCallback(IasEventHandler);
    
    apiResult = Cy_BLE_GetStackLibraryVersion(&stackVersion);
    
    if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Cy_BLE_GetStackLibraryVersion API Error: 0x%2.2x \r\n",apiResult);
    }
    else
    {
        DEBUG_BLE("Stack Version: %d.%d.%d.%d \r\n", stackVersion.majorVersion, 
        stackVersion.minorVersion, stackVersion.patch, stackVersion.buildNumber);
    }
    
}

/*******************************************************************************
* Function Name: StackEventHandler()
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE Component.
*
*  event - the event code
*  *eventParam - the event parameters
*
* Return:
*   None
*
*******************************************************************************/
void StackEventHandler(uint32 event, void* eventParam)
{

    switch(event)
    {
         /* There are some events generated by the BLE component
        *  that are not required for this code example. */
        
        /**********************************************************
        *                       General Events
        ***********************************************************/
        /* This event is received when the BLE component is Started */
        case CY_BLE_EVT_STACK_ON:
        {
            DEBUG_BLE("CY_BLE_EVT_STACK_ON, Start Advertisement \r\n");    
            /* Enter into discoverable mode so that remote device can search it */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            
            break;
        }    
        /* This event is received when there is a timeout. */
        case CY_BLE_EVT_TIMEOUT:
        {
            break;
		}   
        /* This event indicates that some internal HW error has occurred. */    
		case CY_BLE_EVT_HARDWARE_ERROR:    
        {
			break;
		}   
    	case CY_BLE_EVT_STACK_BUSY_STATUS:
        {
			break;
		}
        /* This event indicates completion of Set LE event mask. */
        case CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
        {
            break;
		}            
        /* This event indicates set device address command completed. */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
        {
            break;
        }
            
        /* This event indicates get device address command completed
           successfully */
        case CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE:
        {
            break;
		}
        /* This event indicates set Tx Power command completed. */
        case CY_BLE_EVT_SET_TX_PWR_COMPLETE:
        {
            break;
		}                       
            
        /**********************************************************
        *                       GAP Events
        ***********************************************************/
       
        /* This event indicates peripheral device has started/stopped
           advertising. */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {
            DEBUG_BLE("CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: ");
            if(Cy_BLE_GetConnectionState(appConnHandle) == CY_BLE_CONN_STATE_DISCONNECTED)
            {
                DEBUG_BLE(" <Restart ADV> \r\n");
                printf("Advertising...\r\n\n");
                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
                AccXNotificationEnabled = false;
                AccYNotificationEnabled = false;
                AccZNotificationEnabled = false;
                BattNotificationEnabled = false;
                TempNotificationEnabled = false;
                PresIndicationEnabled = false;
            }
            break;
        }
            
        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED', 
        if Link Layer Privacy is enabled in component customizer. */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
        {
            /* BLE link is established */
            /* This event will be triggered since link layer privacy is enabled */
            DEBUG_BLE("CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE \r\n");
            
            cy_stc_ble_gap_enhance_conn_complete_param_t *param = \
            (cy_stc_ble_gap_enhance_conn_complete_param_t *)eventParam; 
            
            printf("Connected to Device ");
            printf("%02X:%02X:%02X:%02X:%02X:%02X\r\n\n",param->peerBdAddr[5],\
                    param->peerBdAddr[4], param->peerBdAddr[3], param->peerBdAddr[2],\
                    param->peerBdAddr[1], param->peerBdAddr[0]);
         
            cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_ON);
            DEBUG_BLE("\r\nBDhandle : 0x%02X\r\n", param->bdHandle);
                
            break;
        }
        
        /* This event is triggered when there is a change to either the maximum Payload 
        length or the maximum transmission time of Data Channel PDUs in either direction */
        case CY_BLE_EVT_DATA_LENGTH_CHANGE:
        {
            break;
        }
        
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device. */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED: 
		{
			DEBUG_BLE("CY_BLE_EVT_GAP_DEVICE_CONNECTED \r\n");
            AccXNotificationEnabled = false;
            AccYNotificationEnabled = false;
            AccZNotificationEnabled = false;
            BattNotificationEnabled = false;
            TempNotificationEnabled = false;
            PresIndicationEnabled = false;
            break;
		}            
        /* This event is generated when disconnected from remote device or 
           failed to establish connection. */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:   
		{
			if(Cy_BLE_GetConnectionState(appConnHandle) == CY_BLE_CONN_STATE_DISCONNECTED)
            {
                DEBUG_BLE("CY_BLE_EVT_GAP_DEVICE_DISCONNECTED %d\r\n",\
                    CY_BLE_CONN_STATE_DISCONNECTED);
            }
            
            /* Device disconnected; restart advertisement */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
		}            
        /* This event is generated at the GAP Central and the peripheral end 
           after connection parameter update is requested from the host to 
           the controller. */
        case CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
		{
			DEBUG_BLE("CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE \r\n");
            break;
		}
        /* This event indicates completion of the Cy_BLE_SetPhy API*/
		case CY_BLE_EVT_SET_PHY_COMPLETE:
        {
            break;
        }
        /* This event indicates completion of the Cy_BLE_GetPhy API */
        case CY_BLE_EVT_GET_PHY_COMPLETE:
        {
            break;
        }
        /* This event indicates that the controller has changed the transmitter
           PHY or receiver PHY in use */
        case CY_BLE_EVT_PHY_UPDATE_COMPLETE:
        {
            break;
        }		
            
        /**********************************************************
        *                       GATT Events
        ***********************************************************/
            
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device. */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
        	appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            break;
        }    
        /* This event is generated at the GAP Peripheral end after disconnection. */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            DEBUG_BLE("CY_BLE_EVT_GATT_DISCONNECT_IND \r\n");
            cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);

            if(appConnHandle.bdHandle == (*(cy_stc_ble_conn_handle_t *)eventParam).bdHandle)
            {
                printf("Disconnected. \r\n\n");
                appConnHandle.bdHandle = CY_BLE_INVALID_CONN_HANDLE_VALUE;
                appConnHandle.attId    = CY_BLE_INVALID_CONN_HANDLE_VALUE;
            }
            
            break;
        }
        /* This event is triggered when 'GATT MTU Exchange Request' 
           received from GATT client device. */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
            break;
        }

        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
            break;
        }

        /*Indication confirmed*/
        case CY_BLE_EVT_GATTS_HANDLE_VALUE_CNF:
        {
        	PresIndicationReceived = true;
            break;
        }

        /* This event is triggered when there is a write request from the 
           Client device */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
            cy_stc_ble_gatt_write_param_t *write_req_param = (cy_stc_ble_gatt_write_param_t *)eventParam;
            cy_stc_ble_gatts_db_attr_val_info_t attr_param;

            attr_param.connHandle.bdHandle = write_req_param->connHandle.bdHandle;
            attr_param.connHandle.attId = write_req_param->connHandle.attId;
            attr_param.flags = CY_BLE_GATT_DB_PEER_INITIATED;
            attr_param.handleValuePair = write_req_param->handleValPair;
            attr_param.offset = 0;

			if(write_req_param->handleValPair.attrHandle == (CUSTOM_ACC_X_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    AccXNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("AccXNotificationEnabled = %d\r\n", AccXNotificationEnabled);
                    AccxNotificationPacket.connHandle = appConnHandle;
                    AccxNotificationPacket.handleValPair.attrHandle = CUSTOM_ACC_X_CHAR_HANDLE;
                    AccxNotificationPacket.handleValPair.value.val = (unsigned char*)&accx_value;
                    AccxNotificationPacket.handleValPair.value.len = sizeof(accx_value);

                    /*In case of if all CY_BLE_EVT_GATTS_WRITE_REQ are not received - workaround */
                    AccYNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    AccyNotificationPacket.connHandle = appConnHandle;
                    AccyNotificationPacket.handleValPair.attrHandle = CUSTOM_ACC_Y_CHAR_HANDLE;
                    AccyNotificationPacket.handleValPair.value.val = (unsigned char*)&accy_value;
                    AccyNotificationPacket.handleValPair.value.len = sizeof(accy_value);

                    /*In case of if all CY_BLE_EVT_GATTS_WRITE_REQ are not received - workaround */
                    AccZNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    AcczNotificationPacket.connHandle = appConnHandle;
                    AcczNotificationPacket.handleValPair.attrHandle = CUSTOM_ACC_Z_CHAR_HANDLE;
                    AcczNotificationPacket.handleValPair.value.val = (unsigned char*)&accz_value;
                    AcczNotificationPacket.handleValPair.value.len = sizeof(accz_value);
                }
            }

			if(write_req_param->handleValPair.attrHandle == (CUSTOM_ACC_Y_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    AccYNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("AccYNotificationEnabled = %d\r\n", AccYNotificationEnabled);
                    AccyNotificationPacket.connHandle = appConnHandle;
                    AccyNotificationPacket.handleValPair.attrHandle = CUSTOM_ACC_Y_CHAR_HANDLE;
                    AccyNotificationPacket.handleValPair.value.val = (unsigned char*)&accy_value;
                    AccyNotificationPacket.handleValPair.value.len = sizeof(accy_value);
                }
            }

			if(write_req_param->handleValPair.attrHandle == (CUSTOM_ACC_Z_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    AccZNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("AccZNotificationEnabled = %d\r\n", AccZNotificationEnabled);
                    AcczNotificationPacket.connHandle = appConnHandle;
                    AcczNotificationPacket.handleValPair.attrHandle = CUSTOM_ACC_Z_CHAR_HANDLE;
                    AcczNotificationPacket.handleValPair.value.val = (unsigned char*)&accz_value;
                    AcczNotificationPacket.handleValPair.value.len = sizeof(accz_value);
                }
            }

			if(write_req_param->handleValPair.attrHandle == (CUSTOM_BATT_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    BattNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("BattNotificationEnabled = %d\r\n", BattNotificationEnabled);
                    BattNotificationPacket.connHandle = appConnHandle;
                    BattNotificationPacket.handleValPair.attrHandle = CUSTOM_BATT_CHAR_HANDLE;
                    BattNotificationPacket.handleValPair.value.val = (unsigned char*)&batt_value;
                    BattNotificationPacket.handleValPair.value.len = sizeof(batt_value);
                }
            }

			if(write_req_param->handleValPair.attrHandle == (CUSTOM_TEMP_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    TempNotificationEnabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("AccZNotificationEnabled = %d\r\n", TempNotificationEnabled);
                    TempNotificationPacket.connHandle = appConnHandle;
                    TempNotificationPacket.handleValPair.attrHandle = CUSTOM_TEMP_CHAR_HANDLE;
                    TempNotificationPacket.handleValPair.value.val = (unsigned char*)&temp_formated;
                    TempNotificationPacket.handleValPair.value.len = sizeof(temp_formated);
                }
            }

			if(write_req_param->handleValPair.attrHandle == (CUSTOM_PRES_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    PresIndicationEnabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("PresIndicationEnabled = %d\r\n", PresIndicationEnabled);
                    PressIndicationPacket.connHandle = appConnHandle;
                    PressIndicationPacket.handleValPair.attrHandle = CUSTOM_PRES_CHAR_HANDLE;
                    PressIndicationPacket.handleValPair.value.val = (unsigned char*)&press_value;
                    PressIndicationPacket.handleValPair.value.len = sizeof(press_value);
                }
            }

            break;
        }
        /**********************************************************
        *                       Other Events
        ***********************************************************/
        default:
			break;
        
    }
}


void AccXNotification(void)
{
    cy_en_ble_api_result_t apiResult;
    apiResult = Cy_BLE_GATTS_Notification(&AccxNotificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error: 0x%2.2x \r\n", AccxNotificationPacket.handleValPair.attrHandle, apiResult);
    }

}

void AccYNotification(void)
{
    cy_en_ble_api_result_t apiResult;
    apiResult = Cy_BLE_GATTS_Notification(&AccyNotificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error: 0x%2.2x \r\n", AccyNotificationPacket.handleValPair.attrHandle, apiResult);
    }
    
}

void AccZNotification(void)
{
    cy_en_ble_api_result_t apiResult;
    apiResult = Cy_BLE_GATTS_Notification(&AcczNotificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error: 0x%2.2x \r\n", AcczNotificationPacket.handleValPair.attrHandle, apiResult);
    }

}

void BattNotification(void)
{
    cy_en_ble_api_result_t apiResult;
    apiResult = Cy_BLE_GATTS_Notification(&BattNotificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error: 0x%2.2x \r\n", BattNotificationPacket.handleValPair.attrHandle, apiResult);
    }

}

void TempNotification(void)
{
    cy_en_ble_api_result_t apiResult;
    apiResult = Cy_BLE_GATTS_Notification(&TempNotificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error: 0x%2.2x \r\n", TempNotificationPacket.handleValPair.attrHandle, apiResult);
    }

}

void PressIndication(void)
{
    cy_en_ble_api_result_t apiResult;
    static _Bool ind_wait = false;

    if(PresIndicationReceived)
    {
    	ind_wait = false;
    	PresIndicationReceived = false;
    }

    if(!ind_wait)
    {
    	apiResult = Cy_BLE_GATTS_Indication(&PressIndicationPacket);
    }

    if (apiResult == CY_BLE_SUCCESS)
    {
    	ind_wait = true;
    }
}

/*******************************************************************************
* Function Name: BlessInterrupt
********************************************************************************
* BLESS ISR
* It is used used when BLE middleware operates in BLE single CM4
*
*******************************************************************************/
/* BLESS ISR */
void BlessInterrupt(void)
{
    /* Call interrupt processing */
    Cy_BLE_BlessIsrHandler();
}

void IasEventHandler(uint32_t event, void *eventParam)
{
    /* Remove warning for unused parameter */
    (void)eventParam;
}

ieee_float32_t c_float32_to_ieee_11073_float32(float input)
{
	ieee_float32_t ret;
	_Bool mantissa_sign;

	if(input > 0)
	{
		mantissa_sign = false;
	}
	else if(input < 0)
	{
		mantissa_sign = true;
		input = input * -1;
	}
	else
	{
		ret.exponent = 0;
		ret.mantissa = 0;
		return ret;
	}

	if(input != input) //check for NAN
	{
		ret.exponent = 0;
		ret.mantissa = 0x07FE;
		return ret;
	}
	else if(input > (float)2E10) //check for +INF
	{
		ret.exponent = 0;
		ret.mantissa = 0x07FE;
		return ret;
	}
	else if(input < (float)-2E10) //check for -INF
	{
		ret.exponent = 0;
		ret.mantissa = 0x0802;
		return ret;
	}
	else
	{
		ret.exponent = 0;
		while(input > (float)0x7FF) /*Mantissa is limited to 12-bits*/
		{
			input /= 10;
			ret.exponent += 1;
			if(ret.exponent > 127)
			{
				ret.exponent = 0;
				ret.mantissa = 0x07FE;
				return ret;
			}
		}
		while(input*10 < (float)0x7FF) /*Mantissa is limited to 12-bits*/
		{
			input *= 10;
			ret.exponent -= 1;
			if(ret.exponent < -127)
			{
				ret.exponent = 0;
				ret.mantissa = 0x0802;
				return ret;
			}
		}

		if(mantissa_sign)
		{
			ret.mantissa = ((int32_t)input) & 0x00FFFFFF;
			ret.mantissa = ret.mantissa * -1;
			ret.exponent &= 0xff;
			return ret;
		}
		else
		{
			ret.mantissa = ((int32_t)input) & 0x00FFFFFF;
			ret.exponent &= 0xff;
			return ret;
		}
	}
}

int float32_to_ieee_11073_int(float input)
{
	ieee_float32_t value;
	uint32_t result = 0;

	value = c_float32_to_ieee_11073_float32(input);

	result = value.exponent << 24;
	result |= (value.mantissa & 0x00FFFFFF);

	return result;
}


/* [] END OF FILE */
