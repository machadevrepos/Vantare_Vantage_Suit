/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomBlepipeHdle;                    /**< BLEPipeService handle */
  uint16_t  CustomPipedatatxHdle;                  /**< PipeDataTx handle */
  uint16_t  CustomPipectrlrxHdle;                  /**< PipeControlRx handle */
  uint16_t  CustomPipectrltxHdle;                  /**< PipeControlTx handle */
  uint16_t  CustomPipestattxHdle;                  /**< PipeStatusTx handle */
  uint16_t  CustomPipecfgrwHdle;                  /**< PipeConfigRw handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}CustomContext_t;

extern uint16_t Connection_Handle;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint16_t SizePipedatatx = 244;
uint16_t SizePipectrlrx = 180;
uint16_t SizePipectrltx = 180;
uint16_t SizePipestattx = 180;
uint16_t SizePipecfgrw = 180;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define COPY_BLEPIPESERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x3f,0x88,0x10,0x00,0xb4,0xa5,0x4f,0x7c,0x9b,0x60,0x98,0xe0,0xb5,0xc8,0xa0,0x00)
#define COPY_PIPEDATATX_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x3f,0x88,0x10,0x01,0xb4,0xa5,0x4f,0x7c,0x9b,0x60,0x98,0xe0,0xb5,0xc8,0xa0,0x00)
#define COPY_PIPECONTROLRX_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x3f,0x88,0x10,0x02,0xb4,0xa5,0x4f,0x7c,0x9b,0x60,0x98,0xe0,0xb5,0xc8,0xa0,0x00)
#define COPY_PIPECONTROLTX_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x3f,0x88,0x10,0x03,0xb4,0xa5,0x4f,0x7c,0x9b,0x60,0x98,0xe0,0xb5,0xc8,0xa0,0x00)
#define COPY_PIPESTATUSTX_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x3f,0x88,0x10,0x04,0xb4,0xa5,0x4f,0x7c,0x9b,0x60,0x98,0xe0,0xb5,0xc8,0xa0,0x00)
#define COPY_PIPECONFIGRW_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x3f,0x88,0x10,0x05,0xb4,0xa5,0x4f,0x7c,0x9b,0x60,0x98,0xe0,0xb5,0xc8,0xa0,0x00)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_write_permit_req_event_rp0   *write_perm_req;
  aci_gatt_read_permit_req_event_rp0    *read_req;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */

  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomPipedatatxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_1_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_1_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPEDATATX_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPEDATATX_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomPipedatatxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomPipectrltxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3 */

            /* USER CODE END CUSTOM_STM_Service_1_Char_3 */

            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_attribute_modified  */

              /* USER CODE END CUSTOM_STM_Service_1_Char_3_attribute_modified  */

              /* Disabled Notification and Indication management */
              case (!(COMSVC_Notification) | !(COMSVC_Indication)):
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_Disabled_BEGIN  */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_Disabled_BEGIN  */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPECTRLTX_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPECTRLTX_INDICATE_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPECTRLTX_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_COMSVC_Notification_END */
                break;

              /* Enabled Indication management */
              case COMSVC_Indication:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_COMSVC_Indication_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_COMSVC_Indication_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPECTRLTX_INDICATE_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_COMSVC_Indication_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_COMSVC_Indication_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_3_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_3_default */
                break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomPipectrltxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomPipestattxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4 */

            /* USER CODE END CUSTOM_STM_Service_1_Char_4 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_1_Char_4_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_4_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPESTATTX_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_4_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_4_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPESTATTX_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_4_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_4_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomPipestattxHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomPipectrlrxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT;
            Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);

            /* USER CODE END CUSTOM_STM_Service_1_Char_2_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomPipectrlrxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomPipecfgrwHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_5_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            /* USER CODE END CUSTOM_STM_Service_1_Char_5_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomPipecfgrwHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
          if (read_req->Attribute_Handle == (CustomContext.CustomPipestattxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */

            /*USER CODE END CUSTOM_STM_Service_1_Char_4_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_1_Char_4_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_1_Char_4_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomPipestattxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (read_req->Attribute_Handle == (CustomContext.CustomPipecfgrwHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /*USER CODE BEGIN CUSTOM_STM_Service_1_Char_5_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1 */

            /*USER CODE END CUSTOM_STM_Service_1_Char_5_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_1*/
            aci_gatt_allow_read(read_req->Connection_Handle);
            /*USER CODE BEGIN CUSTOM_STM_Service_1_Char_5_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2 */

            /*USER CODE END CUSTOM_STM_Service_1_Char_5_ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE_2*/
          } /* if (read_req->Attribute_Handle == (CustomContext.CustomPipecfgrwHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          write_perm_req = (aci_gatt_write_permit_req_event_rp0*)blecore_evt->data;
          if (write_perm_req->Attribute_Handle == (CustomContext.CustomPipectrlrxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* Allow or reject a write request from a client using aci_gatt_write_resp(...) function */
            /*USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_PIPECTRLRX_WRITE_EVT;
            Notification.DataTransfered.Length = write_perm_req->Data_Length;
            Notification.DataTransfered.pPayload = write_perm_req->Data;
            Custom_STM_App_Notification(&Notification);
            (void)aci_gatt_permit_write(write_perm_req->Connection_Handle,
                                        write_perm_req->Attribute_Handle,
                                        0x00U,
                                        0x00U,
                                        write_perm_req->Data_Length,
                                        write_perm_req->Data);

            /*USER CODE END CUSTOM_STM_Service_1_Char_2_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE*/
          } /*if (write_perm_req->Attribute_Handle == (CustomContext.CustomPipectrlrxHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

          else if (write_perm_req->Attribute_Handle == (CustomContext.CustomPipecfgrwHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* Allow or reject a write request from a client using aci_gatt_write_resp(...) function */
            /*USER CODE BEGIN CUSTOM_STM_Service_1_Char_5_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE */

            /*USER CODE END CUSTOM_STM_Service_1_Char_5_ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE*/
          } /*if (write_perm_req->Attribute_Handle == (CustomContext.CustomPipecfgrwHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;

		case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */
          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.Custom_Evt_Opcode = CUSTOM_STM_NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          Custom_STM_App_Notification(&Notification);
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */
          break;
        }

        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          BLEPipeService
   *
   * Max_Attribute_Records = 1 + 2*5 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for BLEPipeService +
   *                                2 for PipeDataTx +
   *                                2 for PipeControlRx +
   *                                2 for PipeControlTx +
   *                                2 for PipeStatusTx +
   *                                2 for PipeConfigRw +
   *                                1 for PipeDataTx configuration descriptor +
   *                                1 for PipeControlTx configuration descriptor +
   *                                1 for PipeStatusTx configuration descriptor +
   *                              = 14
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 14;

  /* USER CODE BEGIN SVCCTL_InitService1 */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService1 */

  COPY_BLEPIPESERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomBlepipeHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: BLEPipe, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: BLEPipe , handle = 0x%04x \n\r", CustomContext.CustomBlepipeHdle);
  }

  /**
   *  PipeDataTx
   */
  COPY_PIPEDATATX_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomBlepipeHdle,
                          UUID_TYPE_128, &uuid,
                          SizePipedatatx,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                                   CHAR_VALUE_LEN_VARIABLE,
                                   &(CustomContext.CustomPipedatatxHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIPEDATATX, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIPEDATATX , handle = 0x%04x \n\r", CustomContext.CustomPipedatatxHdle);
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */
  /**
   *  PipeControlRx
   */
  COPY_PIPECONTROLRX_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomBlepipeHdle,
                          UUID_TYPE_128, &uuid,
                          SizePipectrlrx,
                          CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPipectrlrxHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIPECTRLRX, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIPECTRLRX , handle = 0x%04x \n\r", CustomContext.CustomPipectrlrxHdle);
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char2 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char2 */
  /**
   *  PipeControlTx
   */
  COPY_PIPECONTROLTX_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomBlepipeHdle,
                          UUID_TYPE_128, &uuid,
                          SizePipectrltx,
                          CHAR_PROP_NOTIFY | CHAR_PROP_INDICATE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPipectrltxHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIPECTRLTX, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIPECTRLTX , handle = 0x%04x \n\r", CustomContext.CustomPipectrltxHdle);
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char3 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char3 */
  /**
   *  PipeStatusTx
   */
  COPY_PIPESTATUSTX_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomBlepipeHdle,
                          UUID_TYPE_128, &uuid,
                          SizePipestattx,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPipestattxHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIPESTATTX, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIPESTATTX , handle = 0x%04x \n\r", CustomContext.CustomPipestattxHdle);
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char4 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char4 */
  /**
   *  PipeConfigRw
   */
  COPY_PIPECONFIGRW_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomBlepipeHdle,
                          UUID_TYPE_128, &uuid,
                          SizePipecfgrw,
                          CHAR_PROP_READ | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPipecfgrwHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIPECFGRW, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIPECFGRW , handle = 0x%04x \n\r", CustomContext.CustomPipecfgrwHdle);
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char5 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char5 */

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_PIPEDATATX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipedatatxHdle,
                                       0, /* charValOffset */
                                       SizePipedatatx, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPEDATATX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPEDATATX command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_PIPECTRLRX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipectrlrxHdle,
                                       0, /* charValOffset */
                                       SizePipectrlrx, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPECTRLRX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPECTRLRX command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_PIPECTRLTX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipectrltxHdle,
                                       0, /* charValOffset */
                                       SizePipectrltx, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPECTRLTX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPECTRLTX command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_3*/
      break;

    case CUSTOM_STM_PIPESTATTX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipestattxHdle,
                                       0, /* charValOffset */
                                       SizePipestattx, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPESTATTX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPESTATTX command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_4*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_4*/
      break;

    case CUSTOM_STM_PIPECFGRW:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipecfgrwHdle,
                                       0, /* charValOffset */
                                       SizePipecfgrw, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPECFGRW command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPECFGRW command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_5*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_5*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 * @param  size: Length of the characteristic value in octets
 *
 */
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_PIPEDATATX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipedatatxHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPEDATATX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPEDATATX command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/
      break;

    case CUSTOM_STM_PIPECTRLRX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipectrlrxHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPECTRLRX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPECTRLRX command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_2*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_2*/
      break;

    case CUSTOM_STM_PIPECTRLTX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipectrltxHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPECTRLTX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPECTRLTX command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_3*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_3*/
      break;

    case CUSTOM_STM_PIPESTATTX:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipestattxHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPESTATTX command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPESTATTX command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_4*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_4*/
      break;

    case CUSTOM_STM_PIPECFGRW:
      ret = aci_gatt_update_char_value(CustomContext.CustomBlepipeHdle,
                                       CustomContext.CustomPipecfgrwHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PIPECFGRW command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PIPECFGRW command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_5*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_5*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  Connection_Handle
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 *
 */
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_PIPEDATATX:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_1*/

      /* USER CODE END Updated_Length_Service_1_Char_1*/
      ret = Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomBlepipeHdle, CustomContext.CustomPipedatatxHdle, SizePipedatatx, pPayload);

      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : Generic_STM_App_Update_Char_Ext command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: Generic_STM_App_Update_Char_Ext command\n\r");
      }
      break;

    case CUSTOM_STM_PIPECTRLRX:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_2*/

      /* USER CODE END Updated_Length_Service_1_Char_2*/
      ret = Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomBlepipeHdle, CustomContext.CustomPipectrlrxHdle, SizePipectrlrx, pPayload);

      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : Generic_STM_App_Update_Char_Ext command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: Generic_STM_App_Update_Char_Ext command\n\r");
      }
      break;

    case CUSTOM_STM_PIPECTRLTX:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_3*/

      /* USER CODE END Updated_Length_Service_1_Char_3*/
      ret = Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomBlepipeHdle, CustomContext.CustomPipectrltxHdle, SizePipectrltx, pPayload);

      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : Generic_STM_App_Update_Char_Ext command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: Generic_STM_App_Update_Char_Ext command\n\r");
      }
      break;

    case CUSTOM_STM_PIPESTATTX:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_4*/

      /* USER CODE END Updated_Length_Service_1_Char_4*/
      ret = Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomBlepipeHdle, CustomContext.CustomPipestattxHdle, SizePipestattx, pPayload);

      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : Generic_STM_App_Update_Char_Ext command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: Generic_STM_App_Update_Char_Ext command\n\r");
      }
      break;

    case CUSTOM_STM_PIPECFGRW:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_5*/

      /* USER CODE END Updated_Length_Service_1_Char_5*/
      ret = Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomBlepipeHdle, CustomContext.CustomPipecfgrwHdle, SizePipecfgrw, pPayload);

      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : Generic_STM_App_Update_Char_Ext command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: Generic_STM_App_Update_Char_Ext command\n\r");
      }
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_2 */

  return ret;
}

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                       ServiceHandle,
                                       CharHandle,
                                       0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                       CharValueLen, /* charValueLen */
                                       0, /* value offset */
                                       243, /* value length */
                                       (uint8_t *)  pPayload);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 1, result : 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 1\n\r");
  }
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_Service_1_Char_1*/

  if (CharValueLen-243<=243)
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         CharValueLen-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 2, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 2\n\r");
    }
  }
  else
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 3, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 3\n\r");
    }
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243+243, /* value offset */
                                         CharValueLen-243-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 4, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 4\n\r");
    }
  }
  return ret;
}

