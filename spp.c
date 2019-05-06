/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * SPP Application for 2070X devices.
 *
 * SPP application uses SPP profile library to establish, terminate, send and receive SPP
 * data over BR/EDR. This sample supports single a single SPP connection.
 *
 * Following compilation flags are important for testing
 *  HCI_TRACE_OVER_TRANSPORT - configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE    - enables WICED_BT_TRACEs.  You can also modify makefile.mk to build
 *                             with _debug version of the library
 *  SEND_DATA_ON_INTERRUPT   - if defined, the app will send 1Meg of data on application button push
 *  SEND_DATA_ON_TIMEOUT     - if enabled, the app will send 4 bytes every second while session is up
 *  LOOPBACK_DATA            - if enabled, the app sends back received data
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Smart Ready (2070x) evaluation board into your computer
 * 2. Configure application using compile flags defined before
 * 3. Add a Bluetooth Device on Windows.  That should create an incoming and outgoing COM ports
 * 4. Use standard terminal emulation application to open the outgoing COM port and send/receive data
 * 
 * Features demonstrated
 *  - Use of SPP library
 *
 *  Note: This snippet app does not support WICED HCI Control and may use transport only for tracing.
 *  If you route traces to WICED HCI UART, use ClientControl app with baud rate equal to that 
 *  set in the wiced_transport_cfg_t structure below (currently set at HCI_UART_DEFAULT_BAUD, i.e. 3 Mbps).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sparcommon.h"
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_spp.h"
#include "wiced_hci.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_memory.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"

#include "eir\bluetooth_eir.h"
#include "sdp\sdp_db.h"

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "AlexaGadgetsProtobuf\directiveParser.pb.h"
#include "AlexaGadgetsProtobuf\alexaDiscoveryDiscoverResponseEvent.pb.h"
#include "AlexaGadgetsProtobuf\alexaDiscoveryDiscoverDirective.pb.h"



#define HCI_TRACE_OVER_TRANSPORT            1   // If defined HCI traces are send over transport/WICED HCI interface
#define SEND_DATA_ON_TIMEOUT                1   // If defined application sends 4 bytes of data every second
//#define LOOPBACK_DATA                     1   // If defined application loops back received data

#define WICED_EIR_BUF_MAX_SIZE              264
#define SPP_NVRAM_ID                        0x50

/* Max TX packet to be sent over SPP */
#define MAX_TX_BUFFER                       1017
#define TRANS_MAX_BUFFERS                   10
#define TRANS_UART_BUFFER_SIZE              1024
#define SPP_MAX_PAYLOAD                     1007

/*****************************************************************************
**  Structures
*****************************************************************************/

#define SPP_RFCOMM_SCN                  4

static void         spp_connection_up_callback(uint16_t handle, uint8_t* bda);
static void         spp_connection_down_callback(uint16_t handle);
static void         encode_alexa_discover_response_event();
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);

wiced_bt_spp_reg_t spp_reg =
{
    SPP_RFCOMM_SCN,                     /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    NULL,                               /* SPP connection establishment failed, not used because this app never initiates connection */
    NULL,                               /* SPP service not found, not used because this app never initiates connection */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};

wiced_transport_buffer_pool_t*  host_trans_pool;
uint16_t                        spp_handle;
wiced_timer_t                   app_tx_timer;

uint8_t pincode[4] = { 0x30, 0x30, 0x30, 0x30 };

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    { TRANS_UART_BUFFER_SIZE, 1},
    NULL,
    NULL,
    NULL
};
#endif

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t app_management_callback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                  app_write_eir(void);
static int                   bt_write_nvram(int nvram_id, int data_len, void *p_data);
static int                   bt_read_nvram(int nvram_id, void *p_data, int data_len);

#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif

//extern wiced_bt_rfcomm_result_t wiced_bt_rfcomm_init(uint32_t buffer_size, uint32_t buffer_cnt);

/*******************************************************************
 * Function Definitions
 ******************************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
void application_start()
{
    wiced_result_t result;

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, TRANS_MAX_BUFFERS);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must 
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE("APP Start\n");

    /* Initialize Stack and Register Management Callback */
    // Register call back and configuration with stack
    wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 * SPP application initialization is executed after BT stack initialization is completed.
 */
void application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    app_write_eir();

    // Initialize SPP library
    wiced_bt_spp_startup(&spp_reg);

#ifdef HCI_TRACE_OVER_TRANSPORT
    // There is a virtual HCI interface between upper layers of the stack and
    // the controller portion of the chip with lower layers of the BT stack.
    // Register with the stack to receive all HCI commands, events and data.
    wiced_bt_dev_register_hci_trace(app_trace_callback);
#endif
    /* create SDP records */
    set_sdp_db(0x0000, 0x0131);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    // This application will always configure device connectable and discoverable
    wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, 0x0012, 0x0800);
    wiced_bt_dev_set_connectability(BTM_CONNECTABLE, 0x0012, 0x0800);
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t               dev_status;
    wiced_bt_dev_pairing_info_t*        p_pairing_info;
    wiced_bt_dev_encryption_status_t*   p_encryption_status;
    int                                 bytes_written, bytes_read;
    wiced_bt_power_mgmt_notification_t* p_power_mgmt_notification;

    WICED_BT_TRACE("app_management_callback %d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        application_init();
        //WICED_BT_TRACE("Free mem:%d", cfa_mm_MemFreeBytes());
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PIN_REQUEST_EVT:
        WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* This application always confirms peer's attempt to pair */
        wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* This application supports only Just Works pairing */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;
        WICED_BT_TRACE("Pairing Complete: %d\n", p_pairing_info->br_edr.status);
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* This application supports a single paired host, we can save keys under the same NVRAM ID overwriting previous pairing if any */
        bt_write_nvram(SPP_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read existing key from the NVRAM  */
        if (bt_read_nvram(SPP_NVRAM_ID, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE("Power mgmt status event: bd (%B) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr, \
                p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
        break;

    default:
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;
    }
    return result;
}


/*
 *  Prepare extended inquiry response data.  Current version publishes device name and 16bit
 *  SPP service.
 */
void app_write_eir(void)
{
    uint8_t *pBuf;
    uint8_t len = 0;

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);

    construct_gadget_eir(pBuf, &len, "Corbit", 0x0000, 0x0131);

    wiced_bt_trace_array("EIR: ", pBuf, MIN(len, 100));

    wiced_bt_dev_write_eir(pBuf, len);

    return;
}

/*
 * SPP connection up callback
 */
void spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    WICED_BT_TRACE("%s handle:%d address:%B\n", __FUNCTION__, handle, bda);
    spp_handle = handle;
}

/*
 * SPP connection down callback
 */
void spp_connection_down_callback(uint16_t handle)
{
    WICED_BT_TRACE("%s handle:%d\n", __FUNCTION__, handle);
    spp_handle = 0;
}

void encode_alexa_discover_response_event()
{
    WICED_BT_TRACE("\nCreating discover response event:\n");
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    alexaDiscovery_DiscoverResponseEventProto discover_response_envelope =
           alexaDiscovery_DiscoverResponseEventProto_init_default;

    strcpy(discover_response_envelope.event.header.namespace, "Alexa.Discovery");
    strcpy(discover_response_envelope.event.header.name, "Discover.Response");

    discover_response_envelope.event.payload.endpoints_count = 1;
    strcpy(discover_response_envelope.event.payload.endpoints[0].endpointId, "4FE5C145B6E55247D1D6A012DF7A2B8FDB77EC9306A3D64061F46C638449D096");
    strcpy(discover_response_envelope.event.payload.endpoints[0].friendlyName, "Corbit");
    discover_response_envelope.event.payload.endpoints[0].capabilities_count = 3;
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[0].type, "test type 1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[0].interface, "Test interface 1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[0].version, "1.0");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[1].type, "test type 2");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[1].interface, "Test interface 2");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[1].version, "1.0");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[2].type, "test type 3");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[2].interface, "Test interface 3");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[2].version, "1.1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.firmwareVersion, "19");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.deviceToken, "f596574b18daf875610b1b549991e9a5175bf70bf6f65e1b458b6005b8d1628e");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.deviceTokenEncryptionType, "1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.amazonDeviceType, "ABOZTM8N18E1R");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.modelName, "Corbit");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.radioAddress, "0123456789AB");

    wiced_bool_t status = pb_encode(&stream, alexaDiscovery_DiscoverResponseEventProto_fields, &discover_response_envelope);
    if (!status)
    {
      printf("%s: Error encoding message\n", __FUNCTION__);
      return;
    }

    printf("bytes written:%zu\n", stream.bytes_written);
    uint8_t index;
    for(index = 0; index < stream.bytes_written; index++)
    {
        printf("0x%02x ", buffer[index]);
    }
    printf("\n");
    //decode_event(buffer, stream.bytes_written);
}

void strcpy(char *s, char *t) {

    while ((*s = *t) != '\0') {

        s++;

        t++;

    }

}

/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len)
{
    // step 1 : decode discovery directive

    // example data from alexa-gadgets-sample-code
    uint8_t notification_binary[] = {
         0x0a, 0x37, 0x0a, 0x1d, 0x0a, 0x0d, 0x4e, 0x6f, 0x74, 0x69,
         0x66, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x12,
         0x0c, 0x53, 0x65, 0x74, 0x49, 0x6e, 0x64, 0x69, 0x63, 0x61,
         0x74, 0x6f, 0x72, 0x12, 0x16, 0x08, 0x01, 0x10, 0x01, 0x1a,
         0x10, 0x0a, 0x08, 0x61, 0x73, 0x73, 0x65, 0x74, 0x49, 0x44,
         0x31, 0x12, 0x04, 0x75, 0x72, 0x6c, 0x31};

    WICED_BT_TRACE("\r\nDecoding Directive: \r\n");
    WICED_BT_TRACE("data length = %d: \r\n", data_len);

    for(int i=0; i<data_len;i++)
        WICED_BT_TRACE("0x%2x\r\n", p_data[i]);

    pb_istream_t stream = pb_istream_from_buffer(p_data, data_len);
//    pb_istream_t stream = pb_istream_from_buffer(notification_binary, sizeof(notification_binary));
    directive_DirectiveParserProto envelope = directive_DirectiveParserProto_init_default;

    wiced_bool_t result = pb_decode(&stream, directive_DirectiveParserProto_fields, &envelope);

    if(result)
    {
        WICED_BT_TRACE("decode directive - Success\r\n");
        WICED_BT_TRACE("name = %s, namespace=%s\n", envelope.directive.header.name, envelope.directive.header.namespace);
    }
    else
    {
        WICED_BT_TRACE("decode directive - Fail\r\n");
    }

    // step 2 : encode discovery response event

    WICED_BT_TRACE("\nCreating discover response event:\r\n");
    uint8_t buffer[256];
    pb_ostream_t stream2 = pb_ostream_from_buffer(buffer, sizeof(buffer));
    alexaDiscovery_DiscoverResponseEventProto discover_response_envelope =
           alexaDiscovery_DiscoverResponseEventProto_init_default;

    strcpy(discover_response_envelope.event.header.namespace, "Alexa.Discovery");
    WICED_BT_TRACE("%c \r\n", discover_response_envelope.event.header.namespace);
    strcpy(discover_response_envelope.event.header.name, "Discover.Response");

    discover_response_envelope.event.payload.endpoints_count = 1;
    strcpy(discover_response_envelope.event.payload.endpoints[0].endpointId, "4FE5C145B6E55247D1D6A012DF7A2B8FDB77EC9306A3D64061F46C638449D096");
    strcpy(discover_response_envelope.event.payload.endpoints[0].friendlyName, "Corbit");
    discover_response_envelope.event.payload.endpoints[0].capabilities_count = 3;
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[0].type, "test type 1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[0].interface, "Test interface 1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[0].version, "1.0");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[1].type, "test type 2");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[1].interface, "Test interface 2");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[1].version, "1.0");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[2].type, "test type 3");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[2].interface, "Test interface 3");
    strcpy(discover_response_envelope.event.payload.endpoints[0].capabilities[2].version, "1.1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.firmwareVersion, "19");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.deviceToken, "f596574b18daf875610b1b549991e9a5175bf70bf6f65e1b458b6005b8d1628e");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.deviceTokenEncryptionType, "1");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.amazonDeviceType, "ABOZTM8N18E1R");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.modelName, "Corbit");
    strcpy(discover_response_envelope.event.payload.endpoints[0].additionalIdentification.radioAddress, "0123456789AB");

    wiced_bool_t status = pb_encode(&stream2, alexaDiscovery_DiscoverResponseEventProto_fields, &discover_response_envelope);
    if (!status)
    {
        WICED_BT_TRACE("%s: Error encoding message\n", __FUNCTION__);
    }
//
//
//    wiced_bt_spp_send_session_data(handle, &buffer, stream2.bytes_written);
#if LOOPBACK_DATA
    //wiced_bt_spp_send_session_data(handle, p_data, data_len);
#endif
    return WICED_TRUE;
}

/*
 * Write NVRAM function is called to store information in the NVRAM.  
 */
static int bt_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
static int bt_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result);
    }
    return (read_bytes);
}


#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif
