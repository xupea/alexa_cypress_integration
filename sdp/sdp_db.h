// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Amazon Software License (the "License"). You may not use this file except in
// compliance with the License. A copy of the License is located at
//
//    http://aws.amazon.com/asl/
//
// or in the "license" file accompanying this file. This file is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, express or implied. See the License for the specific
// language governing permissions and limitations under the License.

#ifndef __SDP_DB_H__
#define __SDP_DB_H__

void set_sdp_db(uint16_t pid, uint16_t vid);

/* 128 bits UUID for legacy bluez devices */
/* 6088D2B3-983A-4EED-9F94-5AD1256816B7 */
#define SDP_GADGET_UUID128  0x60, 0x88, 0xD2, 0xB3, 0x98, 0x3A, 0x4E, 0xED, 0x9F, 0x94, 0x5A, 0xD1, 0x25, 0x68, 0x16, 0xB7

/* Macro to encode the header of a 128 bits UUID */
#define SDP_ATTR_UUID128      ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES)

/* Macro to declare a Service Class based on 128 bits UUID */
#define SDP_ATTR_CLASS_ID128                                   \
SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(17), \
SDP_ATTR_UUID128

//RFCOMM service for receiving ota from Echo device
#define OTA_RFCOMM_SCN 2

//RFCOMM service for receiving data from Echo device (Directives)
#define DIRECTIVE_RFCOMM_SCN 4

#define VENDOR_ID_LOCATION_MSB 252
#define VENDOR_ID_LOCATION_LSB 253
#define PRODUCT_ID_LOCATION_MSB 258
#define PRODUCT_ID_LOCATION_LSB 259


#endif
