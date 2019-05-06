/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.3 at Mon Apr 29 17:19:23 2019. */

#ifndef PB_ALEXADISCOVERY_ALEXADISCOVERYDISCOVERRESPONSEEVENT_PB_H_INCLUDED
#define PB_ALEXADISCOVERY_ALEXADISCOVERYDISCOVERRESPONSEEVENT_PB_H_INCLUDED
#include <pb.h>

#include "eventHeader.pb.h"

#include "alexaDiscoveryDiscoverResponseEventPayload.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _alexaDiscovery_DiscoverResponseEventProto_Event {
    header_EventHeaderProto header;
    alexaDiscovery_DiscoverResponseEventPayloadProto payload;
/* @@protoc_insertion_point(struct:alexaDiscovery_DiscoverResponseEventProto_Event) */
} alexaDiscovery_DiscoverResponseEventProto_Event;

typedef struct _alexaDiscovery_DiscoverResponseEventProto {
    alexaDiscovery_DiscoverResponseEventProto_Event event;
/* @@protoc_insertion_point(struct:alexaDiscovery_DiscoverResponseEventProto) */
} alexaDiscovery_DiscoverResponseEventProto;

/* Default values for struct fields */

/* Initializer values for message structs */
#define alexaDiscovery_DiscoverResponseEventProto_init_default {alexaDiscovery_DiscoverResponseEventProto_Event_init_default}
#define alexaDiscovery_DiscoverResponseEventProto_Event_init_default {header_EventHeaderProto_init_default, alexaDiscovery_DiscoverResponseEventPayloadProto_init_default}
#define alexaDiscovery_DiscoverResponseEventProto_init_zero {alexaDiscovery_DiscoverResponseEventProto_Event_init_zero}
#define alexaDiscovery_DiscoverResponseEventProto_Event_init_zero {header_EventHeaderProto_init_zero, alexaDiscovery_DiscoverResponseEventPayloadProto_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define alexaDiscovery_DiscoverResponseEventProto_Event_payload_tag 2
#define alexaDiscovery_DiscoverResponseEventProto_Event_header_tag 1
#define alexaDiscovery_DiscoverResponseEventProto_event_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t alexaDiscovery_DiscoverResponseEventProto_fields[2];
extern const pb_field_t alexaDiscovery_DiscoverResponseEventProto_Event_fields[3];

/* Maximum encoded size of messages (where known) */
#define alexaDiscovery_DiscoverResponseEventProto_size (15371 + header_EventHeaderProto_size)
#define alexaDiscovery_DiscoverResponseEventProto_Event_size (15365 + header_EventHeaderProto_size)

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define ALEXADISCOVERYDISCOVERRESPONSEEVENT_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
