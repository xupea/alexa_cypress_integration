/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.3 at Mon Apr 29 17:19:22 2019. */

#ifndef PB_HEADER_EVENTHEADER_PB_H_INCLUDED
#define PB_HEADER_EVENTHEADER_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _header_EventHeaderProto {
    char namespace[32];
    char name[32];
    char messageId[32];
/* @@protoc_insertion_point(struct:header_EventHeaderProto) */
} header_EventHeaderProto;

/* Default values for struct fields */

/* Initializer values for message structs */
#define header_EventHeaderProto_init_default     {"", "", ""}
#define header_EventHeaderProto_init_zero        {"", "", ""}

/* Field tags (for use in manual encoding/decoding) */
#define header_EventHeaderProto_namespace_tag    1
#define header_EventHeaderProto_name_tag         2
#define header_EventHeaderProto_messageId_tag    3

/* Struct field encoding specification for nanopb */
extern const pb_field_t header_EventHeaderProto_fields[4];

/* Maximum encoded size of messages (where known) */
#define header_EventHeaderProto_size             102

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define EVENTHEADER_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
