/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.3 at Mon Apr 29 17:19:22 2019. */

#ifndef PB_ALEXAGADGETMUSICDATA_ALEXAGADGETMUSICDATATEMPODIRECTIVE_PB_H_INCLUDED
#define PB_ALEXAGADGETMUSICDATA_ALEXAGADGETMUSICDATATEMPODIRECTIVE_PB_H_INCLUDED
#include <pb.h>

#include "directiveHeader.pb.h"

#include "alexaGadgetMusicDataTempoDirectivePayload.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _alexaGadgetMusicData_TempoDirectiveProto_Directive {
    header_DirectiveHeaderProto header;
    alexaGadgetMusicData_TempoDirectivePayloadProto payload;
/* @@protoc_insertion_point(struct:alexaGadgetMusicData_TempoDirectiveProto_Directive) */
} alexaGadgetMusicData_TempoDirectiveProto_Directive;

typedef struct _alexaGadgetMusicData_TempoDirectiveProto {
    alexaGadgetMusicData_TempoDirectiveProto_Directive directive;
/* @@protoc_insertion_point(struct:alexaGadgetMusicData_TempoDirectiveProto) */
} alexaGadgetMusicData_TempoDirectiveProto;

/* Default values for struct fields */

/* Initializer values for message structs */
#define alexaGadgetMusicData_TempoDirectiveProto_init_default {alexaGadgetMusicData_TempoDirectiveProto_Directive_init_default}
#define alexaGadgetMusicData_TempoDirectiveProto_Directive_init_default {header_DirectiveHeaderProto_init_default, alexaGadgetMusicData_TempoDirectivePayloadProto_init_default}
#define alexaGadgetMusicData_TempoDirectiveProto_init_zero {alexaGadgetMusicData_TempoDirectiveProto_Directive_init_zero}
#define alexaGadgetMusicData_TempoDirectiveProto_Directive_init_zero {header_DirectiveHeaderProto_init_zero, alexaGadgetMusicData_TempoDirectivePayloadProto_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define alexaGadgetMusicData_TempoDirectiveProto_Directive_payload_tag 2
#define alexaGadgetMusicData_TempoDirectiveProto_Directive_header_tag 1
#define alexaGadgetMusicData_TempoDirectiveProto_directive_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t alexaGadgetMusicData_TempoDirectiveProto_fields[2];
extern const pb_field_t alexaGadgetMusicData_TempoDirectiveProto_Directive_fields[3];

/* Maximum encoded size of messages (where known) */
#define alexaGadgetMusicData_TempoDirectiveProto_size (50 + header_DirectiveHeaderProto_size)
#define alexaGadgetMusicData_TempoDirectiveProto_Directive_size (44 + header_DirectiveHeaderProto_size)

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define ALEXAGADGETMUSICDATATEMPODIRECTIVE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
