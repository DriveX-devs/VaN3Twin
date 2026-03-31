/* This file serves the purpose of fixing an issue with asn_internal.h when
ASN_EMIT_DEBUG is specified without ASN_THREAD_SAFE */

#include "asn_internal.h"

int asn_debug_indent = 0;