/**
 * allstar_firefly.c  —  Allstar Firefly 318ALD31K native SubGHz protocol
 *
 * Implements the SubGhzProtocol vtable for the Supertex ED-9 based gate remote.
 * Both the decoder state machine and encoder TX buffer are ported directly from
 * the proven FAP implementation; only the framing (alloc/free/serialize/etc.)
 * changes to match the native SubGHz plugin interface.
 *
 * See allstar_firefly.h for full protocol documentation.
 */

#include "allstar_firefly.h"

#include <lib/subghz/protocols/base.h>

#include <furi.h>
#include <furi_hal.h>
#include <flipper_format/flipper_format.h>
#include <string.h>

#define TAG "AllstarFirefly"

/* ─── Decoder instance ────────────────────────────────────────────────────── */

typedef enum {
    AfRxState_WaitGap,   /**< Waiting for inter-frame gap to sync            */
    AfRxState_Receiving, /**< Collecting symbols for the current frame       */
} AfRxState;

typedef struct {
    SubGhzProtocolDecoderBase base;

    /* State machine */
    AfRxState rx_state;
    uint8_t   rx_syms[AF_SYM_COUNT]; /**< 0 = L (short), 1 = H (long)       */
    uint8_t   rx_count;

    /* Decoded result */
    char dip[AF_BIT_COUNT + 1]; /**< '+', '-', '0', null-terminated          */
} SubGhzProtocolDecoderAllstarFirefly;

/* ─── Encoder instance ────────────────────────────────────────────────────── */

typedef struct {
    SubGhzProtocolEncoderBase base;

    char     dip[AF_BIT_COUNT + 1];
    uint32_t tx_buf[AF_TX_BUF_SIZE];
    uint32_t tx_size;
    uint32_t tx_pos;
} SubGhzProtocolEncoderAllstarFirefly;

/* ═══════════════════════════════════════════════════════════════════════════ */
/* Internal helpers                                                            */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Decode 18 raw symbols into a 9-char DIP string.
 * Returns true on success; false if any invalid (L,H) pair is found.
 */
static bool af_decode_symbols(const uint8_t* syms, char* dip) {
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) {
        uint8_t a = syms[i * 2];
        uint8_t b = syms[i * 2 + 1];
        if      (a == 1 && b == 1) dip[i] = '+';
        else if (a == 0 && b == 0) dip[i] = '-';
        else if (a == 1 && b == 0) dip[i] = '0';
        else return false; /* invalid (L,H) pair */
    }
    dip[AF_BIT_COUNT] = '\0';
    return true;
}

/**
 * Validate that a DIP string contains only '+', '-', '0' and is exactly
 * AF_BIT_COUNT characters long.
 */
static bool af_dip_valid(const char* dip) {
    if(!dip) return false;
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) {
        if(dip[i] != '+' && dip[i] != '-' && dip[i] != '0') return false;
    }
    return (dip[AF_BIT_COUNT] == '\0');
}

/**
 * Encode a DIP string as a uint32 for display purposes.
 * Treats the 9-position trinary code as a base-3 number, MSB first:
 *   '+' = 2,  '0' = 1,  '-' = 0
 * Maximum value: 3^9 - 1 = 19682 = 0x4CE2  (fits in 4 hex digits)
 */
static uint32_t af_dip_to_uint32(const char* dip) {
    uint32_t val = 0;
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) {
        val *= 3;
        if     (dip[i] == '+') val += 2;
        else if(dip[i] == '0') val += 1;
        /* '-' adds 0 */
    }
    return val;
}

/**
 * Build the flat TX pulse/gap duration buffer from a DIP string.
 *
 * Buffer layout (alternating HIGH/LOW starting with HIGH):
 *   even index = HIGH (pulse), odd index = LOW (gap)
 *
 * Per bit:
 *   '+' → [LONG_PULSE, SHORT_GAP, LONG_PULSE, SHORT_GAP]
 *   '-' → [SHORT_PULSE, LONG_GAP, SHORT_PULSE, LONG_GAP]
 *   '0' → [LONG_PULSE, SHORT_GAP, SHORT_PULSE, LONG_GAP]
 *
 * Last gap of each frame is stretched to AF_INTERFRAME_US.
 */
static uint32_t af_build_tx_buf(const char* dip, uint32_t* buf) {
    uint32_t pos = 0;
    for(uint32_t rep = 0; rep < AF_TX_REPEAT; rep++) {
        for(uint32_t bit = 0; bit < AF_BIT_COUNT; bit++) {
            bool     last = (bit == AF_BIT_COUNT - 1);
            char     c    = dip[bit];
            uint32_t p0, g0, p1, g1;

            if(c == '+') {
                p0 = AF_LONG_PULSE_US;  g0 = AF_SHORT_GAP_US;
                p1 = AF_LONG_PULSE_US;  g1 = AF_SHORT_GAP_US;
            } else if(c == '-') {
                p0 = AF_SHORT_PULSE_US; g0 = AF_LONG_GAP_US;
                p1 = AF_SHORT_PULSE_US; g1 = AF_LONG_GAP_US;
            } else { /* '0' */
                p0 = AF_LONG_PULSE_US;  g0 = AF_SHORT_GAP_US;
                p1 = AF_SHORT_PULSE_US; g1 = AF_LONG_GAP_US;
            }

            if(last) g1 = AF_INTERFRAME_US;

            buf[pos++] = p0;
            buf[pos++] = g0;
            buf[pos++] = p1;
            buf[pos++] = g1;
        }
    }
    return pos;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/* Decoder implementation                                                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void* subghz_protocol_decoder_allstar_firefly_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    SubGhzProtocolDecoderAllstarFirefly* instance =
        malloc(sizeof(SubGhzProtocolDecoderAllstarFirefly));
    instance->base.protocol = &subghz_protocol_allstar_firefly;
    instance->rx_state      = AfRxState_WaitGap;
    instance->rx_count      = 0;
    memset(instance->dip, '-', AF_BIT_COUNT);
    instance->dip[AF_BIT_COUNT] = '\0';
    return instance;
}

static void subghz_protocol_decoder_allstar_firefly_free(void* context) {
    furi_assert(context);
    free(context);
}

static void subghz_protocol_decoder_allstar_firefly_reset(void* context) {
    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    instance->rx_state = AfRxState_WaitGap;
    instance->rx_count = 0;
}

/**
 * Edge callback — ported directly from the FAP rx_callback / gdo0_edge_cb.
 *
 * Convention (Flipper SubGHz standard):
 *   level = NEW level after the edge
 *   duration = how long the PREVIOUS level lasted
 *   → level=false (just went LOW):  duration = pulse that just ended
 *   → level=true  (just went HIGH): duration = gap  that just ended
 *
 * State machine:
 *   WaitGap  : ignore pulses; large gap → Receiving
 *   Receiving: classify each pulse (H/L) with range validation;
 *              out-of-range pulse resets to WaitGap immediately;
 *              large gap with full 18 symbols → decode + callback
 */
static void subghz_protocol_decoder_allstar_firefly_feed(
    void* context,
    bool  level,
    uint32_t duration) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;

    if(level) {
        /* ── HIGH just ended (native convention) = pulse just ended ─────
         * duration = width of the HIGH pulse we just measured.           */
        if(instance->rx_state == AfRxState_Receiving) {
            uint8_t sym;
            if(duration >= AF_LONG_PULSE_MIN && duration <= AF_LONG_PULSE_MAX) {
                sym = 1u; /* H */
            } else if(duration >= AF_SHORT_PULSE_MIN && duration <= AF_SHORT_PULSE_MAX) {
                sym = 0u; /* L */
            } else {
                /* Out-of-range — noise, reset immediately */
                instance->rx_state = AfRxState_WaitGap;
                instance->rx_count = 0;
                return;
            }
            if(instance->rx_count < AF_SYM_COUNT) {
                instance->rx_syms[instance->rx_count++] = sym;
            }
        }
    } else {
        /* ── LOW just ended (native convention) = gap just ended ────────
         * duration = width of the LOW gap we just measured.              */
        if(duration >= AF_FRAME_THRESH_US) {
            if(instance->rx_state == AfRxState_Receiving &&
               instance->rx_count == AF_SYM_COUNT) {
                /* Complete 18-symbol frame — attempt decode */
                char decoded[AF_BIT_COUNT + 1];
                if(af_decode_symbols(instance->rx_syms, decoded)) {
                    memcpy(instance->dip, decoded, AF_BIT_COUNT + 1);
                    /* Notify the SubGHz app that we have a valid decode */
                    if(instance->base.callback) {
                        instance->base.callback(
                            &instance->base,
                            instance->base.context);
                    }
                }
                /* Back to WaitGap for clean re-sync on the next frame */
                instance->rx_state = AfRxState_WaitGap;
                instance->rx_count = 0;

            } else if(instance->rx_state == AfRxState_WaitGap) {
                /* Sync gap seen — start collecting the next frame */
                instance->rx_state = AfRxState_Receiving;
                instance->rx_count = 0;

            } else {
                /* Partial/corrupt frame — wait for clean gap */
                instance->rx_state = AfRxState_WaitGap;
                instance->rx_count = 0;
            }
        }
    }
}

static uint8_t subghz_protocol_decoder_allstar_firefly_get_hash_data(void* context) {
    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;
    /* Simple hash: XOR all DIP chars together */
    uint8_t hash = 0;
    for(uint8_t i = 0; i < AF_BIT_COUNT; i++) hash ^= (uint8_t)instance->dip[i];
    return hash;
}

static SubGhzProtocolStatus subghz_protocol_decoder_allstar_firefly_serialize(
    void* context,
    FlipperFormat* flipper_format,
    SubGhzRadioPreset* preset) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;

    /* Write standard header fields — the history system calls serialize()
     * on a blank FlipperFormat and does NOT pre-write these for us.
     * Deserialize is called after the framework has already consumed
     * Frequency/Preset/Protocol, so it correctly reads only Key.        */
    if(!flipper_format_write_uint32(flipper_format, "Frequency", &preset->frequency, 1)) {
        FURI_LOG_E(TAG, "Failed to write Frequency");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    if(!flipper_format_write_string(flipper_format, "Preset", preset->name)) {
        FURI_LOG_E(TAG, "Failed to write Preset");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    if(!flipper_format_write_string_cstr(
           flipper_format, "Protocol", SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME)) {
        FURI_LOG_E(TAG, "Failed to write Protocol");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    if(!flipper_format_write_string_cstr(flipper_format, "Key", instance->dip)) {
        FURI_LOG_E(TAG, "Failed to write Key");
        return SubGhzProtocolStatusErrorParserOthers;
    }
    return SubGhzProtocolStatusOk;
}

static SubGhzProtocolStatus subghz_protocol_decoder_allstar_firefly_deserialize(
    void* context,
    FlipperFormat* flipper_format) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;

    /* The framework reads header fields before calling us;
     * we only need to read the protocol-specific Key field. */
    FuriString* key_str = furi_string_alloc();
    if(!flipper_format_read_string(flipper_format, "Key", key_str)) {
        FURI_LOG_E(TAG, "Missing Key field");
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }

    const char* key_cstr = furi_string_get_cstr(key_str);
    if(strlen(key_cstr) != AF_BIT_COUNT || !af_dip_valid(key_cstr)) {
        FURI_LOG_E(TAG, "Invalid Key value: %s", key_cstr);
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }

    memcpy(instance->dip, key_cstr, AF_BIT_COUNT + 1);
    furi_string_free(key_str);
    return SubGhzProtocolStatusOk;
}

static void subghz_protocol_decoder_allstar_firefly_get_string(
    void* context,
    FuriString* output) {

    furi_assert(context);
    SubGhzProtocolDecoderAllstarFirefly* instance = context;

    /* cat_printf appends to the output string — the SubGHz app
     * pre-populates it with the timestamp before calling us.
     *
     * First line = what the history list displays:
     *   "Allstar Firefly 0x1A2B"
     * Subsequent lines = details screen body.                           */
    uint32_t code = af_dip_to_uint32(instance->dip);
    furi_string_cat_printf(
        output,
        "%s\r\n0x%04lX\r\n"
        "Freq: 318MHz  OOK\r\n"
        "1 2 3 4 5 6 7 8 9\r\n"
        "%c %c %c %c %c %c %c %c %c",
        SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME,
        (unsigned long)code,
        instance->dip[0], instance->dip[1], instance->dip[2],
        instance->dip[3], instance->dip[4], instance->dip[5],
        instance->dip[6], instance->dip[7], instance->dip[8]);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/* Encoder implementation                                                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void* subghz_protocol_encoder_allstar_firefly_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    SubGhzProtocolEncoderAllstarFirefly* instance =
        malloc(sizeof(SubGhzProtocolEncoderAllstarFirefly));
    instance->base.protocol = &subghz_protocol_allstar_firefly;
    memset(instance->dip, '-', AF_BIT_COUNT);
    instance->dip[AF_BIT_COUNT] = '\0';
    instance->tx_size = 0;
    instance->tx_pos  = 0;
    return instance;
}

static void subghz_protocol_encoder_allstar_firefly_free(void* context) {
    furi_assert(context);
    free(context);
}

static void subghz_protocol_encoder_allstar_firefly_stop(void* context) {
    UNUSED(context);
    /* Nothing to release — buffer is stack-allocated in the instance */
}

/**
 * Load a DIP code from a FlipperFormat (called when the user selects
 * a saved .sub file for retransmit, or when the SubGHz app builds a
 * send request from a decoded frame).
 *
 * Reads the "Key" field, validates it, then pre-builds the full TX buffer
 * so yield() can simply walk the array.
 */
static SubGhzProtocolStatus subghz_protocol_encoder_allstar_firefly_deserialize(
    void* context,
    FlipperFormat* flipper_format) {

    furi_assert(context);
    SubGhzProtocolEncoderAllstarFirefly* instance = context;

    /* Framework has already consumed the header; read our Key field. */
    FuriString* key_str = furi_string_alloc();
    if(!flipper_format_read_string(flipper_format, "Key", key_str)) {
        FURI_LOG_E(TAG, "Encoder: missing Key field");
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }

    const char* key_cstr = furi_string_get_cstr(key_str);
    if(strlen(key_cstr) != AF_BIT_COUNT || !af_dip_valid(key_cstr)) {
        FURI_LOG_E(TAG, "Encoder: invalid Key: %s", key_cstr);
        furi_string_free(key_str);
        return SubGhzProtocolStatusErrorParserOthers;
    }

    memcpy(instance->dip, key_cstr, AF_BIT_COUNT + 1);
    furi_string_free(key_str);

    /* Pre-build the complete TX buffer (all AF_TX_REPEAT frames) */
    instance->tx_size = af_build_tx_buf(instance->dip, instance->tx_buf);
    instance->tx_pos  = 0;

    return SubGhzProtocolStatusOk;
}

/**
 * Yield the next (level, duration) pair for transmission.
 *
 * The buffer holds all AF_TX_REPEAT frame repetitions pre-built as a flat
 * alternating HIGH/LOW array: even index = HIGH, odd index = LOW.
 * Returns level_duration_reset() when the entire burst is complete.
 */
static LevelDuration subghz_protocol_encoder_allstar_firefly_yield(void* context) {
    furi_assert(context);
    SubGhzProtocolEncoderAllstarFirefly* instance = context;

    if(instance->tx_pos >= instance->tx_size) {
        return level_duration_reset();
    }

    bool     lv  = (instance->tx_pos % 2 == 0); /* even = HIGH, odd = LOW */
    uint32_t dur = instance->tx_buf[instance->tx_pos++];
    return level_duration_make(lv, dur);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/* Protocol vtable                                                             */
/* ═══════════════════════════════════════════════════════════════════════════ */

const SubGhzProtocolDecoder subghz_protocol_decoder_allstar_firefly = {
    .alloc          = subghz_protocol_decoder_allstar_firefly_alloc,
    .free           = subghz_protocol_decoder_allstar_firefly_free,
    .feed           = subghz_protocol_decoder_allstar_firefly_feed,
    .reset          = subghz_protocol_decoder_allstar_firefly_reset,
    .get_hash_data  = subghz_protocol_decoder_allstar_firefly_get_hash_data,
    .serialize      = subghz_protocol_decoder_allstar_firefly_serialize,
    .deserialize    = subghz_protocol_decoder_allstar_firefly_deserialize,
    .get_string     = subghz_protocol_decoder_allstar_firefly_get_string,
};

const SubGhzProtocolEncoder subghz_protocol_encoder_allstar_firefly = {
    .alloc       = subghz_protocol_encoder_allstar_firefly_alloc,
    .free        = subghz_protocol_encoder_allstar_firefly_free,
    .deserialize = subghz_protocol_encoder_allstar_firefly_deserialize,
    .stop        = subghz_protocol_encoder_allstar_firefly_stop,
    .yield       = subghz_protocol_encoder_allstar_firefly_yield,
};

const SubGhzProtocol subghz_protocol_allstar_firefly = {
    .name    = SUBGHZ_PROTOCOL_ALLSTAR_FIREFLY_NAME,
    .type    = SubGhzProtocolTypeStatic,
    .flag    = SubGhzProtocolFlag_AM |
               SubGhzProtocolFlag_Decodable |
               SubGhzProtocolFlag_Load |
               SubGhzProtocolFlag_Save |
               SubGhzProtocolFlag_Send,
    .decoder = &subghz_protocol_decoder_allstar_firefly,
    .encoder = &subghz_protocol_encoder_allstar_firefly,
};