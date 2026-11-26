#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_SOF 0xAAu
#define UART_MAX_PAYLOAD 16u
#define DEMO_TIMEOUT_MS 50u
#define ARRAY_LENGTH(array) (sizeof(array) / sizeof((array)[0]))

typedef enum {
    WAIT_SOF,      /* Look for the start marker. */
    READ_CMD,      /* Save the command and start the XOR checksum. */
    READ_LEN,      /* Validate the payload length. */
    READ_PAYLOAD,  /* Save one payload byte per call. */
    READ_CHECKSUM  /* Compare the final byte with the computed XOR. */
} ParserState;

typedef enum {
    UART_TIMEOUT = -2, UART_BAD_CHECKSUM = -1,
    UART_IN_PROGRESS = 0, UART_FRAME_OK = 1
} FeedResult;

typedef struct {
    uint8_t command;
    uint8_t length;
    uint8_t payload[UART_MAX_PAYLOAD];
} UartFrame;

typedef struct {
    ParserState state;
    uint32_t timeout_ms;
    uint32_t last_byte_ms;
    uint8_t payload_index;
    uint8_t checksum;
    UartFrame frame;
} UartParser;

static UartParser uart_parser_init(uint32_t timeout_ms)
{
    UartParser parser = {0};

    parser.state = WAIT_SOF;
    parser.timeout_ms = timeout_ms;
    return parser;
}

/* A valid frame is available in parser->frame when UART_FRAME_OK is returned. */
static FeedResult uart_parser_feed(UartParser *parser, uint8_t byte,
                                   uint32_t arrival_ms)
{
    /* Check the gap before using this byte. After a timeout, the caller can
       feed it again as the possible start of a new frame. Unsigned
       subtraction also works when the millisecond clock wraps. */
    if (parser->state != WAIT_SOF && parser->timeout_ms != 0u &&
        (uint32_t)(arrival_ms - parser->last_byte_ms) > parser->timeout_ms) {
        parser->state = WAIT_SOF;
        return UART_TIMEOUT;
    }

    switch (parser->state) {
    case WAIT_SOF:
        if (byte != UART_SOF) {
            return UART_IN_PROGRESS;
        }
        parser->state = READ_CMD;
        break;
    case READ_CMD:
        parser->frame.command = byte;
        parser->checksum = byte;
        parser->state = READ_LEN;
        break;
    case READ_LEN:
        if (byte > UART_MAX_PAYLOAD) {
            parser->state = WAIT_SOF;
            return UART_IN_PROGRESS;
        }
        parser->frame.length = byte;
        parser->payload_index = 0u;
        parser->checksum ^= byte;
        parser->state = byte == 0u ? READ_CHECKSUM : READ_PAYLOAD;
        break;
    case READ_PAYLOAD:
        parser->frame.payload[parser->payload_index] = byte;
        ++parser->payload_index;
        parser->checksum ^= byte;
        if (parser->payload_index == parser->frame.length) {
            parser->state = READ_CHECKSUM;
        }
        break;
    case READ_CHECKSUM:
        parser->state = WAIT_SOF;
        if (byte != parser->checksum) {
            return UART_BAD_CHECKSUM;
        }
        parser->last_byte_ms = arrival_ms;
        return UART_FRAME_OK;
    }

    parser->last_byte_ms = arrival_ms;
    return UART_IN_PROGRESS;
}

typedef struct {
    size_t frames;
    size_t timeouts;
    size_t checksum_errors;
} StreamResult;

static StreamResult uart_feed_stream(UartParser *parser, const uint8_t *bytes,
                                     const uint32_t *arrival_times, size_t count)
{
    StreamResult summary = {0};

    for (size_t i = 0; i < count; ++i) {
        const uint8_t byte = bytes[i];
        const uint32_t arrival_ms = arrival_times[i];
        ParserState previous_state = parser->state;
        FeedResult result = uart_parser_feed(parser, byte, arrival_ms);

        if (result == UART_TIMEOUT) {
            ++summary.timeouts;
            printf("t=%3lums  byte=0x%02X  -> TIMEOUT (%lums gap > %lums) -- parser reset\n",
                   (unsigned long)arrival_ms, (unsigned int)byte,
                   (unsigned long)(uint32_t)(arrival_ms - parser->last_byte_ms),
                   (unsigned long)parser->timeout_ms);
            /* The late byte belongs to the new stream and may be a new SOF. */
            previous_state = parser->state;
            result = uart_parser_feed(parser, byte, arrival_ms);
        }

        printf("t=%3lums  byte=0x%02X  -> ", (unsigned long)arrival_ms,
               (unsigned int)byte);
        if (result == UART_FRAME_OK) {
            const UartFrame *frame = &parser->frame;
            ++summary.frames;
            printf("FRAME OK  CMD=0x%02X LEN=%u PAYLOAD=[",
                   (unsigned int)frame->command, (unsigned int)frame->length);
            for (uint8_t j = 0; j < frame->length; ++j) {
                printf("%s%02X", j == 0u ? "" : " ",
                       (unsigned int)frame->payload[j]);
            }
            puts("]");
        } else if (result == UART_BAD_CHECKSUM) {
            ++summary.checksum_errors;
            printf("CHECKSUM ERROR (expected 0x%02X) -- parser reset\n",
                   (unsigned int)parser->checksum);
        } else if (previous_state == READ_LEN && byte > UART_MAX_PAYLOAD) {
            printf("INVALID LENGTH (%u, max %u) -- parser reset\n",
                   (unsigned int)byte, UART_MAX_PAYLOAD);
        } else if (parser->state == WAIT_SOF) {
            puts("ignored (waiting for SOF)");
        } else {
            puts("receiving...");
        }
    }

    return summary;
}

/* Check how many frames and errors the demonstration produced. */
static int expect_counts(StreamResult actual, size_t frames, size_t timeouts,
                         size_t checksum_errors)
{
    if (actual.frames == frames && actual.timeouts == timeouts &&
        actual.checksum_errors == checksum_errors) {
        return 1;
    }

    fprintf(stderr, "Expected %zu frames, %zu timeouts, %zu checksum errors; "
                    "got %zu, %zu, %zu\n",
            frames, timeouts, checksum_errors,
            actual.frames, actual.timeouts, actual.checksum_errors);
    return 0;
}

int main(void)
{
    const uint8_t clean_bytes[] = {UART_SOF, 0x01, 0x03, 0x10, 0x20, 0x30, 0x02};
    const uint32_t clean_times[] = {0, 5, 10, 15, 20, 25, 30};

    const uint8_t recovery_bytes[] = {
        UART_SOF, 0x01, 0x03, 0x10, UART_SOF, 0x05, 0x01, 0x7F, 0x7B
    };
    const uint32_t recovery_times[] = {0, 5, 10, 15, 200, 205, 210, 215, 220};

    const uint8_t consecutive_bytes[] = {
        UART_SOF, 0x03, 0x01, 0x55, 0x57,
        UART_SOF, 0x04, 0x02, 0xAA, 0xBB, 0x17
    };
    const uint32_t consecutive_times[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};

    UartParser parser = uart_parser_init(DEMO_TIMEOUT_MS);
    StreamResult result;
    int passed = 1;

    puts("Test 1 -- Clean valid frame");
    result = uart_feed_stream(&parser, clean_bytes, clean_times,
                              ARRAY_LENGTH(clean_bytes));
    if (!expect_counts(result, 1, 0, 0)) {
        passed = 0;
    }

    puts("\nTest 2 -- Timeout mid-frame, then recovery");
    parser = uart_parser_init(DEMO_TIMEOUT_MS);
    result = uart_feed_stream(&parser, recovery_bytes, recovery_times,
                              ARRAY_LENGTH(recovery_bytes));
    if (!expect_counts(result, 1, 1, 0)) {
        passed = 0;
    }

    puts("\nTest 3 -- Two valid frames back-to-back");
    parser = uart_parser_init(DEMO_TIMEOUT_MS);
    result = uart_feed_stream(&parser, consecutive_bytes, consecutive_times,
                              ARRAY_LENGTH(consecutive_bytes));
    if (!expect_counts(result, 2, 0, 0)) {
        passed = 0;
    }

    puts("\nTest 4 -- Timeout disabled");
    puts("The gap is ignored. The partial frame continues to its checksum.");
    parser = uart_parser_init(0u);
    result = uart_feed_stream(&parser, recovery_bytes, recovery_times,
                              ARRAY_LENGTH(recovery_bytes));
    if (!expect_counts(result, 0, 0, 1)) {
        passed = 0;
    }

    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
