#include "bluetooth.h"
#include "uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define COMM_TX_BUFFER_SIZE 192U

#if COMM_UART_NUMBER == 0
#define COMM_UBRRH UBRR0H
#define COMM_UBRRL UBRR0L
#define COMM_UCSRB UCSR0B
#define COMM_UCSRC UCSR0C
#define COMM_UDR UDR0
#define COMM_RXEN RXEN0
#define COMM_TXEN TXEN0
#define COMM_UDRIE UDRIE0
#define COMM_UCSZ0 UCSZ00
#define COMM_UCSZ1 UCSZ01
#define COMM_UCSZ2 UCSZ02
#define COMM_USBS USBS0
#define COMM_UPM0 UPM00
#define COMM_UPM1 UPM01
#if defined(USART0_UDRE_vect)
#define COMM_UART_UDRE_VECTOR USART0_UDRE_vect
#elif defined(USART_UDRE_vect)
#define COMM_UART_UDRE_VECTOR USART_UDRE_vect
#else
#error "USART0 data-register-empty interrupt vector is not available."
#endif
#elif COMM_UART_NUMBER == 1
#define COMM_UBRRH UBRR1H
#define COMM_UBRRL UBRR1L
#define COMM_UCSRB UCSR1B
#define COMM_UCSRC UCSR1C
#define COMM_UDR UDR1
#define COMM_RXEN RXEN1
#define COMM_TXEN TXEN1
#define COMM_UDRIE UDRIE1
#define COMM_UCSZ0 UCSZ10
#define COMM_UCSZ1 UCSZ11
#define COMM_UCSZ2 UCSZ12
#define COMM_USBS USBS1
#define COMM_UPM0 UPM10
#define COMM_UPM1 UPM11
#define COMM_UART_UDRE_VECTOR USART1_UDRE_vect
#elif COMM_UART_NUMBER == 2
#define COMM_UBRRH UBRR2H
#define COMM_UBRRL UBRR2L
#define COMM_UCSRB UCSR2B
#define COMM_UCSRC UCSR2C
#define COMM_UDR UDR2
#define COMM_RXEN RXEN2
#define COMM_TXEN TXEN2
#define COMM_UDRIE UDRIE2
#define COMM_UCSZ0 UCSZ20
#define COMM_UCSZ1 UCSZ21
#define COMM_UCSZ2 UCSZ22
#define COMM_USBS USBS2
#define COMM_UPM0 UPM20
#define COMM_UPM1 UPM21
#define COMM_UART_UDRE_VECTOR USART2_UDRE_vect
#elif COMM_UART_NUMBER == 3
#define COMM_UBRRH UBRR3H
#define COMM_UBRRL UBRR3L
#define COMM_UCSRB UCSR3B
#define COMM_UCSRC UCSR3C
#define COMM_UDR UDR3
#define COMM_RXEN RXEN3
#define COMM_TXEN TXEN3
#define COMM_UDRIE UDRIE3
#define COMM_UCSZ0 UCSZ30
#define COMM_UCSZ1 UCSZ31
#define COMM_UCSZ2 UCSZ32
#define COMM_USBS USBS3
#define COMM_UPM0 UPM30
#define COMM_UPM1 UPM31
#define COMM_UART_UDRE_VECTOR USART3_UDRE_vect
#else
#error "COMM_UART_NUMBER must be 0, 1, 2, or 3."
#endif

static char turn_buffer[COMM_MAX_TURNS];
static volatile uint8_t turn_count = 0;

static volatile char tx_buffer[COMM_TX_BUFFER_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile uint8_t tx_count = 0;

static uint8_t comm_next_index(uint8_t index)
{
    index++;

    if (index >= COMM_TX_BUFFER_SIZE) {
        index = 0;
    }

    return index;
}

static uint8_t comm_decimal_length(uint8_t value)
{
    if (value >= 100U) {
        return 3U;
    }

    if (value >= 10U) {
        return 2U;
    }

    return 1U;
}

static uint8_t comm_sequence_length(void)
{
    if (turn_count == 0U) {
        return 0U;
    }

    return (uint8_t)((turn_count * 2U) - 1U);
}

static uint8_t comm_final_message_length(void)
{
    return (uint8_t)(7U + comm_decimal_length(turn_count) + 2U + 10U + comm_sequence_length() + 2U);
}

static uint8_t comm_tx_free_space(void)
{
    uint8_t free_space;
    uint8_t sreg_backup = SREG;

    cli();
    free_space = (uint8_t)(COMM_TX_BUFFER_SIZE - tx_count);
    SREG = sreg_backup;

    return free_space;
}

static uint8_t comm_queue_char(char data)
{
    uint8_t queued = 0;
    uint8_t sreg_backup = SREG;

    cli();

    if (tx_count < COMM_TX_BUFFER_SIZE) {
        tx_buffer[tx_head] = data;
        tx_head = comm_next_index(tx_head);
        tx_count++;
        COMM_UCSRB |= (1U << COMM_UDRIE);
        queued = 1;
    }

    SREG = sreg_backup;

    return queued;
}

static uint8_t comm_queue_string(const char *str)
{
    while (*str != '\0') {
        if (comm_queue_char(*str) == 0U) {
            return 0U;
        }

        str++;
    }

    return 1U;
}

static uint8_t comm_queue_uint8(uint8_t value)
{
    if (value >= 100U) {
        if (comm_queue_char((char)('0' + (value / 100U))) == 0U) {
            return 0U;
        }

        value %= 100U;
        if (comm_queue_char((char)('0' + (value / 10U))) == 0U) {
            return 0U;
        }

        return comm_queue_char((char)('0' + (value % 10U)));
    }

    if (value >= 10U) {
        if (comm_queue_char((char)('0' + (value / 10U))) == 0U) {
            return 0U;
        }

        return comm_queue_char((char)('0' + (value % 10U)));
    }

    return comm_queue_char((char)('0' + value));
}

static void comm_uart_init_registers(uint32_t baud_rate)
{
    uint16_t ubrr_value = (uint16_t)((F_CPU / (16UL * baud_rate)) - 1UL);

#if COMM_UART_NUMBER == 0
    uart_init(baud_rate);
#endif

    COMM_UBRRH = (uint8_t)(ubrr_value >> 8);
    COMM_UBRRL = (uint8_t)ubrr_value;

    COMM_UCSRB = 0;
    COMM_UCSRB |= (1U << COMM_RXEN) | (1U << COMM_TXEN);
    COMM_UCSRB &= (uint8_t)~(1U << COMM_UCSZ2);

    COMM_UCSRC = (1U << COMM_UCSZ1) | (1U << COMM_UCSZ0);
    COMM_UCSRC &= (uint8_t)~((1U << COMM_USBS) | (1U << COMM_UPM1) | (1U << COMM_UPM0));
}

void COMM_Init(void)
{
    uint8_t sreg_backup = SREG;

    cli();

    turn_count = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;

    comm_uart_init_registers(COMM_BAUD_RATE);

    SREG = sreg_backup;
    sei();
}

uint8_t COMM_LogTurn(char turn_direction)
{
    uint8_t logged = 0;
    uint8_t sreg_backup = SREG;

    if ((turn_direction != 'L') && (turn_direction != 'R')) {
        return 0U;
    }

    cli();

    if (turn_count < COMM_MAX_TURNS) {
        turn_buffer[turn_count] = turn_direction;
        turn_count++;
        logged = 1U;
    }

    SREG = sreg_backup;

    return logged;
}

uint8_t COMM_TransmitFinalData(void)
{
    uint8_t i;

    if (comm_tx_free_space() < comm_final_message_length()) {
        return 0U;
    }

    if (comm_queue_string("Turns: ") == 0U) {
        return 0U;
    }

    if (comm_queue_uint8(turn_count) == 0U) {
        return 0U;
    }

    if (comm_queue_string("\r\nSequence: ") == 0U) {
        return 0U;
    }

    for (i = 0; i < turn_count; i++) {
        if (i > 0U) {
            if (comm_queue_char(',') == 0U) {
                return 0U;
            }
        }

        if (comm_queue_char(turn_buffer[i]) == 0U) {
            return 0U;
        }
    }

    return comm_queue_string("\r\n");
}

void COMM_ResetTurns(void)
{
    uint8_t sreg_backup = SREG;

    cli();
    turn_count = 0;
    SREG = sreg_backup;
}

uint8_t COMM_IsTransmitBusy(void)
{
    return (comm_tx_free_space() != COMM_TX_BUFFER_SIZE) ? 1U : 0U;
}

uint8_t COMM_GetTurnCount(void)
{
    uint8_t count;
    uint8_t sreg_backup = SREG;

    cli();
    count = turn_count;
    SREG = sreg_backup;

    return count;
}

void COMM_Test(void)
{
    COMM_Init();

    while (1) {
        COMM_LogTurn('L');
        COMM_LogTurn('R');
        COMM_LogTurn('L');
        COMM_LogTurn('L');

        COMM_TransmitFinalData();
        COMM_ResetTurns();

        _delay_ms(3000);
    }
}

ISR(COMM_UART_UDRE_VECTOR)
{
    if (tx_count > 0U) {
        COMM_UDR = tx_buffer[tx_tail];
        tx_tail = comm_next_index(tx_tail);
        tx_count--;
    } else {
        COMM_UCSRB &= (uint8_t)~(1U << COMM_UDRIE);
    }
}
