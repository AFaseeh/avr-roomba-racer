#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

#ifndef COMM_BAUD_RATE
#define COMM_BAUD_RATE 9600UL
#endif

#ifndef COMM_MAX_TURNS
#define COMM_MAX_TURNS 64U
#endif

/*
 * Arduino Uno/Nano use USART0 for the HC05 connection.
 * On Arduino Mega, COMM_UART_NUMBER can be set to 1, 2, or 3 later if the
 * Bluetooth module is moved to a spare hardware UART.
 */
#ifndef COMM_UART_NUMBER
#define COMM_UART_NUMBER 0
#endif

/*
 * Initializes the UART peripheral used by the HC05 Bluetooth module.
 *
 * This configures the selected USART for COMM_BAUD_RATE, 8 data bits,
 * no parity, and 1 stop bit. It also clears the internal turn log and
 * transmit queue.
 *
 * Call this once during system initialization, before the FSM starts.
 */
void COMM_Init(void);

/*
 * Logs one detected 90-degree turn into the internal turn buffer.
 *
 * turn_direction must be:
 *   'L' for a left turn
 *   'R' for a right turn
 *
 * This function only stores the turn. It does not transmit data.
 * why? well we don't want to burn CPU cycles while detecting turns 
 * we will wait till the end 
 * 
 * limitation we can only store up to 60 turns ONLY (will that's enough for a track? probably)
 *
 * Returns 1 if the turn was stored successfully.
 * Returns 0 if the direction is invalid or the turn buffer is full.
 * How can we handle this if we will? lol
 */
uint8_t COMM_LogTurn(char turn_direction);

/*
 * Formats and queues the final Bluetooth report for transmission.
 *
 * The transmitted format is:
 *   Turns: X
 *   Sequence: L,R,L
 *
 * This should be called by the FSM when the end of the track is detected.
 * Transmission is interrupt-driven and non-blocking after the message is
 * placed in the queue.
 *
 * Returns 1 if the full message was queued successfully.
 * Returns 0 if the transmit queue does not have enough free space.
 */
uint8_t COMM_TransmitFinalData(void);

/*
 * Clears the stored turn sequence and resets the turn count to zero.
 *
 * Use this before starting a new track run or after transmitting test data.
 * It does not clear any bytes that are already waiting in the transmit queue.
 * so this in intialization
 */
void COMM_ResetTurns(void);

/*
 * Checks whether the Bluetooth transmit queue is still sending data.
 *
 * helper function **** for testing
 * Returns 1 if queued bytes are still pending.
 * Returns 0 if the transmit queue is empty.
 */
uint8_t COMM_IsTransmitBusy(void);

/*
 * Returns the number of turns currently stored in the internal turn buffer.
 * helper function **** for testing
 */
uint8_t COMM_GetTurnCount(void);

/*
 * Runs a simple repeating Bluetooth test.
 *
 * The test initializes Bluetooth, logs the sequence L,R,L,L, transmits the
 * final report, resets the turn log, waits 3 seconds, and repeats forever.
 *
 * This is intended for manual HC05/UART verification from main().
 */
void COMM_Test(void);

#endif /* BLUETOOTH_H */
