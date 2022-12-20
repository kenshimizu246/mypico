
/*

0x10 Transmit Request 
Transmits wireless data to the specified destination
#define ZB_TX_REQUEST 0x10
class ZBTxRequest : public PayloadRequest

0x8B Transmit Status
Indicates wireless data transmission success or failure
#define ZB_TX_STATUS_RESPONSE 0x8b
class ZBTxStatusResponse : public FrameIdResponse

*/

#ifndef XBeePico_h
#define XBeePico_h

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "XBee.h"

#define UART_ID uart0
//#define UART_ID uart1
//#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
//#define UART_TX_PIN 4
//#define UART_RX_PIN 5

#define ATAP 2

#define START_BYTE 0x7e
#define ESCAPE 0x7d
#define XON 0x11
#define XOFF 0x13

/**
 * Api Id constants
 */
#define TX_64_REQUEST 0x0
#define TX_16_REQUEST 0x1
#define AT_COMMAND_REQUEST 0x08
#define AT_COMMAND_QUEUE_REQUEST 0x09
#define REMOTE_AT_REQUEST 0x17
#define ZB_TX_REQUEST 0x10
#define ZB_EXPLICIT_TX_REQUEST 0x11
#define RX_64_RESPONSE 0x80
#define RX_16_RESPONSE 0x81
#define RX_64_IO_RESPONSE 0x82
#define RX_16_IO_RESPONSE 0x83
#define AT_RESPONSE 0x88
#define TX_STATUS_RESPONSE 0x89
#define MODEM_STATUS_RESPONSE 0x8a
#define ZB_RX_RESPONSE 0x90
#define ZB_EXPLICIT_RX_RESPONSE 0x91
#define ZB_TX_STATUS_RESPONSE 0x8b
#define ZB_IO_SAMPLE_RESPONSE 0x92
#define ZB_IO_NODE_IDENTIFIER_RESPONSE 0x95
#define AT_COMMAND_RESPONSE 0x88
#define REMOTE_AT_COMMAND_RESPONSE 0x97


/**
 * TX STATUS constants
 */
#define SUCCESS 0x0
#define CCA_FAILURE 0x2
#define INVALID_DESTINATION_ENDPOINT_SUCCESS 0x15
#define NETWORK_ACK_FAILURE 0x21
#define NOT_JOINED_TO_NETWORK 0x22
#define SELF_ADDRESSED 0x23
#define ADDRESS_NOT_FOUND 0x24
#define ROUTE_NOT_FOUND 0x25
#define PAYLOAD_TOO_LARGE 0x74
// Returned by XBeeWithCallbacks::waitForStatus on timeout
#define XBEE_WAIT_TIMEOUT 0xff

// modem status
#define HARDWARE_RESET 0
#define WATCHDOG_TIMER_RESET 1
#define ASSOCIATED 2
#define DISASSOCIATED 3
#define SYNCHRONIZATION_LOST 4
#define COORDINATOR_REALIGNMENT 5
#define COORDINATOR_STARTED 6

#define ZB_BROADCAST_RADIUS_MAX_HOPS 0

#define ZB_TX_UNICAST 0
#define ZB_TX_BROADCAST 8

#define AT_OK 0
#define AT_ERROR  1
#define AT_INVALID_COMMAND 2
#define AT_INVALID_PARAMETER 3
#define AT_NO_RESPONSE 4

#define NO_ERROR 0
#define CHECKSUM_FAILURE 1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH 2
#define UNEXPECTED_START_BYTE 3


#define INIT 0
#define READY 1
#define SENT 1
#define COMP 2
#define FAIL 2
#define ERROR 2

void sendByte(uint8_t b, bool escape);
void send(XBeeRequest &request);

/*
ZBTxRequest is derived from XBeeRequest and it has FrameId.

ZBTxStatusResponse is derived from FrameIdResponse
*/
class MyZBTxRequest : public ZBTxRequest {
public:
    MyZBTxRequest(){}
    MyZBTxRequest(const XBeeAddress64 &addr64, uint8_t *payload, uint8_t payloadLength):ZBTxRequest(addr64, payload, payloadLength){}
    MyZBTxRequest(const XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t *payload,
                  uint8_t payloadLength, uint8_t frameId):ZBTxRequest(addr64, addr16, broadcastRadius, option, payload, payloadLength, frameId){}

    void setStatus(uint8_t status){
        _status = status;
    }
    uint8_t getStatus(){ return _status; }

    void setXBeeErrorCode(uint8_t code){
        _xbeeErrorCode = code;
    }
    uint8_t getXBeeErrorCode(){ return _xbeeErrorCode; }

    void reset(){
        _status = INIT;
        _retryCount = 0;
        _xbeeErrorCode = 0;
    }
private:
    uint8_t _status = INIT;
    uint8_t _retryCount = 0;
    uint8_t _xbeeErrorCode = NO_ERROR;
    // UNEXPECTED_START_BYTE, PACKET_EXCEEDS_BYTE_ARRAY_LENGTH, NO_ERROR, CHECKSUM_FAILURE
};


class XBeePico {
public:
        XBeePico();

        template <typename Arg> struct Callback {
                void (*func)(Arg, uintptr_t);
                uintptr_t data;
                void set(void (*func)(Arg, uintptr_t), uintptr_t data) {
                        this->func = func;
                        this->data = data;
                }
                bool call(Arg arg) {
                        if (this->func) {
                                this->func(arg, this->data);
                                return true;
                        }
                        return false;
                }
        };

        /**
         * Sends a XBeeRequest (TX packet) out the serial port
         */
        void send(XBeeRequest &request);

        /**
         * Returns a sequential frame id between 1 and 255
         */
        uint8_t getNextFrameId();

        void onResponse(void (*func)(XBeeResponse&, uintptr_t), uintptr_t data = 0) { _onResponse.set(func, data); }
        void received(XBeeResponse& resp){
            if (resp.isAvailable()) {
                _onResponse.call(resp);
            }
        }

        void onAtCommandResponse(void (*func)(AtCommandResponse&, uintptr_t), uintptr_t data = 0) { _onAtCommandResponse.set(func, data); }

        void onPacketError(void (*func)(uint8_t, uintptr_t), uintptr_t data = 0) { _onPacketError.set(func, data); }

        void setUartId(uint8_t uart_id){ _uart_id = uart_id;}

        void start();

        bool process_uart_rx(uint8_t c);
private:
        uint8_t _responseFrameData[MAX_FRAME_DATA_SIZE];

        uint8_t _uart_tx_pin = 0;
        uint8_t _uart_rx_pin = 1;

        uint8_t _nextFrameId;
        uint8_t _uart_id = 0;

        void sendByte(uint8_t b, bool escape);

        Callback<XBeeResponse&> _onResponse;
        Callback<AtCommandResponse&> _onAtCommandResponse;
        Callback<uint8_t> _onPacketError;

        int _pos = 0;
        bool _escape = false;
        uint16_t _checksumTotal = 0;
        XBeeResponse _response;
};


#endif //XBeePico_h
