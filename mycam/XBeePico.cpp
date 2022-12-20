
#include "XBeePico.h"

// uart0
static XBeePico* _xbee0 = NULL;

// uart1
static XBeePico* _xbee1 = NULL;


/**
Calculate and verify checksums
To test data integrity, the device calculates and verifies a checksum on non-escaped data.

To calculate the checksum of an API frame:
1. Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
2. Keep only the lowest 8 bits from the result.
3. Subtract this quantity from 0xFF.

To verify the checksum of an API frame:
1. Add all bytes including the checksum; do not include the delimiter and length.
2. If the checksum is correct, the last two digits on the far right of the sum equal 0xFF.

API_ID_INDEX : 3

https://www.digi.com/resources/documentation/Digidocs/90002002/Content/Tasks/t_calculate_checksum.htm?TocPath=API%20Operation%7CAPI%20frame%20format%7C_____1
 */

void __sendByte(uint8_t b, bool escape) {
    if (escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)) {
        uart_putc(UART_ID, ESCAPE);
        uart_putc(UART_ID, b ^ 0x20);
    } else {
        uart_putc(UART_ID, b);
    }
}

void __send(XBeeRequest &request){
    // the new new deal

    __sendByte(START_BYTE, false);

    // send length
    uint8_t msbLen = ((request.getFrameDataLength() + 2) >> 8) & 0xff;
    uint8_t lsbLen = (request.getFrameDataLength() + 2) & 0xff;

    __sendByte(msbLen, true);
    __sendByte(lsbLen, true);

    // api id
    __sendByte(request.getApiId(), true);
    __sendByte(request.getFrameId(), true);

    uint8_t checksum = 0;

    // compute checksum, start at api id
    checksum+= request.getApiId();
    checksum+= request.getFrameId();

    for (int i = 0; i < request.getFrameDataLength(); i++) {
        __sendByte(request.getFrameData(i), true);
        checksum+= request.getFrameData(i);
    }

    // perform 2s complement
    checksum = 0xff - checksum;

    // send checksum
    __sendByte(checksum, true);
}

bool
XBeePico::process_uart_rx(uint8_t ch){
    if(_pos > 0 && ch == START_BYTE) { // ATAP == 2
        _response.setErrorCode(UNEXPECTED_START_BYTE);
        if((_pos - 4) >= 0){
            _response.setFrameLength(_pos - 4);
        } else {
            _response.setFrameLength(0);
        }
        
        _onResponse.call(_response);
        _checksumTotal = 0;
        _pos = 0;
        return false;
    }

    if(_pos > 0 && ch == ESCAPE){
        _escape = true;
        return true;
    }
    if(_escape){
        ch = 0x20 ^ ch;
        _escape = false;
    }
    if (_pos >= API_ID_INDEX) {
        _checksumTotal+= ch;
    }

    //printf(" 0x%02x,", ch);

    // Length: Number of bytes between the length and the checksum -> Frame data
    switch(_pos){
        case 0:
            if(ch == START_BYTE) {
                _pos++;
            }
            break;
        case 1:
            _response.setMsbLength(ch);
            _pos++;
            break;
        case 2:
            _response.setLsbLength(ch);
            _pos++;
            break;
        case 3:
            _response.setApiId(ch);
            _pos++;
            break;
        default:
            if(_pos > MAX_FRAME_DATA_SIZE) {
                _response.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH);
                _checksumTotal = 0;
                return false;
            }
            if (_pos == (_response.getPacketLength() + 3)) {
                //printf("\n");
                //printf("0X%02x\n", _checksumTotal);
                if ((_checksumTotal & 0xff) == 0xff) {
                    _response.setChecksum(ch);
                    _response.setAvailable(true);
                    _response.setErrorCode(NO_ERROR);
                } else {
                    // checksum failed
                    _response.setErrorCode(CHECKSUM_FAILURE);
                }

                // minus 4 because we start after start,msb,lsb,api and up to but not including checksum
                // e.g. if frame was one byte, _pos=4 would be the byte, pos=5 is the checksum, where end stop reading
                _response.setFrameLength(_pos - 4);

                _onResponse.call(_response);

                //printf("received:%d:%d\n", _response.getFrameDataLength(), _response.getErrorCode());

                // reset state vars
                _pos = 0;
                _checksumTotal = 0;

                return true;
            } else {
                // add to packet array, starting with the fourth byte of the apiFrame
                _response.getFrameData()[_pos - 4] = ch;
                _pos++;
            }
    }
    return true;
}

// for uart0
static void on_uart_rx0(){
    while (uart_is_readable(uart0)) {
        uint8_t c = uart_getc(uart0);
        if(_xbee0 != NULL){
            _xbee0->process_uart_rx(c);
        }
    }
}

// for uart1
static void on_uart_rx1(){
    while (uart_is_readable(uart1)) {
        uint8_t c = uart_getc(uart1);
        if(_xbee1 != NULL){
            _xbee1->process_uart_rx(c);
        }
    }
}

XBeePico::XBeePico() {
    _nextFrameId = 0;
    _pos = 0;
    _escape = false;
    _checksumTotal = 0;
    _response.setFrameData(_responseFrameData);

    if(_uart_id == 0){
        uart_init(uart0, 115200);

        // Set the TX and RX pins by using the function select on the GPIO
        // Set datasheet for more information on function select
        gpio_set_function(_uart_tx_pin, GPIO_FUNC_UART);
        gpio_set_function(_uart_rx_pin, GPIO_FUNC_UART);

        int __unused actual = uart_set_baudrate(uart0, UART_BAUD_RATE);

        uart_set_hw_flow(uart0, false, false);

        uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY);

        uart_set_fifo_enabled(uart0, false);

        _xbee0 = this;

//        irq_set_exclusive_handler(UART0_IRQ, on_uart_rx0);
//        irq_set_enabled(UART0_IRQ, true);

//        uart_set_irq_enables(uart0, true, false);
    } else {
        uart_init(uart1, 115200);

        // Set the TX and RX pins by using the function select on the GPIO
        // Set datasheet for more information on function select
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

        int __unused actual = uart_set_baudrate(uart1, UART_BAUD_RATE);

        uart_set_hw_flow(uart1, false, false);

        uart_set_format(uart1, DATA_BITS, STOP_BITS, PARITY);

        uart_set_fifo_enabled(uart1, false);

        _xbee1 = this;

//        irq_set_exclusive_handler(UART1_IRQ, on_uart_rx0);
//        irq_set_enabled(UART1_IRQ, true);

//        uart_set_irq_enables(uart1, true, false);
    }
}

void
XBeePico::start() {
    if(_uart_id == 0){
        _xbee0 = this;

        irq_set_exclusive_handler(UART0_IRQ, on_uart_rx0);
        irq_set_enabled(UART0_IRQ, true);

        uart_set_irq_enables(uart0, true, false);
    } else {
        _xbee1 = this;

        irq_set_exclusive_handler(UART1_IRQ, on_uart_rx0);
        irq_set_enabled(UART1_IRQ, true);

        uart_set_irq_enables(uart1, true, false);
    }
}

void XBeePico::sendByte(uint8_t b, bool escape) {
    __sendByte(b, escape);
}

void XBeePico::send(XBeeRequest &request){
    __send(request);
}

uint8_t XBeePico::getNextFrameId(){
    this->_nextFrameId++;

    if (this->_nextFrameId == 0) {
        // can't send 0 because that disables status response
        this->_nextFrameId = 1;
    }

    return this->_nextFrameId;
}

