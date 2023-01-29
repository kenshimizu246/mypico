#include <stdio.h>
#include <cstdlib>

#include "pico/platform.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
//#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "ArduCAM.h"
#include "ov2640_regs.h"
#include "stdio.h"
#include "bsp/board.h"

#include "XBee.h"
#include "XBeePico.h"

//#include "detection_responder.h"
//#include "image_provider.h"
#include "model_settings.h"
#include "person_detect_model_data.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#define LED_PIN 25

#define CMD_HELLO 0x01
#define CMD_WRITE_REQUEST 0x02
#define CMD_WRITE_DATA 0x03
#define CMD_WRITE_DONE 0x04

#define CMD_CONFIG 0x11
#define CMD_RECV_STAT 0x12
#define CMD_WRITE_REQUEST_ACK 0x13
#define CMD_WRITE_DATA_ACK 0x14
#define CMD_WRITE_DONE_ACK 0x15
#define CMD_WRITE_RESEND 0x16
#define CMD_ARDUCAM_CMD 0x17

#define DATA_SIZE 80

namespace {
    tflite::ErrorReporter    *error_reporter = nullptr;
    const tflite::Model      *model          = nullptr;
    tflite::MicroInterpreter *interpreter    = nullptr;
    TfLiteTensor             *input          = nullptr;

    constexpr int  kTensorArenaSize = 54 * 1024 + 27 * 1024;
    static uint8_t tensor_arena[kTensorArenaSize];
}  // namespace


// manage state controll for send request
enum send_stat {
  ST_INIT,
  ST_SEND,
  ST_RECD,
  ST_RSND,
};

// store transmit request params.
struct xmit_status {
  uint32_t time;
  uint32_t seq;
  uint8_t * dat;
  send_stat stat = ST_INIT;
  uint8_t fid;
  uint8_t rid;
  size_t len;
  uint8_t count;
  alarm_id_t alarm_id;
};
typedef struct xmit_status xmit_status_t;

#define BUFF_SIZE 24
xmit_status_t stats[BUFF_SIZE];
uint32_t ack_cnt = 0; // count data received
uint8_t req_id = 7; // request id

struct ack_status {
  uint8_t fid;
  uint8_t rid;
  bool success;
};
typedef struct ack_status ack_status_t;


// next available index
uint8_t get_xmit_st_idx(uint8_t seq){
    int sqidx = seq % BUFF_SIZE;
    if(stats[sqidx].stat == ST_INIT || stats[sqidx].stat == ST_RECD){
        stats[sqidx].stat = ST_INIT;
        return sqidx;
    }
    for(uint8_t i = 1; i < BUFF_SIZE; i++){
        int s = seq - i;
        if(s < 0){
            s = BUFF_SIZE + s;
        }
        sqidx = s % BUFF_SIZE;
        if(stats[sqidx].stat == ST_INIT || stats[sqidx].stat == ST_RECD){
            stats[sqidx].stat = ST_INIT;
            return sqidx;
        }
    }
    return BUFF_SIZE;
}

uint8_t find_xmit_st_idx(uint8_t seq){
    int sqidx = seq % BUFF_SIZE;
    if(stats[sqidx].seq == seq){
        return sqidx;
    }
    for(uint8_t i = 0; i < BUFF_SIZE; i++){
        if(stats[i].seq == seq){
            return i;
        }
    }
    return BUFF_SIZE;
}

uint8_t find_xmit_st_fid(uint8_t fid){
    for(uint8_t i = 0; i < BUFF_SIZE; i++){
        if(stats[i].fid == fid){
            return i;
        }
    }
    return BUFF_SIZE;
}

uint8_t get_xmit_st_cnt(){
    uint8_t cnt = 0;
    for(uint8_t i = 0; i < BUFF_SIZE; i++){
        if(stats[i].stat == ST_SEND || stats[i].stat == ST_RSND){
            cnt++;
        }
    }
    return cnt;
}

bool is_xmit_st_open(){
    return (get_xmit_st_cnt() > 0);
}


// stores XBee Status Response 0x8B
struct xbee_response {
  uint8_t fid;
  uint8_t retry_cnt;
  uint8_t delivery_stat;
  uint8_t discovery_stat;
  bool success;
  bool timeout;
};
typedef struct xbee_response xbee_response_t;

XBeeAddress64 addr = XBeeAddress64(0x0013A200, 0x41C17206);

static uint8_t received = 0;

queue_t hello_ack_queue;
queue_t xbee_ack_queue;
queue_t request_ack_queue;
queue_t done_ack_queue;
queue_t missing_seq_queue;
queue_t arducam_cmd_queue;

// ------------------
//   Alarm Handlers
// ------------------

int64_t hello_timeout(alarm_id_t id, void *user_data) {
    uint8_t *statp = (uint8_t *)user_data;

    if(!queue_try_add(&hello_ack_queue, statp)){
      printf("fail to add user data in the hello_ack_queue %d\n", *statp);
      return 200;
    }

    return 0;
}

// application response missing
int64_t add_missing_queue(alarm_id_t id, void *user_data) {
    uint8_t *seq = (uint8_t *)user_data;

    if(!queue_try_add(&missing_seq_queue, seq)){
      printf("fail to add user data in the missing_seq_queue %d\n", *seq);
      return 200;
    }

    return 0;
}

// xbee response missing
int64_t xbee_resp_timeout(alarm_id_t id, void *user_data) {
    uint8_t *fidp = (uint8_t *)user_data; // need to check

    xbee_response_t xbee_resp;
    xbee_resp.fid = *fidp;
    xbee_resp.success = false;
    xbee_resp.timeout = true;
    if(!queue_try_add(&xbee_ack_queue, &xbee_resp)){
      printf("fail to add user data in the xbee_ack_queue %d\n", *fidp);
      return 200;
    }

    return 0;
}

// 
int64_t request_timeout(alarm_id_t id, void *user_data) {
    ack_status_t *stp = (ack_status_t *)user_data; // need to check

    stp->success = false;
    if(!queue_try_add(&request_ack_queue, stp)){
      printf("fail to add user data in the xbee_ack_queue %d:%d\n", stp->fid, stp->rid);
      return 200;
    }

    return 0;
}

int64_t done_timeout(alarm_id_t id, void *user_data) {
    uint8_t *ridp = (uint8_t *)user_data; // need to check

    if(!queue_try_add(&done_ack_queue, ridp)){
      printf("fail to add user data in the xbee_ack_queue %d\n", *ridp);
      return 200;
    }

    return 0;
}


// ------------------
//  Request Handlers
// ------------------

void hello_handler(uint8_t tt[]){
    uint8_t stat;

    printf("try to add queue from hello handler...!\n");
    stat = 1; // successfully received app ack from server
    queue_add_blocking(&hello_ack_queue, &stat);
    printf("done to add queue from hello handler...!\n");
}

void arducam_cmd_handler(uint8_t tt[]){
    uint8_t cmd;

    cmd = tt[1];
    queue_add_blocking(&arducam_cmd_queue, &cmd);
}

void write_data_ack_handler(uint8_t tt[]){
    uint8_t rid;
    uint32_t seq;

    rid = tt[1];
    seq = tt[2] << 24;
    seq |= tt[3] << 16;
    seq |= tt[4] << 8;
    seq |= tt[5];

    printf("ack_hndlr: %d %d\n", rid, seq);

    int sqidx = find_xmit_st_idx(seq);
    if(sqidx == BUFF_SIZE){
        printf("ack_hndlr: no buffer available for %d:%d\n", rid, seq);
        return;
    }
    stats[sqidx].stat = ST_RECD;
    ++ack_cnt;

    printf("ack_hndlr: %d %d %d\n", rid, seq, ack_cnt);

    if(!cancel_alarm(stats[sqidx].alarm_id)){
    }
}

void write_data_request_handler(uint8_t tt[]){
    uint8_t rid;
    uint32_t len;
    ack_status_t st;

    rid = tt[1];
    len = tt[2] << 24;
    len |= tt[3] << 16;
    len |= tt[4] << 8;
    len |= tt[5];

    st.rid = rid;
    st.success = true;
    queue_add_blocking(&request_ack_queue, &st);
}

void write_data_done_handler(uint8_t tt[]){
    uint8_t rid;
    uint32_t len;
    uint32_t seq;
    ack_status_t st;

    rid = tt[1];
    len = tt[2] << 24;
    len |= tt[3] << 16;
    len |= tt[4] << 8;
    len |= tt[5];
    seq = tt[6] << 24;
    seq |= tt[7] << 16;
    seq |= tt[8] << 8;
    seq |= tt[9];

    printf("done_hndlr: %d %d\n", rid, seq);

    st.success = true;
    st.rid = rid;
    queue_add_blocking(&done_ack_queue, &st);
}


// ------------------
//  Common Functions
// ------------------

bool send_msg(XBeePico& xbee, ZBTxRequest& tx){
    xbee_response_t xbee_resp;
    uint8_t fid = tx.getFrameId();

    //printf("before send:%d\n", fid);
    received = 0;
    // add cancel timer for xbee_resp blocking
    alarm_id_t aid = add_alarm_in_ms(200, xbee_resp_timeout, &fid, false);
    xbee.send(tx);
    //printf("after send:%d\n", fid);

    for(int i = 0; i < 10; i++){
        queue_remove_blocking(&xbee_ack_queue, &xbee_resp);
        if(xbee_resp.fid == fid){
            cancel_alarm(aid);
            if(xbee_resp.success){
                // printf("\nsuccess : %d\n", fid);
                return true;
            } else if(xbee_resp.timeout) {
                printf("\ntimeout : %d\n", fid);
                return false;
            } else {
                printf("\nunexpected : %d\n", fid);
                return false;
            }
        } else {
            printf("\nunexpected frame ID : %d : %d\n", fid, xbee_resp.fid);
            // queue_add_blocking(&xbee_ack_queue, &xbee_resp);
        }  
    }  
    return false;
}

// ------------------
//  Protocol Functions
// ------------------

bool send_hello(XBeePico& xbee)
{
  uint8_t fid = xbee.getNextFrameId();
  char payload[80];
  payload[0] = CMD_HELLO;
  sprintf((payload+1), "Hello Burst %d", fid);
  size_t len = strlen((payload + 1)) + 1;

  MyZBTxRequest tx = MyZBTxRequest(addr, (uint8_t*)payload, len);

  tx.setFrameId(fid);

  printf("Send Hello!\n");
  bool res = send_msg(xbee, tx);
  printf("Sent Hello!\n");
  return res;
}

bool send_write_request(XBeePico& xbee, uint8_t fid, uint8_t rid, uint32_t len, uint32_t pkt_cnt){
    printf("\nstart send_write_request %d:%d:%d:%d\n", fid, rid, len, pkt_cnt);

    uint8_t payload[10];

    payload[0] = CMD_WRITE_REQUEST;
    payload[1] = rid;
    payload[2] = (len >> 24) & 0xff;
    payload[3] = (len >> 16) & 0xff;
    payload[4] = (len >> 8) & 0xff;
    payload[5] = len & 0xff;
    payload[6] = (pkt_cnt >> 24) & 0xff;
    payload[7] = (pkt_cnt >> 16) & 0xff;
    payload[8] = (pkt_cnt >> 8) & 0xff;
    payload[9] = pkt_cnt & 0xff;
  
    ZBTxRequest tx = ZBTxRequest(addr, (uint8_t*)payload, sizeof(payload));

    tx.setFrameId(fid);

    bool res = send_msg(xbee, tx);

    //printf("end send_write_request %d:%d:%d:%d\n", fid, rid, len, pkt_cnt);

    return res;
}

bool send_write_data(XBeePico& xbee, uint8_t fid, uint8_t rid, uint32_t seq, uint8_t* dt, size_t len){

    uint8_t payload[len+6];

    printf("\nstart send_write_data [%d][%d][%d][%d]\n", fid, rid, seq, len);

    payload[0] = CMD_WRITE_DATA;
    payload[1] = rid;
    payload[2] = (seq >> 24) & 0xff;
    payload[3] = (seq >> 16) & 0xff;
    payload[4] = (seq >> 8) & 0xff;
    payload[5] = seq & 0xff;

    memcpy(payload + 6, dt, len);

    ZBTxRequest tx = ZBTxRequest(addr, (uint8_t*)payload, (len + 6));
    tx.setFrameId(fid);

    if(!send_msg(xbee, tx)){
        return false;
    }

    //printf("end send_write_data [%d][%d][%d][%d]\n", fid, rid, seq, len);

    return true;
}

void send_write_done(XBeePico& xbee, uint8_t fid, uint8_t rid, uint32_t len, uint32_t pkt_cnt){
    printf("\nstart send_write_done %d %d\n", len, pkt_cnt);

    uint8_t payload[10];
  
    payload[0] = CMD_WRITE_DONE;
    payload[1] = rid;
    payload[2] = (len >> 24) & 0xff;
    payload[3] = (len >> 16) & 0xff;
    payload[4] = (len >> 8) & 0xff;
    payload[5] = len & 0xff;
    payload[6] = (pkt_cnt >> 24) & 0xff;
    payload[7] = (pkt_cnt >> 16) & 0xff;
    payload[8] = (pkt_cnt >> 8) & 0xff;
    payload[9] = pkt_cnt & 0xff;
  
    ZBTxRequest tx = ZBTxRequest(addr, (uint8_t*)payload, sizeof(payload));
    tx.setFrameId(fid);

    send_msg(xbee, tx);
    printf("end send_write_done %d\n", fid);
}


// ------------------
//  
// ------------------
void process_missing(XBeePico& xbee){
    uint32_t mseq;
    //printf("process_missing start\n");
    while(queue_try_remove(&missing_seq_queue, &mseq)){
        printf("missing : %d\n", mseq);
        int msqidx = find_xmit_st_idx(mseq);
        if(msqidx == BUFF_SIZE){
            queue_add_blocking(&missing_seq_queue, &mseq);
            printf("no buff available for missing: %d\n", mseq);
            continue;
        }
        if(stats[msqidx].stat == ST_SEND || stats[msqidx].stat == ST_RSND){
            uint32_t seq;

            seq = stats[msqidx].seq;
            stats[msqidx].stat == ST_RSND;
            stats[msqidx].alarm_id = add_alarm_in_ms(200, add_missing_queue, &stats[msqidx].seq, false);
            if(!send_write_data(xbee, stats[msqidx].fid, stats[msqidx].rid, stats[msqidx].seq, stats[msqidx].dat, stats[msqidx].len)){
                queue_add_blocking(&missing_seq_queue, &stats[msqidx].seq);
                if(!cancel_alarm(stats[msqidx].alarm_id)){
                    printf("cancel_alarm returns false: %d %d\n", seq, stats[msqidx].alarm_id);
                }
            }
        } else {
            printf("process_missing ... %d %d\n", stats[msqidx].seq, stats[msqidx].stat);
        }
    }
    //printf("process_missing end\n");
}

void send_picture_by_xbee(XBeePico& xbee, uint8_t * buff, const int len){
    int wrote = 0;
    int seq = 0;

    printf("\nstart send_picture_by_xbee %d\n", len);

    size_t hdr_size = 6;
    size_t data_size = DATA_SIZE - hdr_size;
    int pkt_cnt = 0;

    // init Data Ack Counter
    ack_cnt = 0;

    if((len % data_size) > 0){
        pkt_cnt = (int)(len / data_size) + 1;
    } else {
        pkt_cnt = (int)(len / data_size);
    }

    uint8_t fid, rid;
    alarm_id_t aid;
    ack_status_t st;
    bool res;
    st.success = false;
    while(!st.success) {
        fid = xbee.getNextFrameId();
        rid = req_id++;
        st.fid = fid;
        st.rid = rid;
        aid = add_alarm_in_ms(200, request_timeout, &st, false);
        if(!send_write_request(xbee, fid, rid, len, pkt_cnt)){
            if(!cancel_alarm(aid)){
                printf("cancel write request timeout 1! [%d]\n", aid);
            }
            continue;
        }

        // wait response from server to get ready to receive data.
        queue_remove_blocking(&request_ack_queue, &st);
        if(st.success){
            if(!cancel_alarm(aid)){
                printf("cancel write request timeout 2! [%d]\n", aid);
            }
        }
    }

    uint32_t mseq;
    uint8_t * b = buff;
    int l = len;
    size_t s = 0;
    for(int seq = 0; seq < pkt_cnt; seq++){
        int sqidx = BUFF_SIZE;
        uint8_t sqidx_cnt = 0;
        while(1){
            sqidx = get_xmit_st_idx(seq);

            if(sqidx == BUFF_SIZE){
                process_missing(xbee);
                printf(".");
                sleep_ms(100);
                if(sqidx_cnt++ > 100){
                    return;
                }
            } else {
                break;
            }
        }

        s = min(l, data_size);
        fid = xbee.getNextFrameId();
        stats[sqidx].time = time_us_32();
        stats[sqidx].seq = seq;
        stats[sqidx].len = s;
        stats[sqidx].dat = b;
        stats[sqidx].fid = fid;
        stats[sqidx].rid = rid;
        stats[sqidx].stat = ST_SEND;
        stats[sqidx].alarm_id = add_alarm_in_ms(200, add_missing_queue, &stats[sqidx].seq, false);

        if(!send_write_data(xbee, fid, rid, seq, b, s)){
            if(!cancel_alarm(stats[sqidx].alarm_id)){
                printf("cancel_alarm returns false: %d %d\n", seq, stats[sqidx].alarm_id);
            }
            queue_add_blocking(&missing_seq_queue, &seq);
        }

        process_missing(xbee);

        b = b + s;
        l = l - s;
    }
    process_missing(xbee);

    fid = xbee.getNextFrameId();
    st.fid = fid;
    st.success = false;
    alarm_id_t done_aid = add_alarm_in_ms(200, done_timeout, &st, false);
    send_write_done(xbee, fid, rid, len, pkt_cnt);

    queue_remove_blocking(&done_ack_queue, &st);
    if(!cancel_alarm(done_aid)){
    }

    printf("end send_picture_by_xbee\n");
}

// ----------------------------
//  Callback Function for XBee
// ----------------------------

void func(XBeeResponse& resp, uintptr_t ptr){
    gpio_put(LED_PIN, 0);
    //printf("+===+\n");
    //printf("rcvd:api=0x%02x:avail=%d:isErr=%d:ecode=%d!\n",
    //        resp.getApiId(), resp.isAvailable(), resp.isError(), resp.getErrorCode());
    received = 1;

    switch(resp.getApiId()){
        case ZBTxStatusResponse::API_ID: {
            // Extended Transmit Status - 0x8B
            // https://www.digi.com/resources/documentation/Digidocs/90001480/reference/r_frame_0x8b.htm?TocPath=API%20frames%7C_____11
            ZBTxStatusResponse stat;
            xbee_response_t xbee_resp;
            resp.getZBTxStatusResponse(stat);

            xbee_resp.fid = stat.getFrameId();
            xbee_resp.retry_cnt = stat.getTxRetryCount();
            xbee_resp.delivery_stat = stat.getDeliveryStatus();
            xbee_resp.discovery_stat = stat.getDiscoveryStatus();
            xbee_resp.success = stat.isSuccess();

            //printf("stat:fid=%d:retry=%d:dlvry=%d:dscvry=%d:sucs=%d\n",
            //        stat.getFrameId(), stat.getTxRetryCount(), stat.getDeliveryStatus(), stat.getDiscoveryStatus(), stat.isSuccess());
            uint8_t fid = stat.getFrameId();
            queue_add_blocking(&xbee_ack_queue, &xbee_resp);
            break;
        }
        case ZBExplicitRxResponse::API_ID: {
            // Explicit Receive Indicator - 0x91
            // https://www.digi.com/resources/documentation/Digidocs/90001480/reference/r_frame_0x91.htm?TocPath=API%20frames%7C_____17
            ZBExplicitRxResponse rx;
            resp.getZBExplicitRxResponse(rx);
            uint8_t tt[MAX_FRAME_DATA_SIZE];
            memcpy(tt, rx.getData(), rx.getDataLength());
            tt[rx.getDataLength()] = 0;
            // printf("data:%d:%d:%s\n", rx.getDataLength(), tt[0], (tt + 1));
            switch(tt[0]){
                case CMD_HELLO: // 0x01
                    hello_handler(tt);
                    break;
                case CMD_CONFIG: // 0x11
                    break;
                case CMD_RECV_STAT: // 0x12
                    break;
                case CMD_WRITE_REQUEST_ACK: // 0x13
                    write_data_request_handler(tt);
                    break;
                case CMD_WRITE_DATA_ACK: // 0x14
                    write_data_ack_handler(tt);
                    break;
                case CMD_WRITE_DONE_ACK: // 0x15
                    write_data_done_handler(tt);
                    break;
                case CMD_WRITE_RESEND: // 0x16
                    break;
                case CMD_ARDUCAM_CMD: // 0x17
                    arducam_cmd_handler(tt);
                    break;
                default:
                    break;
            }
            break;
        }
    }
    //printf("-===-\n\n");
    stdio_flush();
}


#define BMPIMAGEOFFSET 66
uint8_t bmp_header[BMPIMAGEOFFSET] =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};

// set pin 10 as the slave select for the digital pot:
const uint8_t CS = 5;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;
ArduCAM myCAM( OV2640, CS );
uint8_t read_fifo_burst(ArduCAM myCAM);
uint8_t read_fifo_burst_xbee(ArduCAM myCAM, XBeePico& xbee);

int main() 
{
  int value=0;
  uint8_t vid, pid;
  uint8_t cameraCommand;
  uint8_t fid = 0;

  stdio_init_all();

  printf("after stdio_init_all!");

  queue_init(&hello_ack_queue, sizeof(uint8_t), 24);
  queue_init(&xbee_ack_queue, sizeof(xbee_response), 24);
  queue_init(&request_ack_queue, sizeof(ack_status_t), 8);
  queue_init(&done_ack_queue, sizeof(ack_status_t), 8);
  queue_init(&missing_seq_queue, sizeof(uint32_t), 24);
  queue_init(&arducam_cmd_queue, sizeof(uint8_t), 8);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  static tflite::MicroMutableOpResolver<5> micro_op_resolver;
  micro_op_resolver.AddAveragePool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddDepthwiseConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
    model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return -2;
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);


  gpio_put(LED_PIN, 1);

  XBeePico xbee = XBeePico();
  xbee.onResponse(func);

  xbee.start();

  sleep_ms(100);

  gpio_put(LED_PIN, 0);

  // put your setup code here, to run once:
  myCAM.Arducam_init();
  gpio_init(CS);
  gpio_set_dir(CS, GPIO_OUT);
  gpio_put(CS, 1);

  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  sleep_ms(100);
  myCAM.write_reg(0x07, 0x00);
  sleep_ms(100);

  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    cameraCommand = myCAM.read_reg(ARDUCHIP_TEST1);
    if (cameraCommand != 0x55) {
      printf(" SPI interface Error!");
      sleep_ms(1000); continue;
    } else {
      printf("ACK CMD SPI interface OK.END\n"); break;
    }
  }
  
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      printf("Can't find OV2640 module!");
      sleep_ms(1000); continue;
    }
    else {
      printf("OV2640 detected.END\n"); break;
    }
  }

  //Change to JPEG capture mode and initialize the OV5642 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  //myCAM.OV2640_set_JPEG_size(OV2640_160x120);
  sleep_ms(1000);
  myCAM.clear_fifo_flag();

  alarm_id_t hello_alarm_id;
  uint8_t hello_fid = 2;
  printf("start hello!\n");
  while(hello_fid != 1){ // hello_fid becomes 1 if it can receive message from server.
      printf("start hello timeout...\n");
      hello_fid = 2; // 2:failed 1:success
      hello_alarm_id = add_alarm_in_ms(1000, hello_timeout, &hello_fid, false);
      printf("hello timeout id %d!\n", hello_alarm_id);
      send_hello(xbee);
      queue_remove_blocking(&hello_ack_queue, &hello_fid);
      printf("remove hello_ack_queue %d\n", hello_fid);
      if(hello_fid == 1){
        if(!cancel_alarm(hello_alarm_id)){
        }
      }
  }
  printf("end hello!\n");

  while (1) 
  {
    printf("start loop 1\n");
    uint8_t cameraCommand_last = 0;
    uint8_t is_header = 0;
    

    //if(SerialUSBAvailable())
    if(true)
    {
      //usart_Command=SerialUsbRead();
      if(!queue_try_remove(&arducam_cmd_queue, &usart_Command)){
          usart_Command = 0x10;
      }
      switch (usart_Command)
      {
        case 0:
          myCAM.OV2640_set_JPEG_size(OV2640_160x120);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_160x120 END\n");
          usart_Command = 0xff;
          break;
        case 1:
          myCAM.OV2640_set_JPEG_size(OV2640_176x144);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_176x144 END\n");
          usart_Command = 0xff;
          break;
        case 2: 
          myCAM.OV2640_set_JPEG_size(OV2640_320x240);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_320x240 END\n");
          usart_Command = 0xff;
          break;
        case 3:
          myCAM.OV2640_set_JPEG_size(OV2640_352x288);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_352x288 END\n");
          usart_Command = 0xff;
          break;
        case 4:
          myCAM.OV2640_set_JPEG_size(OV2640_640x480);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_640x480 END\n");
          usart_Command = 0xff;
          break;
        case 5:
          myCAM.OV2640_set_JPEG_size(OV2640_800x600);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_800x600 END\n");
          usart_Command = 0xff;
          break;
        case 6:
          myCAM.OV2640_set_JPEG_size(OV2640_1024x768);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_1024x768 END\n");
          usart_Command = 0xff;
          break;
        case 7:
          myCAM.OV2640_set_JPEG_size(OV2640_1280x1024);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_1280x1024 END\n");
          usart_Command = 0xff;
          break;
        case 8:
          myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);sleep_ms(1000);
          printf("ACK CMD switch to OV2640_1600x1200 END\n");
          usart_Command = 0xff;
          break;
        case 0x10:
          mode = 1;
          usart_Command = 0xff;
          start_capture = 1;
          printf("ACK CMD CAM start single shoot. 0x10 END\n");
          break;
        case 0x11: 
          usart_Command = 0xff;
          myCAM.set_format(JPEG);
          myCAM.InitCAM();
          break;
        case 0x20:
          mode = 2;
          usart_Command = 0xff;
          start_capture = 2;
          printf("ACK CMD CAM start video streaming. END\n");
          break;
        case 0x30:
          mode = 3;
          usart_Command = 0xff;
          start_capture = 3;
          printf("ACK CMD CAM start single shoot. END\n");
          break;
        case 0x31:
          usart_Command = 0xff;
          myCAM.set_format(BMP);
          myCAM.InitCAM();
          #if !(defined (OV2640_MINI_2MP))        
            myCAM.clear_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
          #endif
          myCAM.wrSensorReg16_8(0x3818, 0x81);
          myCAM.wrSensorReg16_8(0x3621, 0xA7);
          break;
        case 0x40:
          myCAM.OV2640_set_Light_Mode(Auto);usart_Command = 0xff;
          printf("ACK CMD Set to Auto END\n");break;
        case 0x41:
          myCAM.OV2640_set_Light_Mode(Sunny);usart_Command = 0xff;
          printf("ACK CMD Set to Sunny END\n");break;
        case 0x42:
          myCAM.OV2640_set_Light_Mode(Cloudy);usart_Command = 0xff;
          printf("ACK CMD Set to Cloudy END\n");break;
        case 0x43:
          myCAM.OV2640_set_Light_Mode(Office);usart_Command = 0xff;
          printf("ACK CMD Set to Office END\n");break;
        case 0x44:
          myCAM.OV2640_set_Light_Mode(Home);   usart_Command = 0xff;
          printf("ACK CMD Set to Home END\n");break;
        case 0x50:
          myCAM.OV2640_set_Color_Saturation(Saturation2); usart_Command = 0xff;
          printf("ACK CMD Set to Saturation+2 END\n");break;
        case 0x51:
          myCAM.OV2640_set_Color_Saturation(Saturation1); usart_Command = 0xff;
          printf("ACK CMD Set to Saturation+1 END\n");break;
        case 0x52:
          myCAM.OV2640_set_Color_Saturation(Saturation0); usart_Command = 0xff;
          printf("ACK CMD Set to Saturation+0 END\n");break;
        case 0x53:
          myCAM. OV2640_set_Color_Saturation(Saturation_1); usart_Command = 0xff;
          printf("ACK CMD Set to Saturation-1 END\n");break;
        case 0x54:
          myCAM.OV2640_set_Color_Saturation(Saturation_2); usart_Command = 0xff;
          printf("ACK CMD Set to Saturation-2 END\n");break; 
        case 0x60:
          myCAM.OV2640_set_Brightness(Brightness2); usart_Command = 0xff;
          printf("ACK CMD Set to Brightness+2 END\n");break;
        case 0x61:
          myCAM.OV2640_set_Brightness(Brightness1); usart_Command = 0xff;
          printf("ACK CMD Set to Brightness+1 END\n");break;
        case 0x62:
          myCAM.OV2640_set_Brightness(Brightness0); usart_Command = 0xff;
          printf("ACK CMD Set to Brightness+0 END\n");break;
        case 0x63:
          myCAM. OV2640_set_Brightness(Brightness_1); usart_Command = 0xff;
          printf("ACK CMD Set to Brightness-1 END\n");break;
        case 0x64:
          myCAM.OV2640_set_Brightness(Brightness_2); usart_Command = 0xff;
          printf("ACK CMD Set to Brightness-2 END\n");break; 
        case 0x70:
          myCAM.OV2640_set_Contrast(Contrast2);usart_Command = 0xff;
          printf("ACK CMD Set to Contrast+2 END\n");break; 
        case 0x71:
          myCAM.OV2640_set_Contrast(Contrast1);usart_Command = 0xff;
          printf("ACK CMD Set to Contrast+1 END\n");break;
        case 0x72:
          myCAM.OV2640_set_Contrast(Contrast0);usart_Command = 0xff;
          printf("ACK CMD Set to Contrast+0 END\n");break;
        case 0x73:
          myCAM.OV2640_set_Contrast(Contrast_1);usart_Command = 0xff;
          printf("ACK CMD Set to Contrast-1 END\n");break;
        case 0x74:
          myCAM.OV2640_set_Contrast(Contrast_2);usart_Command = 0xff;
          printf("ACK CMD Set to Contrast-2 END\n");break;
        case 0x80:
          myCAM.OV2640_set_Special_effects(Antique);usart_Command = 0xff;
          printf("ACK CMD Set to Antique END\n");break;
        case 0x81:
          myCAM.OV2640_set_Special_effects(Bluish);usart_Command = 0xff;
          printf("ACK CMD Set to Bluish END\n");break;
        case 0x82:
          myCAM.OV2640_set_Special_effects(Greenish);usart_Command = 0xff;
          printf("ACK CMD Set to Greenish END\n");break;  
        case 0x83:
          myCAM.OV2640_set_Special_effects(Reddish);usart_Command = 0xff;
          printf("ACK CMD Set to Reddish END\n");break;  
        case 0x84:
          myCAM.OV2640_set_Special_effects(BW);usart_Command = 0xff;
          printf("ACK CMD Set to BW END\n");break; 
        case 0x85:
          myCAM.OV2640_set_Special_effects(Negative);usart_Command = 0xff;
          printf("ACK CMD Set to Negative END\n");break; 
        case 0x86:
          myCAM.OV2640_set_Special_effects(BWnegative);usart_Command = 0xff;
          printf("ACK CMD Set to BWnegative END\n");break;   
        case 0x87:
          myCAM.OV2640_set_Special_effects(Normal);usart_Command = 0xff;
          printf("ACK CMD Set to Normal END\n");break;   
      }
    }
    if (mode == 1)
    {
      printf("mode 1\n");
      if (start_capture == 1)
      {
        printf("start_capture 1\n");
        myCAM.flush_fifo();
        myCAM.clear_fifo_flag();
        //Start capture
        myCAM.start_capture();
        start_capture = 0;
        printf("end_capture 0\n");
        sleep_ms(1000);

        if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
          printf("burst to xbee start\n");
          //read_fifo_burst(myCAM);
          read_fifo_burst_xbee(myCAM, xbee);
          //Clear the capture done flag
          myCAM.clear_fifo_flag();
          printf("burst to xbee end\n");
          sleep_ms(5000);
        }
      }
    }
  }
}

uint8_t read_fifo_burst_xbee(ArduCAM myCAM, XBeePico& xbee)
{
    int i , count;
    int length = myCAM.read_fifo_length();
    uint8_t * imageBuf =(uint8_t *) malloc(length*sizeof(uint8_t));
    i = 0 ;

    printf("start read_fifo_burst_xbee\n");

    myCAM.CS_LOW();
    myCAM.set_fifo_burst();//Set fifo burst mode
    spi_read_blocking(SPI_PORT, BURST_FIFO_READ,imageBuf, length);
    myCAM.CS_HIGH();
    bool stop = false;

    send_picture_by_xbee(xbee, imageBuf, length);

    printf("end read_fifo_burst_xbee\n");

    free(imageBuf);
    return 1;
}

