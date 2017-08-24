#include <isotp/send.h>
#include <bitfield/bitfield.h>
#include <string.h>

#define PCI_NIBBLE_INDEX 0
#define PAYLOAD_LENGTH_NIBBLE_INDEX 1
#define PAYLOAD_BYTE_INDEX 1

uint32_t rxId;
uint8_t rxLen;
uint8_t rxBuffer [ 8 ];
void fc_delay ( uint8_t sep_time );

void isotp_complete_send(IsoTpShims* shims, IsoTpMessage* message,
        bool status, IsoTpMessageSentHandler callback) {
    if(callback != NULL) {
        callback(message, status);
    }
}

IsoTpSendHandle isotp_send_single_frame(IsoTpShims* shims, IsoTpMessage* message,
        IsoTpMessageSentHandler callback) {
    IsoTpSendHandle handle = {
        success: false,
        completed: true
    };

    uint8_t can_data[CAN_MESSAGE_BYTE_SIZE] = { shims->padding_value };
//    uint8_t can_data[CAN_MESSAGE_BYTE_SIZE] = { 0x00 };
    if(!set_nibble(PCI_NIBBLE_INDEX, PCI_SINGLE, can_data, sizeof(can_data))) {
        shims->log("Unable to set PCI in CAN data");
        return handle;
    }

    if(!set_nibble(PAYLOAD_LENGTH_NIBBLE_INDEX, message->size, can_data,
                sizeof(can_data))) {
        shims->log("Unable to set payload length in CAN data");
        return handle;
    }

    if(message->size > 0) {
        memcpy(&can_data[1], message->payload, message->size);
    }

  /* 20170821 added by michael modified by michael */
    handle.success = shims->send_can_message(message->tx_arbitration_id, can_data,
            shims->frame_padding ? 8 : 1 + message->size);
    //handle.success = true;
    isotp_complete_send(shims, message, handle.success, callback);
    return handle;
}

IsoTpSendHandle isotp_send_multi_frame(IsoTpShims* shims, IsoTpMessage* message,
        IsoTpMessageSentHandler callback) {
#if 0
    // TODO make sure to copy message into a local buffer
    shims->log("Only single frame messages are supported");
    IsoTpSendHandle handle = {
        success: false,
        completed: true,
        tp_state: ISOTP_SEND,
        tranceiver_bytes: 0
    };
    // TODO need to set sending and receiving arbitration IDs separately if we
    // can't always just add 0x8 (and I think we can't)
    return handle;
#endif

  IsoTpSendHandle handle = {
    success: false,
    completed: false,
    tp_state: ISOTP_SEND,
    tranceiver_bytes: 0
  };

  uint8_t TxBuf[8]={ shims->padding_value };
  uint32_t wait_fc = 0, wait_cf = 0, delta = 0;
  uint8_t fc_status = ISOTP_FC_CTS;
  uint8_t blocksize = 0;
  uint8_t min_sep_time = 0;
  uint8_t fc_wait_frames = 0;
  uint16_t seq_id = 1;
  bool bs = false;
  TxBuf[0] = ( PCI_FIRST_FRAME << 4 | ( ( message->size & 0x0F00 ) >> 8 ) );
  TxBuf[1] = ( message->size & 0x00FF );
  memcpy ( TxBuf + 2, message->payload, 6 );             // Skip 2 Bytes PCI
  handle.tranceiver_bytes += 6;
  handle.tp_state = ISOTP_SEND_FF;
  if ( shims->send_can_message(message->tx_arbitration_id, TxBuf, 8) == true ) {
//shims->recv_can_message ( &rxId, &rxLen, rxBuffer );
//printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
    handle.tp_state = ISOTP_WAIT_FIRST_FC;
    while(handle.tp_state!=ISOTP_IDLE && handle.tp_state!=ISOTP_ERROR && handle.tp_state != ISOTP_FINISHED) {
      bs = false;
      switch ( handle.tp_state ) {
        case ISOTP_IDLE:
           break;
        case ISOTP_WAIT_FIRST_FC:
        case ISOTP_WAIT_FC:
           wait_fc = millis ( );
           if ( shims->recv_can_message ( &rxId, &rxLen, rxBuffer ) == true ) {
//printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
             delta = millis ( ) - wait_fc;
             if ( delta >= TIMEOUT_FC ) {
               handle.tp_state = ISOTP_IDLE;
               printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
             }
             else {
                if ( rxId == message->rx_arbitration_id ) {
                  if ( handle.tp_state == ISOTP_WAIT_FIRST_FC ) {
                    fc_status = rxBuffer[ 0 ] & 0x0f;
                    blocksize = rxBuffer[ 1 ];
                    min_sep_time = rxBuffer[ 2 ];
/* fix wrong separation time values according spec */
                    if ( ( min_sep_time > 0x7F ) && ( ( min_sep_time < 0xF1 ) 
                         || ( min_sep_time > 0xF9 ) ) ) min_sep_time = 0x7F;
                  }
                  else
                     printf ( "receive flow clotrol\n" );
                  switch ( rxBuffer[0] & 0x0F ) {
                    case ISOTP_FC_CTS:
                       handle.tp_state = ISOTP_SEND_CF;
                       break;
                    case ISOTP_FC_WT:
/* what to do ? */
                       fc_wait_frames++;
                       if ( fc_wait_frames >= MAX_FCWAIT_FRAME ) {
                         fc_wait_frames=0;
                         handle.tp_state = ISOTP_IDLE;
                       }
                       break;
                    case ISOTP_FC_OVFLW:
                    default:
                       handle.tp_state = ISOTP_IDLE;
                  }
                }
                else {
                   printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
                }
             }
           }
           else {
              printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
           }
           break;

        case ISOTP_SEND_CF:
           while ( handle.tp_state == ISOTP_SEND_CF ) {
             fc_delay ( min_sep_time );
             memset ( TxBuf, shims->padding_value, 8 );
             TxBuf[0] = ( PCI_CONSECUTIVE_FRAME << 4| ( seq_id & 0x0F ) );
             if ( ( message->size - handle.tranceiver_bytes ) > 7 ) {
               memcpy ( TxBuf + 1, message->payload + handle.tranceiver_bytes, 7 );
               handle.tranceiver_bytes += 7;
             }
             else {
                memcpy ( TxBuf + 1, message->payload + handle.tranceiver_bytes, ( message->size - handle.tranceiver_bytes ) );
                handle.tranceiver_bytes = message->size;
                handle.tp_state = ISOTP_FINISHED;
             }

             if ( shims->send_can_message( message->tx_arbitration_id, TxBuf, 8) == true ) {
               if ( blocksize > 0 && handle.tp_state != ISOTP_FINISHED ) {
                 if ( ! ( seq_id % blocksize ) ) {
                   bs = true;
                   handle.tp_state = ISOTP_WAIT_FC;
                 }
               }
               seq_id++;
               seq_id %= 16;
             }
             else {
                printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
             }
           }
           break;
      }
    }
  }
  else {
     printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
  }

  if ( handle.tp_state == ISOTP_FINISHED )
    handle.success = true;
  handle.completed = true;
  isotp_complete_send(shims, message, handle.success, callback);
  return handle;
}

IsoTpSendHandle isotp_send(IsoTpShims* shims, const uint16_t tx_arbitration_id,
        const uint16_t rx_arbitration_id, const uint8_t payload[], uint16_t size,
        IsoTpMessageSentHandler callback) {
    IsoTpMessage message = {
        tx_arbitration_id: tx_arbitration_id,
        rx_arbitration_id: rx_arbitration_id,
        size: size
    };

  /* 20170821 added by michael
     fill message.multi_frame */
    memcpy(message.payload, payload, size);
    if(size < 8) {
        message.multi_frame = false;
        return isotp_send_single_frame(shims, &message, callback);
    } else {
        message.multi_frame = true;
        return isotp_send_multi_frame(shims, &message, callback);
    }
}

bool isotp_continue_send(IsoTpShims* shims, IsoTpSendHandle* handle,
        const uint16_t arbitration_id, const uint8_t data[],
        const uint8_t size) {
    // TODO this will need to be tested when we add multi-frame support,
    // which is when it'll be necessary to pass in CAN messages to SENDING
    // handles.
    if(handle->receiving_arbitration_id != arbitration_id) {
        if(shims->log != NULL) {
            shims->log("The arb ID 0x%x doesn't match the expected tx continuation ID 0x%x",
                    arbitration_id, handle->receiving_arbitration_id);
        }
        return false;
    }
    return false;
}
