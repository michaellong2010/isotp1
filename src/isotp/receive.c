#include <isotp/receive.h>
#include <isotp/send.h>
#include <bitfield/bitfield.h>
#include <string.h>
#include <stdlib.h>

#define ARBITRATION_ID_OFFSET 0x8

static void isotp_complete_receive(IsoTpReceiveHandle* handle, IsoTpMessage* message) {
    if(handle->message_received_callback != NULL) {
        handle->message_received_callback(message);
    }
}

bool isotp_handle_single_frame(IsoTpReceiveHandle* handle, IsoTpMessage* message) {
    isotp_complete_receive(handle, message);
    return true;
}

bool isotp_handle_multi_frame(IsoTpReceiveHandle* handle, IsoTpMessage* message) {
    // call this once all consecutive frames have been received
    isotp_complete_receive(handle, message);
    return true;
}

bool isotp_send_flow_control_frame(IsoTpShims* shims, IsoTpMessage* message) {
    uint8_t can_data[CAN_MESSAGE_BYTE_SIZE] = {0};

    if ( shims->frame_padding == true )
      memset ( can_data, shims->padding_value, CAN_MESSAGE_BYTE_SIZE );
    if(!set_nibble(PCI_NIBBLE_INDEX, PCI_FLOW_CONTROL_FRAME, can_data, sizeof(can_data))) {
        shims->log("Unable to set PCI in CAN data");
        return false;
    }
  can_data[ 0 ] = can_data [ 0 ] & 0xf0;
  can_data[ 0 ]  = can_data [ 0 ] | ( message->fc_status & 0x0f );
  can_data[ 1 ] = message->blocksize;
  can_data[ 2 ] = message->min_sep_time;

    shims->send_can_message(message->tx_arbitration_id, can_data,
            shims->frame_padding ? 8 : 1 + message->size);
    return true;
}


IsoTpReceiveHandle isotp_receive(IsoTpShims* shims, const uint32_t tx_arbitration_id,
        const uint32_t rx_arbitration_id, IsoTpMessageReceivedHandler callback) {
    IsoTpReceiveHandle handle = {
        success: false,
        completed: false,
        tx_arbitration_id: tx_arbitration_id,
        rx_arbitration_id: rx_arbitration_id,
        message_received_callback: callback,
        fc_status: ISOTP_FC_CTS,
        blocksize: 1,
        min_sep_time: 0,
        tp_state: ISOTP_WAIT_DATA
    };

  uint32_t rxId;
  uint8_t  rxLen;
  uint8_t  rxBuffer[8];
  IsoTpMessage msg;
  memset ( &msg, 0, sizeof ( IsoTpMessage ) );
  handle.timeout_ms = millis() + TIMEOUT_SESSION;
  while ( millis () < handle.timeout_ms && handle.tp_state != ISOTP_FINISHED ) {
    if ( shims->recv_can_message ( &rxId, &rxLen, rxBuffer ) == true ) {
/* isotp_continue_receive(IsoTpShims* shims,
        IsoTpReceiveHandle* handle, const uint32_t arbitration_id,
        const uint8_t data[], const uint8_t size) */
      msg = isotp_continue_receive ( shims, &handle, rxId, rxBuffer, rxLen, &msg );
//printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
    }
    else {
       break;
       printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
    }
  }

    return handle;
}

IsoTpMessage isotp_continue_receive(IsoTpShims* shims,
        IsoTpReceiveHandle* handle, const uint32_t arbitration_id,
        const uint8_t data[], const uint8_t size, IsoTpMessage *pmsg ) {
    IsoTpMessage message = {
        tx_arbitration_id: handle->tx_arbitration_id,
        rx_arbitration_id: handle->rx_arbitration_id,
        completed: false,
        multi_frame: false,
        payload: {0},
        size: 0,
        fc_status: handle->fc_status,
        blocksize: handle->blocksize,
        min_sep_time: handle->min_sep_time,
        seq_id: 1
    };
//printf ( "id: %d, %d\n", pmsg->seq_id, pmsg->size );
  if ( pmsg->size != 0 )
    memcpy ( &message, pmsg, sizeof ( IsoTpMessage ) );

    if(size < 1) {
        return message;
    }

    if(handle->rx_arbitration_id != arbitration_id) {
        if(shims->log != NULL)  {
            // You may turn this on for debugging, but in normal operation it's
            // very noisy if you are passing all received CAN messages to this
            // handler.
            /* shims->log("The arb ID 0x%x doesn't match the expected rx ID 0x%x", */
                    /* arbitration_id, handle->arbitration_id); */
        }
        return message;
    }

    IsoTpProtocolControlInformation pci = (IsoTpProtocolControlInformation)
            get_nibble(data, size, 0);

    // TODO this is set up to handle rx a response with a payload, but not to
    // handle flow control responses for multi frame messages that we're in the
    // process of sending

    switch(pci) {
        case PCI_SINGLE: {
           if ( handle->tp_state == ISOTP_WAIT_DATA ) {
            uint8_t payload_length = get_nibble(data, size, 1);
            
            if(payload_length > 0) {
                memcpy(message.payload, &data[1], payload_length);
            }
            
            message.size = payload_length;
            message.completed = true;
            handle->success = true;
            handle->completed = true;
            isotp_handle_single_frame(handle, &message);
            handle->tp_state = ISOTP_FINISHED;
           }
           else
              handle->tp_state = ISOTP_ERROR;
           break;
        }
        //If multi-frame, then the payload length is contained in the 12
        //bits following the first nibble of Byte 0. 
        case PCI_FIRST_FRAME: {
           if ( handle->tp_state == ISOTP_WAIT_DATA ) {
            uint16_t payload_length = (get_nibble(data, size, 1) << 8) + get_byte(data, size, 1);

            if(payload_length > OUR_MAX_ISO_TP_MESSAGE_SIZE) {
                shims->log("Multi-frame response too large for receive buffer.");
                break;
            }

            //Need to allocate memory for the combination of multi-frame
            //messages. That way we don't have to allocate 4k of memory 
            //for each multi-frame response.
            uint8_t* combined_payload = NULL;
            combined_payload = (uint8_t*)malloc(sizeof(uint8_t)*payload_length);

            if(combined_payload == NULL) {
                shims->log("Unable to allocate memory for multi-frame response.");
                break;
            }

            memcpy(combined_payload, &data[2], CAN_MESSAGE_BYTE_SIZE - 2);
            handle->receive_buffer = combined_payload;
            message.size = handle->received_buffer_size = CAN_MESSAGE_BYTE_SIZE - 2;
            handle->incoming_message_size = payload_length;

            message.multi_frame = true;
            handle->success = false;
            handle->completed = false;
            isotp_send_flow_control_frame(shims, &message);
            handle->tp_state = ISOTP_SEND_CF;
            handle->wait_cf = millis();
           }
           else
              handle->tp_state = ISOTP_ERROR;
           break;
        }
        case PCI_CONSECUTIVE_FRAME: {
           if ( handle->tp_state == ISOTP_WAIT_DATA || handle->tp_state == ISOTP_SEND_CF ) {
             uint32_t delta = millis() - handle->wait_cf;
             if ( ( delta >= TIMEOUT_FC ) && message.seq_id > 1) {
               handle->tp_state = ISOTP_ERROR;
               printf ( "time_out\n" );
               return message;
             }
            uint8_t start_index = handle->received_buffer_size;
            uint8_t remaining_bytes = handle->incoming_message_size - start_index;
            message.multi_frame = true;
            
            if ( ( data[0] & 0x0F) != ( message.seq_id & 0x0F ) ) {
              handle->tp_state = ISOTP_ERROR;
              printf ("id error: %d %d\n", message.seq_id, handle->incoming_message_size );
              return message;
            }

//printf ( " remaining_bytes: %d \n ", remaining_bytes );
            if(remaining_bytes > 7) {
                memcpy(&handle->receive_buffer[start_index], &data[1], CAN_MESSAGE_BYTE_SIZE - 1);
                message.size = handle->received_buffer_size = start_index + 7;
            } else {
//printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
                memcpy(&handle->receive_buffer[start_index], &data[1], remaining_bytes);
                handle->received_buffer_size = start_index + remaining_bytes;

                if(handle->received_buffer_size != handle->incoming_message_size){
                    free(handle->receive_buffer);
                    handle->success = false;
                    shims->log("Error capturing all bytes of multi-frame. Freeing memory.");
                } else {
                    memcpy(message.payload,&handle->receive_buffer[0],handle->incoming_message_size);
                    free(handle->receive_buffer);
                    message.size = handle->incoming_message_size;
                    message.completed = true;
                    shims->log("Successfully captured all of multi-frame. Freeing memory.");

                    handle->success = true;
                    handle->completed = true;
                    isotp_handle_multi_frame(handle, &message);
                    handle->tp_state = ISOTP_FINISHED;
                }
            }
            if ( message.blocksize > 0 ) {
              if ( !( message.seq_id % message.blocksize ) && handle->tp_state != ISOTP_FINISHED ) {
//printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
                isotp_send_flow_control_frame( shims, &message );
              }
            }
            message.seq_id++;
            message.seq_id %= 16;
            handle->wait_cf = millis();
           }
           else {
              handle->tp_state =  ISOTP_ERROR;
              if ( handle->receive_buffer != NULL )
                free ( handle->receive_buffer );
           }
           break;
        }
        default:
            break;
    }
    return message;
}
