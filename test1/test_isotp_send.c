#include <stdio.h>
#include <isotp/isotp.h>
#include <string.h>
#include <stdlib.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>

int s = 0;
int nbytes;
struct sockaddr_can addr;
struct can_frame frame;
struct ifreq ifr;
const char *ifname = "vcan0";

void fc_delay ( uint8_t sep_time )
{
  /*if(sep_time < 0x80)
    delay(sep_time);
  else
    delayMicroseconds((sep_time-0xF0)*100);*/
  if ( sep_time < 0x80 )
    usleep (  sep_time * 1000 );
  else
     usleep ( ( sep_time - 0xF0 ) * 100 );
}

bool receive_can ( uint32_t *prcv_id, uint8_t *prcv_len, uint8_t *data ) {
//printf( "line: %d, file: %s, function: %s  \n", __LINE__, __FILE__, __FUNCTION__ );
  if ( s == 0 ) {
  /* 20170809 added by michael
  create a can socket*/
  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    //return -1;
  }
  else {

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_addr.tp.rx_id = 0xabc;
    addr.can_addr.tp.tx_id = 0xdef;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("Error in socket bind");
      //return -2;
    }
  }
  }

  //memset ( &frame, 0, sizeof ( struct can_frame ) );
  nbytes = read(s, &frame, sizeof(struct can_frame));

  if (nbytes < 0) {
    perror("can raw socket read");
    return false;
  }
  else {
  /* paranoid check ... */
  if (nbytes < sizeof(struct can_frame)) {
    fprintf(stderr, "read: incomplete CAN frame\n");
    return false;
  }
  else {
     *prcv_id = frame.can_id;
     *prcv_len = frame.can_dlc;
     memcpy ( data, frame.data, *prcv_len );
     return true;
  }
  }

/*  *prcv_id = 0x108;
  *prcv_len = 8;
  data[ 0 ] = ISOTP_FC_CTS;
  data[ 1 ] = 0x01;
  data[ 2 ] = 0x7F;
  return true;*/
}

// required, this must send a single CAN message with the given arbitration
// ID (i.e. the CAN message ID) and data. The size will never be more than 8
// bytes.
bool send_can(const uint32_t arbitration_id, const uint8_t* data,
        const uint8_t size) {
  if ( s == 0 ) {
  /* 20170809 added by michael
  create a can socket*/
  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket");
    //return -1;
  }
  else {

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_addr.tp.rx_id = 0xabc;
    addr.can_addr.tp.tx_id = 0xdef;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("Error in socket bind");
      //return -2;
    }
  }
  }

  memset ( &frame, 0x00, sizeof ( struct can_frame ) );
  frame.can_id  = arbitration_id;
  frame.can_dlc = size;
  memcpy ( frame.data, data, size );
  nbytes = write(s, &frame, sizeof(struct can_frame));

  //printf("Wrote %d bytes\n", nbytes);
  for ( int i = 0; i < size; i++ ) {
     printf ( " %x ", frame.data[ i ] );
  }
  printf ( "\n" );
  if ( nbytes > 0 )
    return true;
  else
     return false;
//  return true;
}

// optional, provide to receive debugging log messages
void debug(const char* format, ...) {
}


// not used in the current version
bool set_timer(uint16_t time_ms, void (*callback)) {
  return true;
}

// Optional: This is your callback that will be called when the message is
// completely sent. If it was single frame (the only type supported right
// now), this will be called immediately.
void message_sent(const IsoTpMessage* message, const bool success) {
    // You received the message! Do something with it.
  if ( success == true) {
    if ( message->multi_frame == true )
      printf ( "multi-frame message sent success\n" );
    else
       printf ( "single frame message sent success\n" );
  }
  else
     printf ( "message sent fail\n" );
}

int main ( void )
{
  uint8_t txBuffer [ 100 ];
  IsoTpShims shims = isotp_init_shims( debug, send_can, receive_can, set_timer );

  shims.frame_padding = true;
  shims.padding_value = 0x0a;
  //strcpy ( ( char * ) txBuffer, "01234567890123456789012345678901234567890123" );
  //strcpy ( ( char * ) txBuffer, "0123456" );
  IsoTpSendHandle handle;// = isotp_send( &shims, 0x321, 0x123, txBuffer, strlen( txBuffer ), message_sent );

  strcpy ( ( char * ) txBuffer, "0123456789012345" );
  handle = isotp_send( &shims, 0x321, 0x123, txBuffer, strlen( txBuffer ), message_sent );
#if 0
  uint32_t rxId = 0;
  uint8_t rxLen = 0;
  strcpy ( ( char * ) txBuffer, "0123456789012345" );
  txBuffer [0] = 0x10;
  txBuffer [1] = 0x09;
  send_can(0x321, txBuffer, 8);

  receive_can ( &rxId, &rxLen, txBuffer );
  printf ( "id: %d, len: %d\n", rxId, rxLen );
  for ( int i = 0; i < rxLen; i++ ) {
     printf ( " %x", txBuffer[i] );
  }
#endif
#if 0
  if(handle.completed) {
    if(!handle.success) {
        // something happened and it already failed - possibly we aren't able to
        // send CAN messages
        return -1;
    } else {
        // If the message fit in a single frame, it's already been sent
        // and you're done
         printf ( " single frame complete\n " );
    }
    } else {
         while(true) {
        // Continue to read from CAN, passing off each message to the handle
        // this will return true when the message is completely sent (which
        // may take more than one call if it was multi frame and we're waiting
        // on flow control responses from the receiver)
//           bool complete = isotp_continue_send(&shims, &handle, 0x100, data, size);

           //if(complete && handle.completed) {
             if(handle.success) {
                // All frames of the message have now been sent, following
                // whatever flow control feedback it got from the receiver
             } else {
                // the message was unable to be sent and we bailed - fatal
                // error!
             }
           //}
         }
      }
#endif

  printf ( "test isotp send\n" );

 if ( s > 0 ) {
    close ( s );
    printf ("close socket can\n");
  }
  return 0;
}
