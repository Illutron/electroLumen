 #ifdef RUNTIME

// *************************************************************************************
//  ILLUTRON CONTROL NETWORK
//  v0.26
//  29 Jun 2010
// *************************************************************************************
//
//     _____               _     _ _______  ______  _____  __   _
//       |   |      |      |     |    |    |_____/ |     | | \  |
//     __|__ |_____ |_____ |_____|    |    |    \_ |_____| |  \_|
//                                                           
//                                   Nikolaj Møbius  / bitknepper
//                                   Sonny Windstrup / syntaks pedant
//                                   WWW.ILLUTRON.DK

#include <avr/interrupt.h>
#include <avr/io.h>

// TODO: Make TX buffer cyclic 
// TODO: Implement 8-bit user messages
// TODO: Implement 21-bit float & int types

// Revision history:
// v0.26 - Universal range check on all message payloads, NOP fix
// v0.25 - Range check on writereturnint 
// v0.24 - Potential troublesource eliminated in TX handler
// v0.23 - Non-blocking signal modes
// v0.22 - Rewritten non-blocking transmission flow
// v0.21 - Stability fixes
// v0.20 - TX buffer memory management, long-reply safe truncation
// v0.19 - Stability fixes, added signed float8 (sfloat8) type
// v0.18 - Arduino native ADC reads, remote stat fix
// v0.17 - response delay + blocking TX buffer with range check + designated responder
// v0.16 - TX buffer open for appending more data
// v0.15 - Spooler re-implemented
// v0.14 - prehistory

// *************************************************************************************
//  Configuration  
// *************************************************************************************

#define icn_lib_version 26         // ICN library version
#define icn_com_speed 115200       // serial speed
#define icn_rxArgBufSize 48        // number of bytes in rx arg buffers
#define icn_txBufSize 80           // number of bytes in tx buffers
#define icn_broadcastAddress 0     // broadcast receiver address
#define icn_spoolMsgBufSize 12     // number of messages in spool buffer
#define icn_spoolArgBufSize 64     // number of bytes in spool argument buffer
#define icn_uSresponseDelay 150    // microseconds delay before responding

#ifdef icn_mega128
#define icn_numPorts 2
#else
#define icn_numPorts 1
#endif

// *************************************************************************************
//  Signal types
// *************************************************************************************

#define SMT_NOP 0                // no operation

#define SMT_SPOOL_START 1        // request: spool begin (int8 queueSize)
#define SMT_SPOOL_STOP 2         // request: cancel spool mode
#define SMT_SPOOL_END 3          // response: spool response train completed.

#define SMT_UMSG_RQ 10           // request: any user messages
#define SMT_UMSG_NONE 11         // response: no messages

#define SMT_PING_RQ 20           // request: pingback
#define SMT_PING 21              // response: pingback
#define SMT_LIBVER_RQ 22         // request: library version
#define SMT_LIBVER 23            // response: library version (int14)
#define SMT_APPVER_RQ 24         // request: application version
#define SMT_APPVER 25            // response: application version (int14)
#define SMT_APPNAME_RQ 26        // request: application anme 
#define SMT_APPNAME 27           // response: application name
#define SMT_UPTIME_RQ 28         // request: node uptime
#define SMT_UPTIME 29            // response: node uptime (h,m,s) - 3 bytes

#define SMT_APP_AUTHOR_RQ 84     // request: application author 
#define SMT_APP_AUTHOR 85        // response: application author 

#define SMT_APP_START_RQ 30      // request: APP START
#define SMT_APP_STOP_RQ 31       // request: APP STOP
#define SMT_APP_RESET_RQ 32      // request: APP RESET
#define SMT_APP_RESPONSE 39      // response: User app talkback
  
#define SMT_DUMP_RQ 40           // request: forced return buffer dump
#define SMT_DUMP_CHANGE_RQ 41    // request: conditional return buffer dump (only if changed)
#define SMT_NO_DUMP 42           // response: No return data available
#define SMT_NO_CHANGE 43         // response: Return data not sent - no change since last

#define SMT_ADC_DUMP_RQ 44       // request: forced return dump ADCs

#define SMT_DUMP_RAW 50          // response: RAW return buffer dump (unmanaged binary content)
#define SMT_DUMP_MIXED 51        // response: Mixed return buffer dump (header-element pairs)
#define SMT_DUMP_INT8 52         // response: INT8 return buffer dump 
#define SMT_DUMP_INT14 53        // response: INT14 return buffer dump 
#define SMT_DUMP_FLOAT8 54       // response: FLOAT8 return buffer dump
#define SMT_DUMP_FLOAT14 55      // response: FLOAT14 return buffer dump
#define SMT_DUMP_SFLOAT8 56      // response: SIGNED FLOAT8 return buffer dump
#define SMT_DUMP_SFLOAT14 57     // response: SIGNED FLOAT14 return buffer dump

#define SMT_REMOTES_RQ 70        // request: get handles description
#define SMT_REMOTES 71           // response: number and type of handles available. (2 bytes)
#define SMT_REMOTE_SET8_RQ 74    // request: set handle (8bit) (index + int8 value = 2 bytes)
#define SMT_REMOTE_SET14_RQ 75   // request: set handle (14bit) (index + int14 value = 3 bytes)
#define SMT_REMOTE_GET_RQ 76     // request: get handle position (1 byte)
#define SMT_REMOTE_8 77          // response: handle position (8 bits) (index + type + int8 = 3 bytes)
#define SMT_REMOTE_14 78         // response: handle position (14 bits) (index + type + int14 = 4 bytes)

#define SMT_SIG_RANGE_RQ 80      // request: get number of signals available
#define SMT_SIG_RANGE 81         // response: number of signals available
#define SMT_SIG_RQ 82            // request: perform signal (1 byte)
#define SMT_ASIG_RQ 83           // request: perform signal (nonblocking) (1 byte)

#define SMT_FAIL 90              // response: FAIL (generic)
#define SMT_TRUNCATED 91         // response: FAIL - MESSAGE TRUNCATED
#define SMT_UNSUP 92             // response: FAIL - UNSUPPORTED
#define SMT_OOR 93               // response: FAIL - OUT OF RANGE
#define SMT_SYNTAX_ERROR 94      // response: FAIL - SYNTAX ERROR
#define SMT_SPOOL_TIMEOUT 95     // response: FAIL - SPOOL RESPONSE TIMEOUT

#define SMT_OK 100               // response: OK (generic)

#define SMT_USER_RQ 128         // user request

// *************************************************************************************
//  Macros
// *************************************************************************************

#define SET(x,y) (x |=(1<<y))					//-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))       		// |
#define CHK(x,y) (x & (1<<y))           		// |
#define TOG(x,y) (x^=(1<<y))            		//-+

// *************************************************************************************
//  Structs
// *************************************************************************************

struct icns_message
{
  volatile unsigned char msgPort; 
  volatile unsigned char msgFrom;
  volatile unsigned char msgTo;
  volatile unsigned char msgSocket;
  volatile unsigned char msgType;
  volatile unsigned char msgLength;
  volatile unsigned char msgComplete;
  volatile unsigned char *msgContent;
};

struct icns_receiveQueue
{
  unsigned char queueID;
  int argBufSize;
  volatile unsigned char rxState;
  volatile unsigned char messagePadPtr; // selects which messagepad is curently being used
  icns_message messagePads[2]; // message buffers for local use when not in external buffered mode
  icns_message* activePad;  // points to active message buffer (whether local or external)
  volatile unsigned char* argBuffer; // contains message payloads
  volatile int argHead; // offset for next message recording start
  volatile int payloadHead; // counts payload bytes received in current message
  void (*msgHandler)(struct icns_message*, struct icns_receiveQueue*);
};

struct icns_transmitQueue
{
  unsigned char queueID;
  volatile unsigned char idle;
  volatile unsigned char buffer[icn_txBufSize];
  volatile unsigned char txReadHead;
  volatile unsigned char txWriteHead;
  void (*completionHandler)(struct icns_transmitQueue*);
};


// *************************************************************************************
//  Enums
// *************************************************************************************

enum icne_paramMode { int8, int14, float8, float14, sfloat8, sfloat14 };
// enum icne_spoolMode { noSpool, spoolRec, spoolPlay };

// *************************************************************************************
//  Static memory allocation
// *************************************************************************************

volatile unsigned char icn_rxArgBuffers[icn_numPorts][icn_rxArgBufSize];
struct icns_receiveQueue icn_rxQueues[icn_numPorts];
struct icns_transmitQueue icn_txQueues[icn_numPorts];
boolean icn_param_outBuf_changed = false; 
unsigned char icn_param_outBuf[icn_my_params_out_num*2];
unsigned char icn_param_inBuf[icn_my_params_in_num*2];
icne_paramMode icn_param_outType = icn_my_params_out_type;
icne_paramMode icn_param_inType = icn_my_params_in_type;

volatile unsigned char icn_spoolSize;
volatile unsigned char icn_spoolHead;
volatile unsigned char icn_spoolStarter;
volatile unsigned char icn_spoolPort;
volatile unsigned char icn_spoolSocket;
volatile unsigned char icn_spoolResponder;

icns_message icn_spoolMsgBuffer[icn_spoolMsgBufSize];
unsigned char icn_spoolArgBuffer[icn_spoolArgBufSize];
unsigned char icn_spoolLastPersonal;

// *************************************************************************************
//  Queue attendant functions
// *************************************************************************************

void icn_resetRXqueue(struct icns_receiveQueue *q)
{
  int i = q->queueID;
  q->argBufSize = icn_rxArgBufSize;          // argument buffer size
  q->argHead = 0;
  q->payloadHead = 0;
  q->rxState = 0;
  q->messagePadPtr = 0;
  q->activePad = &q->messagePads[q->messagePadPtr];
  q->argBuffer = &icn_rxArgBuffers[i][0];
  q->msgHandler = &icn_mrx_autoResponder; 
}

void icn_resetTXqueue(struct icns_transmitQueue *q)
{
  q->idle = true;
  q->txWriteHead = 0;
  q->txReadHead = 0;
  q->completionHandler = &icn_mtx_makeIdle;
}

// *************************************************************************************
//  BUFFER INITIALIZATION
// *************************************************************************************


void icn_setup_buffers()
{
  for (int i=0;i<icn_numPorts;i++)
  {
    icns_receiveQueue *qr = &icn_rxQueues[i]; 
    qr->queueID = i;                            // identify network segments
    icn_resetRXqueue(qr);
  }
  
  for (int i=0;i<icn_numPorts;i++)
  {
    icns_transmitQueue *qt = &icn_txQueues[i]; 
    qt->queueID = i;                            // identify network segments
    icn_resetTXqueue(qt);
  }
  
  icn_spoolSize = 0;
  icn_spoolHead = 0;
}

// *************************************************************************************
//  SPOOLER CONTROLS
// *************************************************************************************

void icn_spoolRecord(unsigned char starter, unsigned char len, unsigned char socket, unsigned char port, unsigned char responder)
{
  if (len < icn_spoolMsgBufSize)
  {
    icn_spoolStarter = starter;
    icn_spoolResponder = responder;
    icn_spoolPort = port;
    icn_spoolSocket = socket;
    icn_spoolSize = len;
    icn_spoolHead = 0;
    struct icns_receiveQueue *qr = &icn_rxQueues[port];
    qr->argHead = 0;
    qr->argBufSize = icn_spoolArgBufSize;
    qr->argBuffer = &icn_spoolArgBuffer[0];
    qr->activePad = &icn_spoolMsgBuffer[0];
    qr->msgHandler = &icn_mrx_spoolRecorder;
  }
}

void icn_spoolPlay()
{
  // the end of the recording process.
  struct icns_receiveQueue *qr = &icn_rxQueues[icn_spoolPort];
  qr->payloadHead = 0;
  qr->rxState = 0;
  qr->messagePadPtr = 0;
  qr->activePad = &qr->messagePads[qr->messagePadPtr];
  qr->argHead = 0;
  qr->argBufSize = icn_rxArgBufSize;
  qr->argBuffer = &icn_rxArgBuffers[icn_spoolPort][0];
  qr->msgHandler = &icn_mrx_spoolNext;
  icn_spoolHead = 0;
  icn_spoolLastPersonal = 0;
  icn_spoolCheck();
}

void icn_spoolStop()
{
  struct icns_receiveQueue *qr = &icn_rxQueues[icn_spoolPort];
  icn_resetRXqueue(qr);
  struct icns_transmitQueue *qt =  &icn_txQueues[icn_spoolPort];
  icn_resetTXqueue(qt);
}

void icn_spoolCheck()
{
  // has the playback spoolHead reached the end yet?
  if (icn_spoolHead < icn_spoolSize)
  {
    // it is still not at the end of the spool playback process.
    // discover whether I'm next in line
    icns_message *m = &icn_spoolMsgBuffer[icn_spoolHead];
    if (m->msgTo != icn_broadcastAddress)
    {
        // message is for someone. make a note of recipient.
        icn_spoolLastPersonal = m->msgTo;
    }
    switch(m->msgTo)
    {
      case icn_broadcastAddress :
      {
        icn_parse_message(m);
      }
      break;
      
      case icn_my_address :
      {
        delayMicroseconds(icn_uSresponseDelay);
        icns_transmitQueue *qt = &icn_txQueues[icn_spoolPort];
        qt->completionHandler = &icn_mtx_spoolNext;
        icn_parse_message(m);
      }
      break;
      
      default:
      {
        // next message in queue not handled by me - wait for other node
      }
      break;
    }
    // endswitch
  }
  else
  {
    // end of spool playback process.
    if 
    (
      (icn_spoolLastPersonal == icn_my_address) || 
      (
        (icn_spoolLastPersonal == 0) &&         // specific responder was unassigned
        (icn_spoolResponder == icn_my_address)  // if I'm designated responder then I'll do the bumper
      )
    )
    {
      // I get to send bumper message, yay!
     icns_transmitQueue *qt = &icn_txQueues[icn_spoolPort];
     qt->completionHandler = &icn_mtx_spoolStop;
     unsigned char t[1];
     icn_sendMessage(icn_spoolPort, icn_spoolStarter, icn_my_address, icn_spoolSocket, SMT_SPOOL_END, 0, &t[0]); // send bumper
    }
    else
    {
      icns_receiveQueue *qr = &icn_rxQueues[icn_spoolPort];
      qr->msgHandler = &icn_mrx_spoolBumper; // wait for bumper
    }
  }
}

// *************************************************************************************
//  HANDLERS FOR RECEIVED MESSAGES
// *************************************************************************************

void icn_mrx_autoResponder(struct icns_message *m, struct icns_receiveQueue *q)
{
  unsigned char mp = 1-(q->messagePadPtr);
  q->messagePadPtr = mp; // swap receiver pad buffers
  q->activePad = &q->messagePads[mp];
  if ((m->msgTo == icn_my_address) || (m->msgTo == icn_broadcastAddress))
  {
    delayMicroseconds(icn_uSresponseDelay);
    icn_parse_message(m);  
  }
}

void icn_mrx_spoolRecorder(struct icns_message *m, struct icns_receiveQueue *q)
{
  if ((m->msgTo == icn_my_address) || (m->msgTo == icn_broadcastAddress))
  {
    // message received is worth listening to. increment argbufptr.
    q->argHead += m->msgLength;
  }
  else
  {
    if (q->activePad->msgLength > 0)
    {
      // note that the recorded message is stripped of its payload
      q->activePad->msgComplete = false;
      q->activePad->msgLength = 0;
    }
  }
  icn_spoolHead++;
  if (icn_spoolHead >= icn_spoolSize)
  {
    // end of recording process
    icn_spoolPlay();
  }
  else
  {
    q->activePad = &icn_spoolMsgBuffer[icn_spoolHead]; // prepare the next message pad
  }
}

void icn_mrx_spoolNext(icns_message *m, icns_receiveQueue *q)
{
  // message received during spool play.
  icn_spoolHead++;
  icn_spoolCheck();
}

void icn_mrx_spoolBumper(icns_message *m, icns_receiveQueue *q)
{
  // the bumper arrived from someone else, move to autoResponder state
  icn_app_spoolStopped(icn_spoolPort, false);
  icn_spoolStop();
}

// *************************************************************************************
//  HANDLERS FOR COMPLETION OF MESSAGE TRANSMISSIONS
// *************************************************************************************

void icn_mtx_dontcare(struct icns_transmitQueue *q)
{
 // We believe in nothing, Lebowski. Nothing! 
 // And tomorrow we come back and we cut off your jonson. 
}

void icn_mtx_makeIdle(struct icns_transmitQueue *q)
{
  q->idle = true;
  q->txReadHead = 0;
  q->txWriteHead = 0;
  // free transmission buffer 
}

void icn_mtx_spoolNext(struct icns_transmitQueue *q)
{
  icn_resetTXqueue(q);
  icn_spoolHead++;
  icn_spoolCheck();
}

void icn_mtx_spoolStop(struct icns_transmitQueue *q)
{
  icn_app_spoolStopped(icn_spoolPort, true);
  icn_spoolStop();
}

// *************************************************************************************
//   ____   _____  ____   ___     _     _      
//  / ___| | ____||  _ \ |_ _|   / \   | |    
//  \___ \ |  _|  | |_) | | |   / _ \  | |    
//   ___) || |___ |  _ <  | |  / ___ \ | |___ 
//  |____/ |_____||_| \_\|___|/_/   \_\|_____|
//
// *************************************************************************************

// *************************************************************************************
//  Initialize serial interface
// *************************************************************************************

void icn_setup_serial()
{
  
  #ifdef icn_mega128

  // WIRING 

    UCSR0A=0x02;
    UCSR0B=0xd8;			//-Set up serial port 0 115200 8N1
    UCSR0C=0x06;			// |
    UBRR0L=16;                          // |
    UBRR0H=0;                           //-+
    
    UCSR1A=0x02;
    UCSR1B=0xd8;			//-Set up serial port 1 115200 8N1
    UCSR1C=0x06;			// |
    UBRR1L=16;                          // |
    UBRR1H=0;                           //-+
    
  #else
  
    // ARDUINO
    
    UCSR0A=0x02;                        //-Set up serial port 115200 8N1
    UCSR0B=0xd8;			// |
    UCSR0C=0x06;			// |
    UBRR0=16;                           //-+
    
  #endif
  sei();  

}

// *************************************************************************************
//  TX 
// *************************************************************************************

void icn_tx_start(unsigned char port)
{
  switch(port)
  {
    case 0:
#ifndef icn_mega128
    digitalWrite(icn_my_talkPin, HIGH);
#endif
    UDR0 = 0xff; 
    break;

#ifdef icn_mega128
    case 1:
    digitalWrite(icn_my_talkPin, HIGH);
    UDR1 = 0xff;
    break;
#endif 

  }
}

SIGNAL(USART_TX_vect)
{
  icns_transmitQueue *qt = &icn_txQueues[0];
  if (qt->txReadHead < qt->txWriteHead)
  {
    UDR0 = qt->buffer[qt->txReadHead++];
  }
  else
  {
     // end of transmission
    digitalWrite(icn_my_talkPin,LOW);
    qt->completionHandler(qt);
  }
}

// *************************************************************************************
//  RX
// *************************************************************************************

#ifdef icn_mega128

// WIRING

SIGNAL(USART0_RX_vect)
{
  // INCOMING CHAR FROM FTDI PORT
  char c = UDR0;
  icn_handleRX(&icn_rxQueues[0],c);
}

SIGNAL(USART1_RX_vect)
{
  // INCOMING CHAR FROM AUX PORT
  char c = UDR1;
  icn_handleRX(&icn_rxQueues[1],c);
}

#else

// ARDUINO

SIGNAL(USART_RX_vect)
{
  char c = UDR0;
  icn_handleRX(&icn_rxQueues[0],c);
}

#endif

// *************************************************************************************
//  Stream Message Interceptor
// *************************************************************************************

void icn_handleRX(struct icns_receiveQueue *q, char c)
{
   unsigned char v = (unsigned char) c;
   icns_message *pad = q->activePad;
   
   if (v == 0xff)
   {
     // sync char received.
     pad->msgComplete = false;
     pad->msgPort = q->queueID;
     q->rxState = 1; 
   }
   else
   {
     switch(q->rxState)
     {
       case 0:
       {
         // waiting for sync char. nothing.
       }
       break;
       
       case 1:
       {
         pad->msgTo = v;
         q->rxState = 2;
       }
       break;
       
       case 2:
       {
         pad->msgSocket = v;
         q->rxState = 3;
       }
       break;
       
       case 3:
       {
         pad->msgFrom = v;
         q->rxState = 4;
       }
       break;
       
       case 4:
       {
         pad->msgLength = v-1;
         q->rxState = 5;
       }
       break;
       
       case 5:
       {
          pad->msgType = v;
          if (v == 1)
          {
            // SPOOL START RECEIVED
            // MAKE SURE MESSAGE RECEIVER MODE IS AUTORESPONDER
            q->msgHandler = &icn_mrx_autoResponder; 
          }
          if (pad->msgLength > 0)
          {
            q->payloadHead = 0; // reset payload reader head position
            // message has argument payload. do we have room enough to store it?
            if (q->argHead + pad->msgLength < q->argBufSize)
            {
              // yeah we got room, record away
              pad->msgContent = &q->argBuffer[q->argHead]; // note argument buffer position
              q->rxState = 6; // now receive payload
            }
            else
            {
              pad->msgContent = &q->argBuffer[0]; 
              q->rxState = 7; // no, we don't. oops.
            }
          }
          else
          {
            // handle the empty message.
            pad->msgComplete = true;
            q->rxState = 0;
            q->msgHandler(q->activePad,q);
          }
       }
       break;
       
       case 6:
       {
         // start recording payload.
         q->argBuffer[q->argHead + q->payloadHead++] = v;
         if (q->payloadHead >= pad->msgLength) 
         {
           // recording complete.
           pad->msgComplete = true;
           q->rxState = 0;
           q->msgHandler(q->activePad,q);
         }
       }
       break;
       
       case 7:
       {
         // I can't record this message so ignore payload.
         q->payloadHead++;
         if (q->payloadHead >= pad->msgLength) 
         {
           pad->msgComplete = false; // 
           pad->msgLength = 0;
           q->rxState = 0;
           q->msgHandler(q->activePad,q);
         }
       }
       break;
       
     }
   }
}

// *************************************************************************************
//  Stream Message Transmitter
// *************************************************************************************

void icn_sendMessage(
unsigned char port,
unsigned char msgTo,
unsigned char msgFrom,
unsigned char socket,
unsigned char type,
unsigned char length,
unsigned char *payload
)
{
  if (port < icn_numPorts)
  {
    struct icns_transmitQueue *t = &icn_txQueues[port];
//    
    if (t->idle != true)
    {
      // already in transmission mode
      t->buffer[t->txWriteHead++] = 0xff;
    }
    int p = t->txWriteHead;
    if (t->txWriteHead + length + 5 < icn_txBufSize)
    {
      t->txWriteHead += (5+length);
      t->buffer[p++] = msgTo;
      t->buffer[p++] = socket;
      t->buffer[p++] = msgFrom;
      t->buffer[p++] = length+1;
      t->buffer[p++] = type;
      for (int i=0;i<length;i++)
      {
        t->buffer[p++] = icn_clamp8(payload[i]);
      }
      if (t->idle == true)
      {
        t->idle = false;
        icn_tx_start(port);
      }
    }  
    else
    {
      // message is too long. now what ?
    }
  }
  else
  {
    // illegal port index
  }
}

void icn_voidReply(icns_message *m, unsigned char msgType)
{
  unsigned char t[1];
  icn_sendMessage(m->msgPort, m->msgFrom, icn_my_address, m->msgSocket, msgType, 0, &t[0]);
}

void icn_byteReply(icns_message *m, unsigned char msgType, unsigned char v)
{
  unsigned char t[1];
  t[0] = v;
  icn_sendMessage(m->msgPort, m->msgFrom, icn_my_address, m->msgSocket, msgType, 1, &t[0]);
}

void icn_wordReply(icns_message *m, unsigned char msgType, int v)
{
  unsigned char t[2];
  writeInt14 (&t[0], v);  
  icn_sendMessage(m->msgPort, m->msgFrom, icn_my_address, m->msgSocket, msgType, 2, &t[0]);
}

void icn_bufferReply(icns_message *m, unsigned char msgType, int length, unsigned char *buf)
{
  icn_sendMessage(m->msgPort, m->msgFrom, icn_my_address, m->msgSocket, msgType, length, buf);
}

// *************************************************************************************
//   ____      _     ____   ____   _____  ____  
//  |  _ \    / \   |  _ \ / ___| | ____||  _ \ 
//  | |_) |  / _ \  | |_) |\___ \ |  _|  | |_) |
//  |  __/  / ___ \ |  _ <  ___) || |___ |  _ < 
//  |_|    /_/   \_\|_| \_\|____/ |_____||_| \_\
//                                            
// *************************************************************************************

boolean icn_parse_message(icns_message *m)
{
  
  boolean forMe = (m->msgTo == icn_my_address);
  boolean broadCast = (m->msgTo == icn_broadcastAddress);
  unsigned char complete = m->msgComplete;
  unsigned char type = m->msgType;
  unsigned char length = m->msgLength;
  
  switch(type)
  {
    
    case SMT_SPOOL_START:
    {
      if ((broadCast) && (length == 2))
      {
        // okay seems like a correctly formatted spool request
        icn_spoolRecord(m->msgFrom, m->msgContent[0], m->msgSocket, m->msgPort, m->msgContent[1]);
      }
    }
    break;
    
    case SMT_NOP:
    {
      // uh... okay
      if (forMe)
      {
        icn_voidReply(m, SMT_OK);
      }
    }
    break;
    
    case SMT_PING_RQ:
    {
      if (forMe)
      {
        icn_voidReply(m, SMT_PING);
      }
    }
    break;

    case SMT_ADC_DUMP_RQ:
    {
      if (forMe)
      {
        unsigned char t[12];
        for(int i=0;i<6;i++)
        {
          int n = analogRead(i) * 16;
          writeInt14(&t[i<<1],n);
        } 
        icn_bufferReply(m, SMT_DUMP_FLOAT14, 12, &t[0]);
      }
    }
    break;
    
    case SMT_LIBVER_RQ:
    {
      if (forMe)
      {
        icn_wordReply(m, SMT_LIBVER, icn_lib_version);
      }
    }
    break;
    
    case SMT_APPVER_RQ:
    {
      if (forMe)
      {
        icn_wordReply(m, SMT_APPVER, icn_my_appVersion);
      }
    }
    break;

    case SMT_APPNAME_RQ:
    {
      if (forMe)
      {
        unsigned char tbuf[32] = icn_my_appName;
        int len = 255;
        int lpos = 0;
        while ((lpos < 32) && (len == 255))
        {
          if (tbuf[lpos++] == 0) len = lpos;
        }
        icn_bufferReply(m, SMT_APPNAME, len, &tbuf[0]);
      }
    }
    break;
    
    case SMT_APP_AUTHOR_RQ:
    {
      if (forMe)
      {
        unsigned char tbuf[32] = icn_my_appAuthor;
        int len = 255;
        int lpos = 0;
        while ((lpos < 32) && (len == 255))
        {
          if (tbuf[lpos++] == 0) len = lpos;
        }
        icn_bufferReply(m, SMT_APP_AUTHOR, len, &tbuf[0]);
      }
    }
    break;

    case SMT_UPTIME_RQ:
    {
      if (forMe)
      {
        long n = millis();
        int s = (int) ((n / 1000) % 60);
        int mins = (int) (n / 60000);
        int h = mins / 60;
        unsigned char t[3];
        t[0] = h % 99;
        t[1] = mins % 60;
        t[2] = s;
        icn_bufferReply(m, SMT_UPTIME, 3, &t[0]);
      }
    }
    break;

    case SMT_SIG_RANGE_RQ:
    {
      if (forMe)
      {
        icn_byteReply(m, SMT_SIG_RANGE, icn_my_signals_num);
      }
    }
    break;
        
    case SMT_SIG_RQ:
    case SMT_ASIG_RQ:
    {
      if ((forMe) || (broadCast))
      {
        if (m->msgLength == 1)
        {
          unsigned char signal = m->msgContent[0];
          if (signal < icn_my_signals_num)
          {
            if (m->msgType == SMT_ASIG_RQ)
            {
              icn_voidReply(m, SMT_OK);
              icn_app_signal(m->msgFrom, m->msgSocket, signal, broadCast);
            }
            else
            {
              int r = icn_app_signal(m->msgFrom, m->msgSocket, signal, broadCast);
              if (forMe)
              {
                icn_byteReply(m, SMT_APP_RESPONSE, r);
              }
            }
          }
          else
          {
            if (forMe)
            {
              icn_voidReply(m, SMT_OOR);
            }
          }
        }
        else
        {
          if (forMe)
          {
            if (m->msgComplete)
            {
              icn_voidReply(m, SMT_SYNTAX_ERROR);
            }
            else
            {
              icn_voidReply(m, SMT_TRUNCATED);
            }
          }
        }
      }
    }
    break;
    
    case SMT_APP_START_RQ:
    {
      if ((forMe) || (broadCast))
      {
        int r = icn_app_start(m->msgFrom, m->msgSocket, broadCast);
        if (forMe)
        {
          icn_byteReply(m, SMT_APP_RESPONSE, r);
        }
      }
    }
    break;

    case SMT_APP_STOP_RQ:
    {
      if ((forMe) || (broadCast))
      {
        int r = icn_app_stop(m->msgFrom, m->msgSocket, broadCast);
        if (forMe)
        {
          icn_byteReply(m, SMT_APP_RESPONSE, r);
        }
      }
    }
    break;
    

    case SMT_APP_RESET_RQ:
    {
      if ((forMe) || (broadCast))
      {
        int r = icn_app_reset(m->msgFrom, m->msgSocket, broadCast);
        if (forMe)
        {
          icn_byteReply(m, SMT_APP_RESPONSE, r);
        }
      }
    }
    break;

    case SMT_DUMP_RQ:
    {
      if (forMe)
      {
        icn_bufferDump(m);
      }
    }
    break;
    
    case SMT_DUMP_CHANGE_RQ:
    {
      if (forMe)
      {
        if (icn_param_outBuf_changed)
        {
          icn_bufferDump(m);
        }
        else
        {
          icn_voidReply(m, SMT_NO_CHANGE);
        }
      }
    }
    break;
    
    case SMT_REMOTES_RQ:
    {
      if (forMe)
      {
        unsigned char t[2];
        t[0] = icn_my_params_in_num;
        t[1] = icn_my_params_in_type;
        icn_bufferReply(m, SMT_REMOTES, 2, &t[0]);
      }
    }
    break;

    case SMT_REMOTE_SET8_RQ:
    {
      if ((forMe) || (broadCast))
      {
        if (length == 2)
        {
          unsigned char idx = m->msgContent[0];
          unsigned int v = m->msgContent[1];
          if (idx < icn_my_params_in_num)
          {
            if ( 
              (icn_my_params_in_type == float8) ||
              (icn_my_params_in_type == float14) ||
              (icn_my_params_in_type == sfloat14)
              )
            {
              // remote 8-bit float writes are expanded to 14-bit range
              unsigned int vs = ((unsigned int) v * 129) >> 1; // scale 0-254 to 0-16383
              writeInt14(&icn_param_inBuf[idx<<1], vs);
            }
            else
            {
              writeInt14(&icn_param_inBuf[idx<<1], (unsigned int) v);
            }
            int r = icn_app_param_changed(m->msgFrom, m->msgSocket, idx, broadCast);
            if (forMe)
            {
              icn_byteReply(m, SMT_APP_RESPONSE, r);
            }
          }
          else
          {
            icn_voidReply(m, SMT_OOR);
          }
        }
        else
        {
          // length mismatch
          if (forMe)
          {
            if (m->msgComplete)
            {
              icn_voidReply(m, SMT_TRUNCATED);
            }
            else
            {
              icn_voidReply(m, SMT_SYNTAX_ERROR);
            }
          }
        }
      }
    }
    break; 
    
    case SMT_REMOTE_SET14_RQ:
    {
      if ((forMe) || (broadCast))
      {
        if (length == 3)
        {
          unsigned char idx = m->msgContent[0];
          int offset = idx << 1;
          if (idx < icn_my_params_in_num)
          {
            icn_param_inBuf[offset++] = m->msgContent[1];
            icn_param_inBuf[offset] = m->msgContent[2];
            int r = icn_app_param_changed(m->msgFrom, m->msgSocket, idx, broadCast);
            if (forMe)
            {
              icn_byteReply(m, SMT_APP_RESPONSE, r);
            }
          }
          else
          {
            icn_voidReply(m, SMT_OOR);
          }
        }
        else
        {
          // length mismatch
          if (forMe)
          {
            if (m->msgComplete)
            {
              icn_voidReply(m, SMT_TRUNCATED);
            }
            else
            {
              icn_voidReply(m, SMT_SYNTAX_ERROR);
            }
          }
        }
      }
    }
    break; 
    
    default:
    {
      if (forMe)
      {
        icn_voidReply(m, SMT_UNSUP);
      }
    }
    break;
    
  }
  return false;
}

// *************************************************************************************
//  Number formatters and feedback code
// *************************************************************************************

void icn_bufferDump(icns_message *m)
{
  switch(icn_param_outType)
  {
    case int8:
    {
      icn_bufferReply(m, SMT_DUMP_INT8, icn_my_params_out_num, &icn_param_outBuf[0]);
    }
    break;
    
    case int14:
    {
      icn_bufferReply(m, SMT_DUMP_INT14, icn_my_params_out_num<<1, &icn_param_outBuf[0]);
    }
    break;
    
    case float8:
    {
      icn_bufferReply(m, SMT_DUMP_FLOAT8, icn_my_params_out_num, &icn_param_outBuf[0]);
    }
    break;

    case sfloat8:
    {
      icn_bufferReply(m, SMT_DUMP_SFLOAT8, icn_my_params_out_num, &icn_param_outBuf[0]);
    }
    break;

    case float14:
    {
      icn_bufferReply(m, SMT_DUMP_FLOAT14, icn_my_params_out_num<<1, &icn_param_outBuf[0]);
    }
    break;

    case sfloat14:
    {
      icn_bufferReply(m, SMT_DUMP_SFLOAT14, icn_my_params_out_num<<1, &icn_param_outBuf[0]);
    }
    break;

    default:
    {
      icn_voidReply(m, SMT_NO_DUMP);
    }
    break;
    
  }
  icn_app_dumped(m->msgFrom, m->msgSocket);
}

// ------------------------------------------------------------
// NUMBER FORMAT HELPER FUNCTIONS
// ------------------------------------------------------------

unsigned char icn_clamp8(int v)
{
  if (v < 0) v = 0;
  if (v > 254) v = 254;
  return (unsigned char) v;
}

unsigned int readInt14(unsigned char *ptr)
{
  unsigned int v = 0;
  v += ptr[0];
  v <<= 7;
  v += ptr[1];
  return v;
}

void writeInt14(volatile unsigned char *ptr, int v)
{
  if (v < 0) v = 0;
  if (v > 16383) v = 16383;
  ptr[0] = (v >> 7) & 0x7f;
  ptr[1] = v & 0x7f;
}

float readFloat8(unsigned char *ptr)
{
  float vf = (float) ptr[0] / 254;
  return vf;
}

void writeFloat8(unsigned char *ptr, float v)
{
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  ptr[0] = (unsigned char) (v * 254);
}

float readSFloat8(unsigned char *ptr)
{
  float vf = ((float) ptr[0] / 127) - 1;
  return vf;
}

void writeSFloat8(unsigned char *ptr, float v)
{
  if (v < -1) v = -1;
  if (v > 1) v = 1;
  ptr[0] = (unsigned char) (v * 127) + 127;
}

float readFloat14(unsigned char *ptr)
{
  int v = readInt14(ptr);
  float vf = ((float) v )/ 16383;
  return vf;
}

float readSFloat14(unsigned char *ptr)
{
  int v = readInt14(ptr);
  float vf = (((float) v )/ 8191.5) - 1;
  return vf;
}

void writeFloat14(unsigned char *ptr, float v)
{
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  int vi = (int) (v * 16383);
  writeInt14(ptr, vi);
}

void writeSFloat14(unsigned char *ptr, float v)
{
  if (v < -1) v = -1;
  if (v > 1) v = 1;
  v += 1;
  int vi = (int) (v * 8191.5);
  writeInt14(ptr, vi);
}


// *************************************************************************************
//   _   _  ____   _____  ____   _         _     _   _  ____  
//  | | | |/ ___| | ____||  _ \ | |       / \   | \ | ||  _ \ 
//  | | | |\___ \ |  _|  | |_) || |      / _ \  |  \| || | | |
//  | |_| | ___) || |___ |  _ < | |___  / ___ \ | |\  || |_| |
//   \___/ |____/ |_____||_| \_\|_____|/_/   \_\|_| \_||____/ 
//                                                          
// *************************************************************************************

boolean writeReturnInt(int index, unsigned int value)
{
  if ((index >= 0) && (index < icn_my_params_out_num))
  {
    switch(icn_param_outType)
    {
      case int8:
      {
        icn_param_outBuf[index] = icn_clamp8(value);
          icn_param_outBuf_changed = true;
        return true;
      }
      break;
      
      case int14:
      {
        writeInt14(&icn_param_outBuf[index<<1],value);
        icn_param_outBuf_changed = true;
        return true;
      }
      break;
      
      default:
      {
        // incorrect type for int return
        return false;
      }
      break;
    }
  }
  else
  {
    // out of bounds index error
    return false;
  }
}

boolean writeReturnFloat(int index, float value)
{
  if ((index >= 0) && (index < icn_my_params_out_num))
  {
    switch(icn_param_outType)
    {
      case sfloat8:
      {
        writeSFloat8(&icn_param_outBuf[index], value);
        icn_param_outBuf_changed = true;
        return true;
      }
      case float8:
      {
        writeFloat8(&icn_param_outBuf[index], value);
        icn_param_outBuf_changed = true;
        return true;
      }
      break;
      case float14:
      {
        writeFloat14(&icn_param_outBuf[index<<1], value);
        icn_param_outBuf_changed = true;
        return true;
      }
      break;
      case sfloat14:
      {
        writeSFloat14(&icn_param_outBuf[index<<1], value);
        icn_param_outBuf_changed = true;
        return true;
      }
      break;
      default:
      {
        // incorrect type for float return
        return false;
      }
      break;
    }
  }
  else
  {
    // out of bounds index error
    return false;
  }
}

// ------------------------------------------------------------
// USERLAND REMOTE PARAMETER INPUT FUNCTIONS
// ------------------------------------------------------------

float readRemoteFloat(unsigned int index)
{
  if (index < icn_my_params_in_num)
  {
    switch(icn_param_inType)
    {
      case float8:
      case float14:
      {
        return readFloat14(&icn_param_inBuf[index << 1]);
      }
      break;
      case sfloat8:
      case sfloat14:
      {
        return readSFloat14(&icn_param_inBuf[index << 1]);
      }
      break;
      default:
      {
        return -1; // invalid buffer type 
      }
      break;
    }
  }
  else
  {
    return -2; // out of bounds index errorr
  }
}

int readRemoteInt(unsigned int index)
{
  if (index < icn_my_params_in_num)
  {
    return readInt14(&icn_param_inBuf[index<<1]);
  }
  else
  {
    return -2; // out of bounds index errorr
  }
}

void netstart()
{
  icn_setup_buffers();
  pinMode(icn_my_talkPin, OUTPUT);
  digitalWrite(icn_my_talkPin, LOW);
  icn_setup_serial();
}


// *************************************************************************************
//                        W  W  W  .  I  L  L  U  T  R  O  N  .  D  K
// *************************************************************************************
#endif
