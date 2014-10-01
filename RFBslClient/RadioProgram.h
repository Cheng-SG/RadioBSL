/*
 * =====================================================================================
 *
 *       Filename:  RadioProgram.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/11/2014 19:09:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef _RADIO_PROGRAM_H
#define _RADIO_PROGRAM_H

//general definations
#define RedLedOn()    P5OUT&=0xEF
#define GreenLedOn()  P5OUT&=0xDF
#define BSL_Update_Error();   {RedLedOn();while(1);}

//definations for BSL protocol
#define BSL_CONNECT_TIMEOUT      (1024*2)  //5 seconds
#define BSL_BUFFER_SIZE               128 

#define CMD_CONNECT                0xA55A
#define CMD_RX_DATA_BLOCK          0xA111
#define CMD_RUN_PROGRAM            0xA222

#define BSL_ACK_OK                     0x5000
#define BSL_ACK_BUSY                   0x5111
#define BSL_ACK_SEQ_ERROR              0x5222
#define BSL_ACK_CHECKSUM_ERROR         0x5333
#define BSL_ACK_PACKETSIZE_ERROR       0x5444
#define BSL_ACK_ERASE_DONE             0x5555

typedef struct
{
    uint8_t   len;
    uint8_t   chksum;
    uint16_t  cmd;
    uint8_t   data[BSL_BUFFER_SIZE];
}BslMsg;

/**************************************************
 *
 *  Received ihex data format
 *
 *  [B B|B|BB|B|B B B B B B....B|B]
 *   seq |  | |          |       |    
 *      len | |          |       |
 *        addr|          |       |
 *           type        |       |
 *                      Data     |
 *                            checksum
 *
 * ***********************************************/

typedef struct
{
    uint16_t  ack_cmd;
    uint16_t  ack_data;
}BslAck;

//definations for hex programing 
#define HEX_MAX_DATA_SIZE          32
#define HEX_TYPE_DATA            0x00
#define HEX_TYPE_ENDF            0x01
#define HEX_TYPE_START_SEG_ADDR  0x03

#endif
