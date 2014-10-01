#include <msp430.h>
#include "RadioProgram.h"
//#include "Mprintf.h"

module RadioProgramP
{
    provides interface Init;
    provides interface RadioProgram;
    uses interface Leds;
    uses interface Timer<TMilli> as Timer;

    uses interface SplitControl as RadioControl;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend as Send;
    uses interface Receive as Receive;

    uses interface BlockWrite;
}
implementation
{
    message_t pkt;
    bool           StorageBusy,StartProgram;
    uint8_t        BslBuf[BSL_BUFFER_SIZE],BslDataBuf[HEX_MAX_DATA_SIZE];
    uint32_t       BslAddr=0;
    uint16_t       Bsli,BslHexCount=0,BslHexRightSeq,BslHexSeq=0,BslHexAddr=0,BslMaxDataSize=0;
    uint8_t        BslConnected,BslHexLen=0,BslHexType=0,*BslHexData,BslChkSum=0,BslDataBlockLen=0;

    void SendAck(void);
    void ProcessPacket(message_t *msg,void* payload);
    bool TestCheckSum(uint8_t *data,uint8_t len);

    void FlashProgram(void);

    MSP430REG_NORACE(P4SEL);
    MSP430REG_NORACE(P4DIR);
    MSP430REG_NORACE(P4OUT);
    MSP430REG_NORACE(P3SEL);
    MSP430REG_NORACE(P3DIR);
    MSP430REG_NORACE(U0BR0);
    MSP430REG_NORACE(U0BR1);
    MSP430REG_NORACE(U0MCTL);
    MSP430REG_NORACE(U0TCTL);
    MSP430REG_NORACE(U0RCTL);
    MSP430REG_NORACE(U0CTL);
    MSP430REG_NORACE(U0TXBUF);
    MSP430REG_NORACE(U0RXBUF);
    MSP430REG_NORACE(U1TXBUF);
    MSP430REG_NORACE(ME1);

    MSP430REG_NORACE(P1IE       );
    MSP430REG_NORACE(P1IFG      );
    MSP430REG_NORACE(P2IE       );
    MSP430REG_NORACE(P2IFG      );
    MSP430REG_NORACE(SVSCTL     );
    MSP430REG_NORACE(DMACTL0    );
    MSP430REG_NORACE(DMACTL1    );
    MSP430REG_NORACE(DMA0CTL    );
    MSP430REG_NORACE(DMA1CTL    );
    MSP430REG_NORACE(CACTL1     );
    MSP430REG_NORACE(CACTL2     );
    MSP430REG_NORACE(CAPD       );
    MSP430REG_NORACE(TACTL      );
    MSP430REG_NORACE(TAR        );
    MSP430REG_NORACE(TACCTL0    );
    MSP430REG_NORACE(TACCR0     );
    MSP430REG_NORACE(TACCTL1    );
    MSP430REG_NORACE(TACCR1     );
    MSP430REG_NORACE(TACCTL2    );
    MSP430REG_NORACE(TACCR2     );
    MSP430REG_NORACE(TBCTL      );
    MSP430REG_NORACE(TBR        );
    MSP430REG_NORACE(TBCCTL0    );
    MSP430REG_NORACE(TBCCR0     );
    MSP430REG_NORACE(TBCCTL1    );
    MSP430REG_NORACE(TBCCR1     );
    MSP430REG_NORACE(TBCCTL2    );
    MSP430REG_NORACE(TBCCR2     );
    MSP430REG_NORACE(TBCCTL3    );
    MSP430REG_NORACE(TBCCR3     );
    MSP430REG_NORACE(TBCCTL4    );
    MSP430REG_NORACE(TBCCR4     );
    MSP430REG_NORACE(TBCCTL5    );
    MSP430REG_NORACE(TBCCR5     );
    MSP430REG_NORACE(TBCCTL6    );
    MSP430REG_NORACE(TBCCR6     );
    MSP430REG_NORACE(ADC12CTL0  );
    MSP430REG_NORACE(ADC12CTL1  );
    MSP430REG_NORACE(ADC12IFG   );
    MSP430REG_NORACE(ADC12IE    );
    MSP430REG_NORACE(ADC12MCTL0 );
    MSP430REG_NORACE(ADC12MCTL1 );
    MSP430REG_NORACE(ADC12MCTL2 );
    MSP430REG_NORACE(ADC12MCTL3 );
    MSP430REG_NORACE(ADC12MCTL4 );
    MSP430REG_NORACE(ADC12MCTL5 );
    MSP430REG_NORACE(ADC12MCTL6 );
    MSP430REG_NORACE(ADC12MCTL7 );
    MSP430REG_NORACE(ADC12MCTL8 );
    MSP430REG_NORACE(ADC12MCTL9 );
    MSP430REG_NORACE(ADC12MCTL10);
    MSP430REG_NORACE(ADC12MCTL11);
    MSP430REG_NORACE(ADC12MCTL12);
    MSP430REG_NORACE(ADC12MCTL13);
    MSP430REG_NORACE(ADC12MCTL14);
    MSP430REG_NORACE(ADC12MCTL15);
    MSP430REG_NORACE(DAC12_0CTL );
    MSP430REG_NORACE(DAC12_0DAT );
    MSP430REG_NORACE(DAC12_1CTL );
    MSP430REG_NORACE(DAC12_1DAT );
    MSP430REG_NORACE(BCSCTL2    );

    command error_t Init.init()
    {
        BslConnected = 0; 
        StorageBusy = FALSE;
        StartProgram = FALSE;
        BslMaxDataSize = call Packet.maxPayloadLength();
        call RadioControl.start();
        //printfInit();
        return SUCCESS;
    }

    command bool RadioProgram.isConnected()
    {
        if(BslConnected)
            return TRUE;
        return FALSE;
    }

    event void Timer.fired()
    {
        BslHexRightSeq = 0;
        BslHexCount = 0;
        BslAddr = 0;
        BslConnected = 0;
        StorageBusy = FALSE;
        StartProgram = FALSE;
        BslMaxDataSize = call Packet.maxPayloadLength();
        //printf("connection timeout\n");
    }


    void ProcessMessage(void)
    {
        BslMsg*  bslmsg;
        BslAck*  ack;

        bslmsg = (BslMsg*)BslBuf;
        ack = (BslAck*)call Packet.getPayload(&pkt,sizeof(BslAck));
        //printf("start processing message, len=%u, cmd=0x%04X\n",bslmsg->len,bslmsg->cmd);

        call Leds.led0Toggle();

        if( bslmsg->len > BslMaxDataSize) //data lenght too long
        {
            //printf("message size too big\n");
            ack->ack_cmd = BSL_ACK_PACKETSIZE_ERROR;
            ack->ack_data = bslmsg->len;
            SendAck();
            return;
        }

        if( TestCheckSum((uint8_t*)bslmsg,bslmsg->len+4) == FALSE )
        {
            //printf("message chksum error\n");
            ack->ack_cmd = BSL_ACK_CHECKSUM_ERROR;
            ack->ack_data = 1;
            SendAck();
            return;
        }

        if(BslConnected==0 && bslmsg->cmd!=CMD_CONNECT)
        {
            ack->ack_cmd = BSL_ACK_SEQ_ERROR;
            ack->ack_data = 0;
            SendAck();
            return;
        }
        else if(BslConnected==1)
        {
            ack->ack_cmd = BSL_ACK_BUSY;
            ack->ack_data = 0;
            SendAck();
            return;
        }

        switch(bslmsg->cmd)
        {
            case CMD_CONNECT:
                call BlockWrite.erase();
                BslHexRightSeq = 0;
                BslHexCount = 0;
                BslAddr = 0;
                BslConnected = 1;
                StorageBusy = FALSE;
                StartProgram = FALSE;
                BslMaxDataSize = call Packet.maxPayloadLength();
                call Timer.startOneShot(BSL_CONNECT_TIMEOUT);
                //printf("Set up connection\n");
                ack->ack_cmd = BSL_ACK_OK;
                ack->ack_data = BslMaxDataSize;
                SendAck();
                break;
            case CMD_RX_DATA_BLOCK:
                call Timer.startOneShot(BSL_CONNECT_TIMEOUT);

                if(StorageBusy)
                {
                    //printf("Storage busy\n");
                    ack->ack_cmd = BSL_ACK_BUSY;
                    ack->ack_data = BslHexRightSeq;
                    SendAck();
                    break;
                }

                BslDataBlockLen = bslmsg->data[0];
                BslHexSeq =  bslmsg->data[1] + (bslmsg->data[2]<<8);

                if( BslHexSeq != BslHexRightSeq)
                {
                    if((BslHexSeq+BslDataBlockLen)==BslHexRightSeq) //Ack message lost,Sender retransmit
                    {
                        //printf("message ACK lost, resend ACK\n");
                        ack->ack_cmd = BSL_ACK_OK;
                        ack->ack_data = BslHexSeq;
                        SendAck();
                        break;
                    }
                    //printf("message SEQUENCE error, expected:%u, received:%u\n",(BslHexRightSeq+Bsli),BslHexSeq);
                    ack->ack_cmd = BSL_ACK_SEQ_ERROR;
                    ack->ack_data = BslHexRightSeq;
                    SendAck();
                    break;
                }

                //copy to local buffer and write to flash
                StorageBusy = TRUE;
                call BlockWrite.write(BslAddr, &(bslmsg->data[1]), (bslmsg->len-1));
                BslAddr += (bslmsg->len-1);
                ack->ack_cmd = BSL_ACK_OK;
                ack->ack_data = BslHexRightSeq;
                BslHexRightSeq += BslDataBlockLen;
                break;
            case CMD_RUN_PROGRAM:
                if(StorageBusy)
                {
                    call Timer.startOneShot(BSL_CONNECT_TIMEOUT);
                    ack->ack_cmd = BSL_ACK_BUSY;
                    ack->ack_data = 0;
                    SendAck();
                }
                else
                {
                    call Timer.stop();
                    ack->ack_cmd = BSL_ACK_OK;
                    ack->ack_data = 0;
                    SendAck();
                    StartProgram = TRUE;
                }
                break;
            default:
                break;
        }
    }

    bool TestCheckSum(uint8_t *data,uint8_t len)
    {
        uint8_t i,sum;

        sum=0;
        for(i=0;i<len;i++)
        {
            sum += *data;
            data++;
        }

        return (sum==0);
    }

    void SendAck(void)
    {
        uint16_t dst;
        dst = call AMPacket.destination(&pkt);
        call Send.send(dst,&pkt,sizeof(BslAck));
        call Leds.led0Toggle();
        //printf("Ack: cmd=0x%04X data=%u\n\n",cmd,data);
    }


    event void RadioControl.startDone(error_t error)
    {
        if(error !=SUCCESS)
            call RadioControl.start();
    }

    event void RadioControl.stopDone(error_t error)
    {
        if(BslConnected && error==SUCCESS )
            call RadioControl.start();
    }

    event void Send.sendDone(message_t *msg, error_t error)
    {
        if(StartProgram)
        {
            //WDTCTL = 0;  //trigger a PUC reset 
            FlashProgram();
            //((void(*)())0xF600)();
        }
    }

    event message_t* Receive.receive(message_t *msg, void *payload, uint8_t len)
    {
        uint16_t src,dst;  //source and destination
        BslMsg*  bslmsg = (BslMsg*)payload;

        //printf("Packet received, len=%u\n",len); 
        dst = call AMPacket.destination(msg);
        if(dst == TOS_NODE_ID)           //message not for me
        {
            //check if command is valid
            if( bslmsg->cmd == CMD_CONNECT || \
                    bslmsg->cmd == CMD_RX_DATA_BLOCK || \
                    bslmsg->cmd == CMD_RUN_PROGRAM)
            {
                src = call AMPacket.source(msg);
                call AMPacket.setDestination(&pkt,src);
                memcpy(BslBuf,payload,len);
                ProcessMessage();
            }
        }
        return msg;
    }

    event void BlockWrite.writeDone(storage_addr_t x, void* buf, storage_len_t y, error_t result)
    {
        call BlockWrite.sync();
    }

    event void BlockWrite.eraseDone(error_t result)
    {
        BslAck*  ack;
        //printf("Erase done\n");
        call Leds.led0Toggle();
        StorageBusy = FALSE;
        BslConnected = 2;
        ack = (BslAck*)call Packet.getPayload(&pkt,sizeof(BslAck));
        ack->ack_cmd = BSL_ACK_ERASE_DONE;
        ack->ack_data = 0xDEAD;
        SendAck();
    }

    event void BlockWrite.syncDone(error_t result)
    {
        //printf("Flash write done\n");
        StorageBusy = FALSE;
        SendAck();
    }

    uint8_t __attribute__((section(".data"))) __attribute__ ((noinline)) BslSpiSendByte(uint8_t byte)
    {
        while( (IFG1&0x80) == 0);
        U0TXBUF = byte;
        while( (IFG1&0x40) == 0 );
        return U0RXBUF;
    }

    void __attribute__ ((noinline))  BslInit(void)
    {
        P5OUT &= ~(BIT6); //turn on Blue LED to indicate start writing flash
        //clock initilize main clock to run at aproximate 4MHz
        DCOCTL = 0xCC;
        BCSCTL1 = 0x07;
        BCSCTL2 = 0x02;

        //innitiate modules to disable all possible interrupts
        dint();
        WDTCTL = 0x5A80;
        IE1 = 0;    //disable all interrupts
        IE2 = 0;
        IFG1 = 0;   //clear all interrupt flags
        IFG2 = 0;
        //initatiate modules
        P1IE        = 0;      //GPIO
        P1IFG       = 0;
        P2IE        = 0;
        P2IFG       = 0;
        TACTL       = 0;      //TIMERA
        TAR         = 0;     
        TACCTL0     = 0;
        TACCR0      = 0;
        TACCTL1     = 0;
        TACCR1      = 0;
        TACCTL2     = 0;
        TACCR2      = 0;
        TBCTL       = 0;      //TIMERB
        TBR         = 0;     
        TBCCTL0     = 0;
        TBCCR0      = 0;
        TBCCTL1     = 0;
        TBCCR1      = 0;
        TBCCTL2     = 0;
        TBCCR2      = 0;
        TBCCTL3     = 0;
        TBCCR3      = 0;
        TBCCTL4     = 0;
        TBCCR4      = 0;
        TBCCTL5     = 0;
        TBCCR5      = 0;
        TBCCTL6     = 0;
        TBCCR6      = 0;
        DMACTL0     = 0;      //DMA
        DMACTL1     = 0;
        DMA0CTL     = 0;
        DMA1CTL     = 0;
        CACTL1      = 0;      //comparator
        CACTL2      = 0;
        CAPD        = 0;
        ADC12CTL0   = 0;      //ADC12
        ADC12CTL1   = 0;
        ADC12IFG    = 0;
        ADC12IE     = 0;
        ADC12MCTL0  = 0;
        ADC12MCTL1  = 0;
        ADC12MCTL2  = 0;
        ADC12MCTL3  = 0;
        ADC12MCTL4  = 0;
        ADC12MCTL5  = 0;
        ADC12MCTL6  = 0;
        ADC12MCTL7  = 0;
        ADC12MCTL8  = 0;
        ADC12MCTL9  = 0;
        ADC12MCTL10 = 0;
        ADC12MCTL11 = 0;
        ADC12MCTL12 = 0;
        ADC12MCTL13 = 0;
        ADC12MCTL14 = 0;
        ADC12MCTL15 = 0;
        DAC12_0CTL  = 0;  //DAC12
        DAC12_0DAT  = 0;
        DAC12_1CTL  = 0;
        DAC12_1DAT  = 0;
        SVSCTL      = 0;      //Voltage superviser

        //parameters initialize
        BslAddr=0;
        Bsli=0;
        BslHexCount=0;BslHexAddr=0;
        BslHexRightSeq=0;BslHexSeq=0;
        BslHexLen=0;BslHexType=0;
        BslHexData=0;BslChkSum=0;
        //set IO pins, FLASH_CS-P4.4  FLASH_HOLD-P4.7   RADIO_CS=P4.2 
        P4SEL &= ~(BIT7|BIT6|BIT5|BIT4|BIT2);  //set FLASH_CS FLASH_HOLD RADIO_CS ad output
        P4DIR |= (BIT7|BIT6|BIT5|BIT4|BIT2);
        P4OUT &= ~(BIT6|BIT5);     //RADIO_REST=0 RADIO_VREF_EN=0
        P4OUT |= (BIT7|BIT4|BIT2); //FLASH_HOLD=1 FLASH_CS=1 RADIO_CS=1
        //printf("\nP4SEL-%02X P4DIR-%02X P4OUT-%02X\n",P4SEL,P4DIR,P4OUT); 

        P3SEL |= (BIT3|BIT2|BIT1);  //set MOSI MISO SCLK pins
        P3DIR |= (BIT3|BIT1);  //MOSI and SCLK as output
        P3DIR &= (~BIT2);  //MISO as input
        //printf("P3SEL-%02X P3DIR-%02X P3IN-%02X\n",P3SEL,P3DIR,P3IN); 

        //initiate SPI module
        U0BR0  = 2;
        U0BR1  = 0;
        U0MCTL = 0;
        U0TCTL = 0b10100011;
        U0RCTL = 0b00000000;
        U0CTL  = 0b00010111;
        ME1   |= 0b01000000; //enable SPI module
        U0CTL &= (~BIT0);

        //release device from deep power down mode
        P4OUT &= (~BIT4);
        BslSpiSendByte(0xAB);
        P4OUT |= BIT4;
    }

    bool __attribute__((section(".data"))) __attribute__ ((noinline)) Stm25pRead(uint32_t addr,uint8_t* buf,uint16_t len) 
    {
        if(len>256)
            return FALSE;
        //read program from stm23p80
        P4OUT &= (~BIT4);
        BslSpiSendByte(0x03);
        BslSpiSendByte(addr>>16);
        BslSpiSendByte(addr>>8);
        BslSpiSendByte(addr);
        while(len--)
        {
            *buf++ = BslSpiSendByte(0);
        }
        P4OUT |= BIT4;
        return TRUE;
    }

    void __attribute__((section(".data"))) __attribute__ ((noinline)) BslFlashErase(void)
    {
        //set flash memory controller
        while( FCTL3&1 ); //(loop if busy)no need to check since it run from flash
        FCTL2 = 0xA582; // Clock=SMCLK_1Mhz FN=2
        FCTL1 = 0xA506; // ERASE all main memory and infomation memory 
        FCTL3 = 0xA500; // Lock = 0 
        *((uint16_t*)0xF000)=0; // erase all flash segments and infomem 
        while( FCTL3&1 ); //(loop if busy)no need to check since it run from flash
        FCTL1 = 0xA500; // WRT = 0 
        FCTL3 = 0xA510; // Lock = 1
    }

    void __attribute__((section(".data"))) __attribute__ ((noinline))  BslFlashWrite(uint8_t* addr,uint8_t* data,uint16_t len)
    {
        while( FCTL3&0x1 ); //(loop if busy)no need to check since it run from flash
        FCTL1 = 0x0A540; // WRT = 1 
        FCTL3 = 0x0A500; // Lock = 0 
        while(len--)
        {
            *addr++ = *data++; // program Flash word 
            while( FCTL3&0x1 ); //loop if busy
        }
        FCTL1 = 0x0A500; // WRT = 0 
        FCTL3 = 0x0A510; // Lock = 1
    }

    //void __attribute__((section(".bsl_text"))) FlashProgram(void)
    void __attribute__((section(".data"))) __attribute__ ((noinline)) FlashProgram(void)
    //void FlashProgram(void)
    {
        BslInit(); 

        BslFlashErase(); 

        while(BslAddr<0x10000)
        {
            //read stm25p80
            Stm25pRead(BslAddr,BslBuf,BSL_BUFFER_SIZE);
            BslAddr += BSL_BUFFER_SIZE;

            //process and write to flash
            for(Bsli=0;Bsli<BSL_BUFFER_SIZE;Bsli++)
            {
                //printf("%02X",BslBuf[Bsli]);
                if(BslHexCount == 0)
                {
                    BslHexSeq=BslBuf[Bsli];
                    BslChkSum = 0;
                    BslHexCount++;
                    continue;
                }
                if(BslHexCount == 1)
                {
                    BslHexSeq += (BslBuf[Bsli]<<8);
                    BslChkSum = 0;
                    //printf("SEQ:%04u ",BslHexSeq);
                    //printf(":");
                    if(BslHexSeq!=BslHexRightSeq )
                    {
                        //printf("BslError: program SEQUENCE error\n");
                        //P5OUT^=0x10;
                        BSL_Update_Error();
                    }
                    BslHexCount++;
                    continue;
                }
                if(BslHexCount == 2)
                {
                    BslHexLen=BslBuf[Bsli];
                    BslChkSum += BslBuf[Bsli];
                    //printf("LEN:%02u ",BslHexLen);
                    if(BslHexLen>HEX_MAX_DATA_SIZE)
                    {
                        //printf("BslError: program LEN error\n");
                        //P5OUT^=0x10;
                        BSL_Update_Error();
                    }
                    BslHexCount++;
                    continue;
                }
                if(BslHexCount == 3)
                {
                    BslHexAddr = (BslBuf[Bsli]<<8);
                    BslChkSum += BslBuf[Bsli];
                    BslHexCount++;
                    continue;
                }
                if(BslHexCount == 4)
                {
                    BslHexAddr += BslBuf[Bsli];
                    BslChkSum += BslBuf[Bsli];
                    //printf("ADDR:%04X ",BslHexAddr);
                    BslHexCount++;
                    continue;
                }
                if(BslHexCount == 5)
                {
                    BslHexType=BslBuf[Bsli];
                    BslChkSum += BslBuf[Bsli];
                    //printf("TYPE:%02X ",BslHexType);
                    if( (BslHexType != HEX_TYPE_DATA) && (BslHexType != HEX_TYPE_ENDF) && (BslHexType != HEX_TYPE_START_SEG_ADDR) )
                    {
                        //printf("BslError: program TYPE error,Type:%2X\n",BslHexType);
                        //P5OUT^=0x10;
                        BSL_Update_Error();
                    }
                    BslHexCount++;
                    continue;
                }

                BslChkSum += BslBuf[Bsli];

                if(BslHexType==HEX_TYPE_DATA)
                {
                    BslDataBuf[BslHexCount-6]=BslBuf[Bsli];
                }

                if(BslHexType==HEX_TYPE_ENDF)
                {
                    //Dump Flash
                    //for(BslHexAddr=0x4000;BslHexAddr<0xFFFF;BslHexAddr++)
                    //{
                    //    U1TXBUF= *((uint8_t*)BslHexAddr);
                    //    while(!(IFG2&0x20));
                    //}
                    //P5OUT &= 0XDF;while(1);
                    BCSCTL2     = 0;  //CLOCK
                    WDTCTL = 0;  //trigger a PUC reset 
                }

                if(BslHexCount>=(BslHexLen+6))
                {
                    if(BslChkSum != 0)
                    {
                        //printf("BslError: program CHECKSUM error");
                        //P5OUT^=0x10;
                        BSL_Update_Error();
                    }
                    //printf("CHECKSUM:%02X ",BslChkSum);
                    if(BslHexType==HEX_TYPE_DATA)
                        BslFlashWrite((uint8_t*)BslHexAddr,BslDataBuf,BslHexLen);
                    BslHexRightSeq = BslHexSeq+1;
                    BslHexCount = 0;
                    //printf("\n");
                    continue;
                }

                BslHexCount++;
            }
        }

        //printf("BslError: program update error\n");
        BSL_Update_Error();
    }

}

