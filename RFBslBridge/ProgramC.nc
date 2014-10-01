#include <msp430.h>

module ProgramC
{
    uses interface Boot;
    uses interface Leds;

    uses interface SplitControl as RadioControl;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend;
    uses interface Receive;
}
implementation
{
    enum
    {
        UARTBUF_SIZE = 128,
        UARTQUEUE_SIZE = 8,
        RADIOQUEUE_SIZE = 8,
    };

    MSP430REG_NORACE(U1CTL  ); 
    MSP430REG_NORACE(U1TCTL );     
    MSP430REG_NORACE(U1RCTL ); 
    MSP430REG_NORACE(U1BR0  ); 
    MSP430REG_NORACE(U1BR1  ); 
    MSP430REG_NORACE(U1MCTL ); 
    MSP430REG_NORACE(P3SEL  ); 
    MSP430REG_NORACE(P3DIR  ); 
    MSP430REG_NORACE(P3SEL  ); 
    MSP430REG_NORACE(P3DIR  ); 
    MSP430REG_NORACE(IFG2   ); 
    MSP430REG_NORACE(IE2    ); 
    MSP430REG_NORACE(ME2    ); 
    MSP430REG_NORACE(U1TXBUF);
    MSP430REG_NORACE(U1RXBUF);

    norace uint8_t  Ubuf[UARTQUEUE_SIZE][UARTBUF_SIZE];
    norace uint8_t  Uin,Uout;
    norace bool     Ufull,Ubusy;
    norace bool     UInalter,UOutalter;
    norace uint8_t  Ucount,Utotal,UAckcount;

    message_t       Rbuf[RADIOQUEUE_SIZE];
    norace uint16_t RInfo[RADIOQUEUE_SIZE][2],Rdst;
    norace uint8_t  Rin,Rout,Ssum;
    norace bool     Rfull;
    
    norace uint8_t Rcount,Rlen,Rsum,RMaxLength;
    uint8_t *pointer;

    task void Radio_sendTask();

    event void Boot.booted()
    {
        //init uart to 115200
        U1CTL  = 0x10;
        U1TCTL = 0x20;
        U1RCTL = 0x00;
        U1BR0  = 0x09;
        U1BR1  = 0x00;
        U1MCTL = 0x08;
        P3SEL  |=(1<<6);
        P3DIR  |=(1<<6);
        P3SEL  |=(1<<7);
        P3DIR  &=0x7F;
        IFG2   &= 0xDF;
        IE2    |=0x30;
        ME2    |=0x30;
        
        //initilize local variables
        Uin=0;
        Uout=0;
        UAckcount=0;
        Ufull     = FALSE;
        UInalter  = FALSE;
        UOutalter = FALSE;
        Utotal = 0;

        Rin=0;
        Rout=0;
        Rfull= FALSE;
        Rdst = AM_BROADCAST_ADDR; 
        Rcount=0;
        Rlen=0;
        RMaxLength = call Packet.maxPayloadLength() + 4;

        call RadioControl.start();
    }

    event void RadioControl.startDone(error_t error)
    {
        if(error != SUCCESS)
        {
            call RadioControl.start();
        }
    }

    event void RadioControl.stopDone(error_t error)
    {
    }

    task void Uart_sendTask()
    {
        if(Uin == Uout && Ufull == FALSE)
        {
            Ubusy = FALSE;
            return;
        }
        Ucount = 0;
        Utotal = Ubuf[Uout][0]+4;
        U1TXBUF = 0x7E;
    }

    TOSH_SIGNAL(UART1TX_VECTOR)
    {
       uint8_t temp;

       if(UAckcount)
       {
           U1TXBUF = 0x7C;
           UAckcount--;
           return;
       }

       if(Utotal)
       {
           if(Ucount < Utotal)
           {
               temp = Ubuf[Uout][Ucount];
               if(UOutalter == FALSE)
               {
                   if(temp == 0x7E || temp == 0x7D || temp == 0x7C)
                   {
                       UOutalter = TRUE;
                       U1TXBUF = 0x7D;
                   }
                   else
                   {
                       U1TXBUF = temp;
                       Ucount++;
                   }
               }
               else
               {
                   UOutalter = FALSE;
                   U1TXBUF = temp & 0xDF;
                   Ucount++;
               }
           }
           else if(Ucount == Utotal)
           {
               Uout = (Uout+1) & (UARTQUEUE_SIZE-1);
               Ufull = FALSE;

               Utotal = 0;
               post Uart_sendTask();
               call Leds.led2Toggle();
           }
       }
    }

    
    void receive(message_t* msg, void* payload, uint8_t len)
    {
        uint8_t* p;
        uint8_t  i;
        uint16_t src;

        if(len>UARTBUF_SIZE-2)
        {
            return;
        }

        if(Ufull == FALSE)
        {
            Ubuf[Uin][0] = len;
            Ssum = 0;
            p = (uint8_t*)payload;

            src = call AMPacket.source(msg);
            Ubuf[Uin][2] = src;
            Ssum += Ubuf[Uin][2];
            Ubuf[Uin][3] = (src>>8);
            Ssum += Ubuf[Uin][3];

            for(i=0;i<len;i++)
            {
                Ubuf[Uin][i+4]=*p;
                Ssum += *p;
                p++;
            }

            Ssum = -Ssum;
            Ubuf[Uin][1] = Ssum;

            atomic
            {
                Uin = (Uin+1) & (UARTQUEUE_SIZE-1);
                if(Uin == Uout)
                    Ufull = TRUE;
                if(Ubusy == FALSE)
                {
                    post Uart_sendTask();
                    Ubusy = TRUE; 
                }
            }
        }
        else
            call Leds.led0Toggle();
    }

    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len)
    {
        receive(msg,payload,len);
        return msg;
    }

    task void Radio_sendTask()
    {
        uint8_t len;
        uint16_t dst;
        atomic
        {
            if(Rin == Rout && Rfull == FALSE)
            {
                return;
            }
        }

        len = RInfo[Rout][0];
        dst = RInfo[Rout][1];
        if(call AMSend.send(dst,&(Rbuf[Rout]),len)!=SUCCESS)
        {
            post Radio_sendTask();
        }
    }

    event void AMSend.sendDone(message_t* msg, error_t error)
    {
        if(error == SUCCESS)
        {
            Rout = (Rout+1) & (RADIOQUEUE_SIZE-1);
            Rfull = FALSE;
            call Leds.led2Toggle();
        }
        post Radio_sendTask();
    }


    TOSH_SIGNAL(UART1RX_VECTOR)
    {
        uint8_t data;
        
        data=U1RXBUF;
        //U1TXBUF = data;
        //return;
        //call Leds.led0Toggle();
        if(Rfull == TRUE)
        {
            call Leds.led0Toggle();
            return;
        }

        if(data == 0x7E)
        {
            Rcount = 1;
            return;
        }
        if(data == 0x7C)
        {
            return;
        }
        if(Rcount > 0 && data == 0x7D)
        {
            UInalter = TRUE;
            return;
        }
        if(UInalter == TRUE)
        {
            data |= 0x20;
            UInalter = FALSE;
        }

        switch(Rcount)
        {
            case 0:
                return;
                break;
            case 1:
                Rlen = data+4;
                if(Rlen > RMaxLength)
                    Rcount = 0;
                else
                    Rcount++;
                break;
            case 2:
                Rsum = data;
                Rcount++;
                break;
            case 3:
                Rdst = data;
                Rsum += data;
                Rcount++;
                break;
            case 4:
                Rdst |= (((uint16_t)data)<<8);
                pointer = (uint8_t*)call Packet.getPayload(&(Rbuf[Rin]),Rlen-4);
                Rsum += data;
                Rcount++;
                break;
            default:
                *pointer++ = data;
                Rsum += data;
                Rcount++;
                if( Rcount > Rlen )
                {
                    Rcount = 0;
                    if(Rsum == 0)
                    {
                        RInfo[Rin][0] = Rlen-4;
                        RInfo[Rin][1] = Rdst;
                        Rin = (Rin+1) & (RADIOQUEUE_SIZE-1);
                        if(Rin == Rout)
                            Rfull = TRUE;
                        post Radio_sendTask();

                        if(Ubusy == FALSE)
                            U1TXBUF = 0x7C;
                        else
                            UAckcount++;
                    }
                }
                break;
        }
    }
}
