configuration ProgramAppC
{
}
implementation
{
    components MainC, ProgramC as AppC, LedsC;

    components ActiveMessageC;
    components new AMSenderC(0x90);
    components new AMReceiverC(0x90);

    AppC -> MainC.Boot;
    AppC.Leds -> LedsC;

    AppC.RadioControl -> ActiveMessageC;
    AppC.Packet -> AMSenderC;
    AppC.AMPacket -> AMSenderC;
    AppC.AMSend -> AMSenderC;
    AppC.Receive -> AMReceiverC;
}
