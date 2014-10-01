#include "StorageVolumes.h"

configuration RadioProgramC
{
    provides interface Init;
    provides interface RadioProgram;
}
implementation
{
    components MainC,RadioProgramP,LedsC;
    components new TimerMilliC() as Timer;
    components new BlockStorageC(VOLUME_BSLBLOCK);

    components ActiveMessageC;
    components new AMSenderC(0x90);
    components new AMReceiverC(0x90);

    Init = RadioProgramP;
    RadioProgram = RadioProgramP;
    MainC.SoftwareInit -> RadioProgramP;
    RadioProgramP.Leds -> LedsC;

    RadioProgramP.Timer -> Timer;
    RadioProgramP.RadioControl -> ActiveMessageC;
    RadioProgramP.Packet -> AMSenderC;
    RadioProgramP.AMPacket -> AMSenderC;
    RadioProgramP.Send -> AMSenderC;
    RadioProgramP.Receive -> AMReceiverC;

    RadioProgramP.BlockWrite -> BlockStorageC.BlockWrite;
}
