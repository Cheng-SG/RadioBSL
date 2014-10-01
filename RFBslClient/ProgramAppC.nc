configuration ProgramAppC
{
}
implementation
{
    components MainC, ProgramC as AppC, LedsC;
    components RadioProgramC;
    components new TimerMilliC() as Timer;

    AppC -> MainC.Boot;

    AppC.Leds -> LedsC;
    AppC.Timer -> Timer;
}

