module ProgramC
{
    uses interface Boot;
    uses interface Leds;
    uses interface Timer<TMilli> as Timer;
}
implementation
{
    event void Boot.booted()
    {
        call Timer.startPeriodic(512);
    }

    event void Timer.fired()
    {
        call Leds.led1Toggle();
    }

}

