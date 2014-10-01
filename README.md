RadioBSL
========

Telosb Wireless programming (TinyOS)

Introduction:
This repo contains modules that implements wireless programming function for telosb mote running TinyOS. It program just one mote at one time and works only for motes in direct communication range of the Bridge (or Baststation). If your want more complex function like whole network programming system like Deluge. You can see it as an replacement of the default USB-plug programming, except that it works through the radio.

Files:
Radio-bsl.py: the python script run on computer to connect the remote mote, send your compiled binary program to the wireless     mote, and run the programm
RFBslBridge: the folder implement a Serial-to-Radio bridge that forword the data send by Radio-bsl.py script to the target mote over the 802.15.4 wireless radio, and at the same time forward the ack from the remote module back to script.
RFBslClient: The folder contains the RadioProgram commponet (including: RadioProgram.h, RadioPragramP.nc and RadioProgramC.nc) and a simple Blink example that uses it. The program do nothing but just blink the LED and wait for a remote programming.

How it works:
Basically the RadioProgram module implement the very basic fuction of receiving HEX binary data from radio and store it in external m25p80 NOR flash. When the HEX is received correctly and fully, it get copied from external storage and write into the msp430's flash. After that, just reboot. 
At the computer side, the python script send the HEX binary file (set ID before that) through the Serial-to-Radio bridge mote connectted to the computer to the remote node, and then ask the remote node to program it flash and reboot.
To include radio progrmaming capability to your program, just copy these files(RadioProgram.h, RadioProgram.nc, RadioPragramP.nc, RadioProgramC.nc and volumes-stm25p.xml) to your program folder, and add one line in your high level configuration file: "components RadioProgramC;"

Usage:
Step1: copy Radio-bsl.py to /usr/bin
       command: chmod +x Radio-bsl.py && sudo cp Radio-bsl.py /usr/bin/
Step2: compile the RFBslBridge and program it to a Telosb mote (to act as Bridge later)
       command: make telosb
                make telosb reinstall,1 bsl,/dev/ttyUSB0
Step3: compile the RFBslClient and program it to a target Telosb mote
       command: make telosb
                make telosb reinstall,5 bsl,/dev/ttyUSB1
Step4: copy RadioProgram* files from RFBslClient folder to YOUR_PROGRAM_FOLDER, include RadioProgramC in your highest level Tinyos configuration file (refer to the RFBslClient example), compile your program and upload it with Radio-bsl.py script
       command: cd RFBslClient
                cp RadioProgram* volumes-stm25p.xml YOUR_PROGRAM_FOLDER
                cd YOUR_PROGRAM_FOLDER  && make telosb
                Radio-bsl.py /dev/ttyUSB0 5

More usage information about the Radio-bsl.py script can be find by just run the script with no input. It can allow you to change the destination ID, program a specific HEX file to Remote mote, and Setup an mote as Bridge.
Email to: Cheng Li (vivid8710@gmail.com) for more help. 

