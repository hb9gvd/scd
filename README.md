# scd
Serial Command Delayer (for NPR - New Packet Radio - attached Computer)

topic:
As the embedded microcontroller in the NPR modem responds
at its default speed of 921600bps only to interactive
communications, a serial command delayer is needed 
for automated commands to be sent to the NPR-modem.

This serial command delayer (scd) belongs to a 
NPR connected Raspi (or similar) sending the automated
commands eg. by cronjob.

compile: 
gcc -Wall -o scd scd.c;chmod u+x scd

usage: 
scd [device] [bitrate] [delay in ms] ['command']

example: 
scd /dev/ttyACM0 921600 20 'radio on\r\n' # turns radio on
scd /dev/ttyACM0 921600 20 'status\r\n' # shows status page
scd /dev/ttyACM0 921600 20 '\03' # send CTRL-C to stop status page

remarks:
scd displays received serial answers after timeout.

'\r\n' -> CR, Linefeed is required to finish command.
'\03' is the character to send for a CTRL-C
