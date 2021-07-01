# scd

As the embedded microcontroller in the [NPR modem](https://hackaday.io/project/164092-npr-new-packet-radio) responds
at its default speed of 921600bps only to interactive
communications, a serial command delayer is needed 
for automated commands to be sent to the NPR-modem.

This serial command delayer (scd) belongs to a 
[NPR](https://hackaday.io/project/164092-npr-new-packet-radio) connected Raspi (or similar) sending the automated
commands eg. by cronjob.

#### compile: 
`gcc -Wall -o scd scd.c;chmod u+x scd`

#### usage: 
`scd [device] [bitrate] [delay in ms] ['command']`

#### example: 
`scd /dev/ttyACM0 921600 20 'radio on\r\n'` # turns radio on  
`scd /dev/ttyACM0 921600 20 'status\r\n'` # shows status page  
`scd /dev/ttyACM0 921600 20 '\03'` # send CTRL-C to stop status page  

#### remarks:
scd displays received serial answers after timeout.

`'\r\n'` CR, Linefeed is required to finish command.  
`'\03'` is the character to send for a CTRL-C  

precompiled version for [AMD64](https://github.com/hb9gvd/scd/blob/main/scd-amd64) and [Raspberry Pi](https://github.com/hb9gvd/scd/blob/main/scd-raspi) are availble too.
