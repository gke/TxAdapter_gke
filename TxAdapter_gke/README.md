
TxAdapter_gke
=============

This is an open source standalone adapter translating CPPM signals where 
available from a transmitter, to Hubsan and FlySky signals.

Almost all of the adapter code was originally derived from the work of 
PhracturedBlue and others for the universal transmitter "Deviation". 

There have been numerous modifications, and in some cases rewrites, of 
the original code.

https://bitbucket.org/PhracturedBlue/deviation

The TXAdapter was developed for my own use and the multicopters I 
have but may be of interest to others. 

You should track PhracturedBlue's Deviation site for additional 
functionality as it is not my intention to do this myself ;).

Battery Monitoring
==================

When used for the Hubsan X4 products flight battery voltages may be monitored 
with the throttle being progressively scaled back when a preset voltage is 
reached forcing a landing before the battery is destroyed. A buzzer may 
be added to give an audible warning.

MultiWii Telemetry
==================

The Adapter produces MultiWii GUI compatible telemetry for attitude and 
battery voltage display. The only restriction is that for 8MHz processors 
the MultiWii GUI baud rate is limited to 56K.

Processor and Pin Assignments
=============================

The Adapter will work on most, if not all, Arduino compatible boards. 
It will also work on DIY Atmel based boards. Programming is by use of 
the Arduino development environment. Uploading programmes uses the processors 
UART, or where available USB, interface. A programmer is not required.

The default pin assignments are the same as those used by Midelic are 
detailed in the config.h file. Other parameters including unique ID, 
battery cutout voltage etc may also be changed in config.h.

http://www.rcgroups.com/forums/showthread.php?t=1954078&highlight=hubsan+adapt

Transceiver Module
==================

The adapter requires an a7105 transceiver and an Arduino Pro Mini or 
similar preferably at 3.3V. It is possible to modify the original 
Turnigy 9X removable Tx module using its a7105 transceiver.

http://www.rcgroups.com/forums/showpost.php?p=27180258&postcount=2

