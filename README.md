Hubsan & FlySky Tx Adapter
==========================

TxAdapter_gke is an open source standalone adapter translating CPPM signals where 
available from a transmitter, to Hubsan and FlySky signals.

You should read the extended README file in the source directory and look at my 
RCGroups Blog if you need more information.

http://www.rcgroups.com/forums/showthread.php?t=2067079

Almost all of the adapter code was originally derived from the work of 
PhracturedBlue and others for the universal transmitter "Deviation". 

There have been numerous modifications, and in most cases rewrites, of 
the original code.

https://bitbucket.org/PhracturedBlue/deviation

The adapter requires an a7105 transceiver and an Arduino Pro Mini or 
similar preferably at 3.3V. It is possible to modify the original 
Turnigy 9X removable Tx module using its a7105 transceiver.

Code may be modified using the Arduino development environment and 
uploaded using the associated bootloader(s)depending on the particular 
Atmega processor. Pinouts are defined in config.h and the defaults are 
those adopted by Midelic.

http://www.rcgroups.com/forums/showthread.php?t=1954078&highlight=hubsan+adapt

The TXAdapter was developed for my own use and the multicopters I 
have but may be of interest to others. 

You should track PhracturedBlue's Deviation site for additional 
functionality as it is not my intention to do this myself ;).
