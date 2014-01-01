TXAdapter
=========

This is an open source standalone adapter translating CPPM signals, where available, from a transmitter to Hubsan and FlySky signals. 

Almost all of the adapter code was originally derived from the work of PhracturedBlue and others for the universal transmitter "Deviation". There have been numerous modifications, and in some cases rewrites, of the original code.

https://bitbucket.org/PhracturedBlue/deviation

The adapter requires an a7105 transceiver and an Arduino Pro Mini or similar preferably at 3.3V.  It is possible to modify the original Turnigy 9X removable Tx module using its a7105 transciever.

Code may be modified using the Arduino development environment and uploaded using the associated bootloader(s)depending on the particular Atmega processor. Pinouts are defined in config.h and the defaults are those adopted by Midelic whose source code is not open.

http://www.rcgroups.com/forums/showthread.php?t=1954078&highlight=hubsan+adapt

The TXAdapter was developed for my own use and the multicopters I have but may be of interest to others. You should track PhracturedBlue's Deviation site for additional functionality as it is not my intention to do this myself ;).
