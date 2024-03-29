Zolertia RE-Mote platform (revision B)
============================================

The RE-Mote is a hardware development platform to build real IoT (Internet of Things) applications and products, aimed to high skilled developers as well as Makers (Do-It-Yourself enthusiasts) and early beginners, providing an industry-ready and resilient hardware solution for most Smart Cities, Home Comfort, eHealth and Industrial applications.  The RE-Mote conciliates an ultra-low power consumption with a high performance design, meeting specifications of processing resources, security and resilient operation.

The RE-Mote platform was designed jointly with universities and industrial partners from different countries in the context of RERUM <https://ict-rerum.eu> European Project.

The RE-Mote features a Zoul as its core module and it is bundled with the following features:

* ARM Cortex-M3 with 512KB flash and 32KB RAM (16KB retention), 32MHz.
* ISM 2.4-GHz IEEE 802.15.4 & Zigbee/Thread compliant.
* ISM 868-, 915-, 920-, 950-MHz ISM/SRD Band.
* On-board RF switch to programatically select RF itnerfaces.  Above RF interfaces can be used alternatively over a single RP-SMA connector for external antenna, or simultaneously by using an UFl pigtail or soldering an internal ceramic chip antenna (available on request).
* AES-128/256, SHA2 Hardware Encryption Engine.
* ECC-128/256, RSA Hardware Acceleration Engine for Secure Key Exchange.
* Power consumption down to 150nA using our shutdown mode.
* Programming over BSL without requiring to press any button to enter bootloader mode.
* Built-in battery charger (500mA), Energy Harvesting and Solar Panels to be connected to standards LiPo batteries.
* Power input with wide range 3.7-16VDC.
* On-board micro USB connector for USB 2.0 applications.
* RGB LED to allow more than 7 colour combinations.
* On-board nano-watt Real Time Clock Calendar (RTCC).
* User and Reset buttons.
* On-board Micro-SD for external storage.
* On-board external Watchdog Timer (WDT) for resilient operation.
* Small form-factor of 73x40 mm.
* Available with enclosure for indoor use

The changes from Revision A to this new Revision B are summarized in the [Wiki](https://github.com/Zolertia/Resources/wiki/RE-Mote:-Enhancements-from-Rev.A-to-Rev.B)

The most prominent feature of the RE-Mote is its ultra low-power implementation, allowing a flexible and time/date-aware control of the platform operation modes by using its real-time clock (RTCC) and an ultra-low power PIC governing the battery manager.

The RE-Mote features an optional custom-made enclosure to fit most scenarios, allowing to easily connect sensors, actuators and rechargeable LiPo batteries.  Its on-board RP-SMA antenna eliminates the need to mechanize an external antenna, allowing to alternatively use either a sub-1GHz or 2.4GHz antenna, or a multiband one.  For dual band applications it is possible to use both sub-GHz and 2.4GHz interfaces simultaneously.

The external WDT with battery monitor allows a robust and resilience operation for most critical applications.

Zoul pin-out
=============

![RE-Mote pin-out (front)][remote-pinout-front]
![RE-Mote pin-out (back)][remote-pinout-back]


RE-Mote and DW1000 Connexions
===========
RE-Mote | DWM1000
----------------
Port 29 - ADC2 | Port 17 - SPICSn
Port 36 - USD.CLK | Port 20 - SPICLK 
Port 35 - USD.MOSI | Port 18 - SPIMOSI
Port 34 - USD.MISO | Port 19 - SPIMISO
Port 26 - GND | Port 8 or 21 or 23 or 24 - VSS
Port 27 - 3.3VDC output pin | Port 5, 6, 7 - VDDAON, VDD3V3, VDD3V3
Port 28 - ADC1 | Port 2 - IRQ / GPIO8