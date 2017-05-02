Adding the support of the Decawave DW1000 on Contiki OS for the Zolertia Z1.
============================
The Decawave DW1000 is a Ultra Wide Band transceiver.
The development of the driver for Contiki OS is based on a driver developed by the Luleå University of Technology, LTU for the Mulle platform (Kinetis K60). 

The driver is defined in the "dev/dw1000" folder. The platform name is "z1-dw1000".
Actually, the driver do not support Duty Cycle protocol, but this layer is in development for the Ultra Wide Band.

![DW1000 product image](https://maximilien-charlier.be/divers/DW1000-product.jpg "DW1000 product image")

Connection between the DW1000 and the Zolertia Z1.
============================

|Function       | Port on the Zolertia Z1 | Port on the DWM1000 |
| ------------- | ----------------------- | ------------------- |
| Interrupt     | JP1C pin 46             | Port2.3 GPIO8       |
| SCLK          | JPI1B pin 34 SPI.CLK    | SPICLK              |
| MOSI          | JPI1B pin 36 SPI.SIMO   | SPIMOSI             |
| MISO          | JPI1B pin 38 SPI.SOMI   | SPIMISO             |
| SS            | JPI1B pin 32 Port4.0    | SPICS               |
| GND           | JPI1B pin 24, DGND      | GND                 |
| **Power**     | Port on the Zolertia Z1 |                     |
| 5V            | JPI1B pin 23 USB+5V     |                     |

For power supply, we use a "5V to 3.3V For DC-DC Step-Down Power Supply Buck Module AMS1117 LDO 800MA", do not use the 3.3V port on the Zolertia Z1 it do not support the electric demand of the DW1000.


Acknowledgments
============================
We thank H. Derhamy and K. Albertsson for sharing their initial DW1000 driver for the Mulle platform (Kinetis K60). We also acknowledge DecaWave Ltd for their support. Finaly, we thank B. Quoitin for this support throughout the development of this driver.


The Contiki Operating System
============================
Contiki is an open source operating system that runs on tiny low-power
microcontrollers and makes it possible to develop applications that
make efficient use of the hardware while providing standardized
low-power wireless communication for a range of hardware platforms.

Contiki is used in numerous commercial and non-commercial systems,
such as city sound monitoring, street lights, networked electrical
power meters, industrial monitoring, radiation monitoring,
construction site monitoring, alarm systems, remote house monitoring,
and so on.

For more information, see the Contiki website:

[http://contiki-os.org](http://contiki-os.org)
