Zolertia Z1 module compatible with the DWM1000 platform
============================================

![Zolertia Z1 module connected to a DWM1000][zolertia-z1-dwm1000]

For more information about the radio driver read the README in the `/dev/dw1000` folder.

Connection between the DWM1000 and the Zolertia Z1.
------

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

For power supply, we use a "5V to 3.3V For DC-DC Step-Down Power Supply Buck Module AMS1117 LDO 800MA", do not use the 3.3V port on the Zolertia Z1 it does not support the electric demand of the DW1000.

[zolertia-z1-dwm1000]: images/zolertia-z1-dwm1000.jpg "Zolertia Z1 connected with a DecaWaveDWM1000"