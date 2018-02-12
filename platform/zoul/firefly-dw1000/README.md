Zolertia Firefly Revision A platform with DWM1000 support
============================================

![Zolertia Firefly Revision A breakout board][firefly-reva]

This platform support the DWM1000 transceiver. 
For more information about the radio driver read the [README of the DecaWave DW1000 radio driver](../../../dev/dw1000/README.md) and for more informations about the FIREFLY rev A please read the [README of the Zolertia Firefly Revision A](../firefly-reva/README.md).

Connections between the DWM1000 and the Zolertia FIREFLY.
------

|Function       | Port on the Zolertia Z1 | Port on the DWM1000 |
| ------------- | ----------------------- | ------------------- |
| Interrupt     | 21 - PA5/ADC1/AIN5      | Port2.3 GPIO8       |
| SCLK          | 16 - PD0/SPI1.SCLK      | SPICLK              |
| MOSI          | 12 - PC6/SPI1.MOSI      | SPIMOSI             |
| MISO          | 15 - PD2/SPI1.MISO      | SPIMISO             |
| SS            | 22 - PA4/ADC2/AIN4  	  | SPICS               |
| GND           | 10 or 19 - DGND         | GND                 |
| Power, 3.3v   | 11 or 20 - +VDD         | VDD                 |


[firefly-reva]: ../images/firefly-reva.jpg "Zolertia Firefly Revision A breakout board"
[readme-dw1000]: ../../../dev/dw1000/README.md "README of the DecaWave DW1000 radio driver"
[readme-firefly-reva]: ../firefly-reva/README.md "README of the Zolertia Firefly Revision A"