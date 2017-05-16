Adding the support of the Decawave DW1000 on Contiki OS for the Zolertia Z1.
============================
The Decawave DW1000 is a Ultra Wide Band transceiver.
The development of the driver for Contiki OS is based on a driver developed by the Luleå University of Technology, LTU for the Mulle platform (Kinetis K60). 

The driver is defined in the "dev/dw1000" folder. The platform name is "z1-dw1000".

![DW1000 product image](https://maximilien-charlier.be/divers/DW1000-product.jpg "DW1000 product image")

Feature.
------
* **Full support of the Asymmetrical Double-Sided Two-Way Ranging protocol and the Single-Sided TWR protocol**.

* **Antenna Delay**
This driver gives the possibilities to use a default antenna delay. It gives an accuracy of about two meters.

* **Power Bias Correction**
This feature gives the possibilities to correct the ranging bias related to the received power based on the theoretical receive power levels for a given distance.
It can be used to improve the ranging value after the antenna delay calibration.
For the calibration you should disable this feature. To do this, you can set the macro "DW1000_ENABLE_RANGING_BIAS_CORRECTION" to false (0).

* **RDC**
This driver uses the "NullRDC" protocol. I other world the antenna will be always put in "Receive mode" when we don't send messages. This is expensive in energy. You can reduce the consumption by the usage of the SNIFF mode (by the call of the function, "void dw_set_snif_mode(uint8_t enable, uint8_t rx_on, uint8_t rx_off)" with good value (see the user-manual "0x1D – SNIFF Mode").

* **LED**
The DecaWave DWM1000 have some LED indicated the preamble detection, the PHR detection, the receive good event and finally the TX good event. You can reduce consumption by disabling this feature using the macro "DEBUG_LED".

* **Acknowledgment** available for all configurations.

Ranging compatibility.
------
The ranging feature is not available for all possible configuration.

| Data rate | Preamble  | Double Buffering  | SS-TWR | DS-TWR |
| --------- | --------- | ----------------- | ------ | ------ |
| 110       | 1024-4096 | Yes or No         | **Yes**| **Yes**|
| 850       | 256-512   | Yes or No         | No     | **Yes**|
| 6800      | 256       | Yes or No         | No     | **Yes**|
| 6800      | 128       | Yes               | No     | **Yes**|
| 6800      | 128       | No                | No     | No     |

Connection between the DW1000 and the Zolertia Z1.
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


Acknowledgments
------
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
