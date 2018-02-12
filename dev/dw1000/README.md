Decawave DW1000 on Contiki OS
============================
The Decawave DW1000 is a Ultra Wide Band transceiver.
The development of the driver for Contiki OS is based on a driver developed by the Luleå University of Technology, LTU for the Mulle platform (Kinetis K60). The port of this radio driver on Contiki was made by Maximilien Charlier, UMONS, maximilien.charlier@umons.ac.be, github user: [maxcharlier](https://github.com/maxcharlier).

The radio driver is currently compatible with the Zolertia Z1 and Zolertia Zoul platform. The platform name is `z1-dw1000` or `remote-dw1000` or `firefly-dw1000`.

This implementation is presented and evaluated (for the Zolertia Z1 platform) in our paper: [*Support for IEEE 802.15.4 ultra-wideband communications in the Contiki operating system*](http://ieeexplore.ieee.org/document/7797662/), IEEE SCVT 2016

![DecaWave DWM1000][decawave-dwm1000]

Features
------
* **Radio configuration** You can change the default configuration of the radio (in the board.h of your platform).
  - The channel (1, 2, 3, 4, 5 or 7) with the macro `DW1000_CHANNEL` and a value with a type of `dw1000_channel_t`.
  - The bitrate (110 or 850 or 6800 kbps) with the macro `DW1000_DATA_RATE` and a value with a type of `dw1000_data_rate_t`.
  - The preamble length with the macro `DW1000_PREAMBLE` (from 64 to 4096 symbols) and a value with a type of `dw1000_preamble_length_t`.
  - The PRF, 16 or 64 MHz with the macro `DW1000_PRF` and a value with a type of `dw1000_prf_t`.

* **Full support of the Asymmetrical Double-Sided Two-Way Ranging protocol and the Single-Sided TWR protocol**.

* **Antenna Delay**
This driver gives the possibilities to use a default antenna delay. It gives an accuracy of about two meters.

* **Power Bias Correction**
This feature gives the possibilities to correct the ranging bias related to the received power based on the theoretical receive power levels for a given distance.
It can be used to improve the ranging value after the antenna delay calibration.
For the calibration you should disable this feature. To do this, you can set the macro `DW1000_ENABLE_RANGING_BIAS_CORRECTION` to false (0).

* **RDC**
This driver uses the "NullRDC" protocol. I other world the antenna will always be put in "Receive mode" when we don't send messages. This is expensive in energy. You can reduce the consumption by the usage of the SNIFF mode (by the call of the function, `void dw_set_snif_mode(uint8_t enable, uint8_t rx_on, uint8_t rx_off)` with good value (see the [dw1000-user-manual] in section "0x1D – SNIFF Mode").

* **LED**
The DecaWave DWM1000 has some LED indicated the preamble detection, the PHR detection, the receive OK and finally the TX OK event. You can reduce consumption by disabling this feature using the macro `DEBUG_LED`.

* **Acknowledgment** Automatic acknowledgment available for all configurations.
Alterable using `NETSTACK_RADIO.set_value(RADIO_PARAM_RX_MODE, radio_value_t value)` and the value `RADIO_RX_MODE_AUTOACK`.

* **Frame filtering** Automatic frame filtering available for all configurations.
Alterable using `NETSTACK_RADIO.set_value(RADIO_PARAM_RX_MODE, radio_value_t value)` and the value `RADIO_RX_MODE_ADDRESS_FILTER`.

* **Interrupt and pooling mode** 
The radio can generate interrupt or be used in polling mode (for TSCH).
Alterable using `NETSTACK_RADIO.set_value(RADIO_PARAM_RX_MODE, radio_value_t value)` and the value `RADIO_RX_MODE_POLL_MODE`.

* **TSCH** is (only) available for the Zoul platform (we need a 20 MHz SPI connection and a 32-bit processor).

TSCH Configuration
------
According to the [dw1000-aph010], we can use the DW1000 on multiple channel simultaneously. We have chosen to use 6 virtual channels described in the following table. A bitrate of 6800 kbps and a preamble length of 256 symbols is used by default.

The preamble length and the bite rate can be changed but you have to recompute the value of the macro `RADIO_DELAY_BEFORE_TX` used to know the time offset between the trigger of a transmission and the effective transmission of the SFD. To compute this offset you can add the macro `RADIO_DELAY_MEASUREMENT` in the [radio driver](./dw1000-radiodriver.c) with the value 1. You have to disable TSCH and use rime to send a frame. The radio driver will print the value for the macro `RADIO_DELAY_BEFORE_TX`. This macro has to be changed in the board.h file

| Virtual Channel| Physical Channel| PRF (MHz)| Preamble Code|
| ------- | -------- | --- | -------- |
| 0       | 1        | 16  | 1        |
| 1       | 3        | 16  | 5        |
| 2       | 5        | 16  | 3        |
| 3       | 1        | 64  | 12       |
| 4       | 3        | 64  | 9        |
| 5       | 5        | 64  | 9        |

Code structure
------
The radio driver is implemented in multiple file.
* `dev/dw1000/dw1000-driver.[ch]` handle the high level function of the radio driver (implement the radio driver structure, the ranging feature, the configuration)
* `dev/dw1000/dw1000.[ch]` handle the low level function of the radio driver (read a register, change the pan id,)
* `dev/dw1000/dw1000-const.h` contain register values describe in the [dw1000-user-manual].
* `dev/dw1000/dw1000-arch.h` this header file contain functions needed on each platform to support the communication in SPI with the DWM1000.
* `dev/dw1000/dw1000-ranging-bias.[ch]` handle the ranging bias correction. Correction of the error in the ranging from the decrease of received power.
* `dev/dw1000/driver-util.[ch]` give some utility function such as print a frame, the system status or the system state, produce an ACK, give the theoretical transmission time.

Ranging compatibility
------
The ranging feature is not available for all possible configuration.

| Data rate | Preamble  | Double Buffering  | SS-TWR | DS-TWR |
| --------- | --------- | ----------------- | ------ | ------ |
| 110       | 1024-4096 | Yes or No         | **Yes**| **Yes**|
| 850       | 256-512   | Yes or No         | No     | **Yes**|
| 6800      | 256       | Yes or No         | No     | **Yes**|
| 6800      | 128       | Yes               | No     | **Yes**|
| 6800      | 128       | No                | No     | No     |

Acknowledgments
------
We thank H. Derhamy and K. Albertsson for sharing their initial DW1000 driver for the Mulle platform (Kinetis K60). We also acknowledge DecaWave Ltd for their support. Finaly, we thank B. Quoitin for this support throughout the development of this driver.

Additional Documentation
------
For additional informations you can refer to the [DW1000 User Manual][dw1000-user-manual] and the [DWM1000 Datasheet][dwm1000-datasheet]. Other information can be found in the [Application Notes of DecaWave][decawave-application-notes].


[dw1000-aph010]: https://www.decawave.com/sites/default/files/aph010_dw1000_inter_channel_interference.pdf "DW1000 APH010 - Inter Channel Interference"
[dw1000-radiodriver]: ./dw1000-driver.c "DW1000 Radio Driver"
[dw1000-user-manual]: https://www.decawave.com/sites/default/files/dw1000_user_manual_2.12.pdf "DW1000 user manual"
[decawave-application-notes]: https://www.decawave.com/application-notes "DecaWave Application Notes" 
[dwm1000-datasheet]: https://www.decawave.com/sites/default/files/dwm1000-datasheet-v1.6.pdf "DWM1000 Datasheet"
[decawave-dwm1000]: images/decawave-dwm1000.jpg "DecaWave DWM1000"