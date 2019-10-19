# L0Stack - M5Stack Library fork for STM32L0

Welcome to program with L0Stack Core

## 1. Get Started

#### Here is the article to get started

* For Windows*

https://docs.m5stack.com/#/en/quick_start/m5core/m5stack_core_get_started_Arduino_Windows


## 2. Example

https://github.com/m5stack/M5Stack/tree/master/examples

## 3. API Reference

https://github.com/m5stack/M5Stack/blob/master/src/M5Stack.h#L19

https://docs.m5stack.com/#/en/api

## 4. H/W Reference

#### Pinout

*We have several kinds of M5Cores, There is [their difference in schematic](https://github.com/m5stack/M5-Schematic/blob/master/Core/hardware_difference_between_cores.md).*

**LCD & TF Card**

*LCD Resolution: 320x240*

<table>
 <tr><td>ESP32 Chip</td><td>GPIO23</td><td>GPIO19</td><td>GPIO18</td><td>GPIO14</td><td>GPIO27</td><td>GPIO33</td><td>GPIO32</td><td>GPIO4</td></tr>
 <tr><td>ILI9341</td><td>/</td><td>MISO</td><td>CLK</td><td>CS</td><td>DC</td><td>RST</td><td>BL</td><td> </td></tr>
 <tr><td>TF Card</td><td>MOSI</td><td>MISO</td><td>CLK</td><td> </td><td> </td><td> </td><td> </td><td>CS</td></tr>

</table>

**Button & Speaker**

<table>
 <tr><td>ESP32 Chip</td><td>GPIO39</td><td>GPIO38</td><td>GPIO37</td><td>GPIO25</td></tr>
 <tr><td>Button Pin</td><td>BUTTON A</td><td>BUTTON B</td><td>BUTTON C</td></tr>
 <tr><td>Speaker</td><td> </td><td> </td><td> </td><td>Speaker Pin</td></tr>
</table>

**GROVE A**

<table>
 <tr><td>ESP32 Chip</td><td>GPIO22</td><td>GPIO21</td></tr>
 <tr><td>GROVE A</td><td>SCL</td><td>SDA</td></tr>
</table>


## 5. USER CASES

* [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI)  - TFT library for the ESP8266 and ESP32 that supports different driver chips

* [M5ez](https://github.com/ropg/M5ez) - The easy way to program on the M5Stack


#### Note:

* How to upgrade M5Stack Libary

  https://docs.m5stack.com/#/en/related_documents/upgrade_m5stack_lib
