# Flexi-Luminaire
Team project for 24-671: "Electromechanical Systems Design" at Carnegie Mellon University


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#components">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

Many tabletops and benchwork applications/projects require the use of both hands as well as an adjustable light source. During these tasks, manual adjustments of the brightness and position of the light source can often disrupt the operative workflow of the user, wasting valuable time. The goal of this project was to provide an intelligent hands-free lighting system that automatically adapts to the task of the user and minimizes workflow interruptions. 


<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Components

* [Intel Celeron Mini PC](https://www.amazon.com/Beelink-Windows-Celeron-Computer-Supports/dp/B09373HTN7/ref=sr_1_4?dchild=1&keywords=windows+10+mini+pc&qid=1635965865&sr=8-4)
* [12V 10A Power Adapter](https://www.amazon.com/BINZET-Adapter-Converter-Regulator-Flexible/dp/B00Z9X4GLW/ref=sr_1_1_sspa?crid=2P0IKUITREDU4&dchild=1&keywords=12v+10amp+power+supply&qid=1635965896&sprefix=12v+10amp+%2Caps%2C179&sr=8-1-spons&psc=1&smid=AYOMIWFVJV81A&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEzVlBBM08xOFc1TVBJJmVuY3J5cHRlZElkPUEwNzA0MzM2MU82VlhBQ0E1VTdURyZlbmNyeXB0ZWRBZElkPUEwMTM1MzYxMTZaWEhQM0MzNVRSUiZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=)
* [Leap Motion Controller](https://www.robotshop.com/en/leap-motion-controller.html?gclid=Cj0KCQjw5oiMBhDtARIsAJi0qk3YCjcAiLCobMI_5lXg90MmsfTuj_A1lpi5YGJtwPuDrVFjbz7b9toaAgxNEALw_wcB)
* [NeoPixel Ring (4x 1/4 Ring)](https://www.adafruit.com/product/1768?gclid=Cj0KCQjw5oiMBhDtARIsAJi0qk3HKKZvtyy_qkwPZq9q8IA2FByjLKafa1tfgS3PZGer1pkM_ZmxuSUaAloKEALw_wcB)
* [TSL2591 Light Sensor](https://www.amazon.com/CQRobot-Ambient-Compatible-Raspberry-TSL25911FN/dp/B083KM51DF)
* [Arduino Uno](https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU/ref=sr_1_1_sspa?dchild=1&keywords=Arduino+Uno&qid=1635966234&s=industrial&sr=1-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyQzFIV0M2MTBUN1o3JmVuY3J5cHRlZElkPUEwNDA3ODU1M0RFSjRPUjhMOVVUQiZlbmNyeXB0ZWRBZElkPUEwNzY1NzM2M0VJQ0VJMTdZNDVHWCZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=)
* [Ultrasonic Sensor](https://www.amazon.com/Tangyy-Ultrasonic-Distance-Transmitter-Raspberry/dp/B08QMW1T34/ref=sr_1_5?dchild=1&keywords=ultrasonic+sensor&qid=1635966262&s=industrial&sr=1-5)
* [sg90 9g MicroServo (2x)](https://www.amazon.com/Micro-Servos-Helicopter-Airplane-Controls/dp/B07MLR1498/ref=sr_1_5?dchild=1&keywords=micro+servo&qid=1635966564&sr=8-5)
* [12V 5V5A Buck Converter](https://www.amazon.com/UCTRONICS-Converter-Transformer-Voltage-Regulator/dp/B07XXWQ49N/ref=sr_1_8?dchild=1&keywords=5v+3.3v+5A+buck+converter&qid=1635966682&sr=8-8)
* [100uF capacitor](https://www.amazon.com/Electrolytic-Capacitor-Assortment-0-1uF-1000uF-Capacitors/dp/B08ZHKLW1M/ref=sr_1_2_sspa?dchild=1&keywords=100uF+capacitor&qid=1635966741&sr=8-2-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEzVlVMNkI1SkgwUElZJmVuY3J5cHRlZElkPUEwMDY0NTc5TEE3MEhVTkRCQjZMJmVuY3J5cHRlZEFkSWQ9QTA2MzEzMDkxSlRNWkhKQTZUT0pKJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==)
* [Nema 17 Stepper Motor (2x)](https://www.amazon.com/Twotrees-Nema17-Stepper-17HS4401S-Printer/dp/B07Y2SVNGP/ref=sr_1_1_sspa?crid=1FWCCYBCRL0NT&dchild=1&keywords=nema+17+stepper+motor&qid=1635966767&sprefix=nema+17+%2Caps%2C207&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExUjFMTzMxWU84OTRJJmVuY3J5cHRlZElkPUEwODM4NjY5MVdRWVdJQ0NJSFNXTiZlbmNyeXB0ZWRBZElkPUEwMDM0MjQ5VEg3TUtPSjdGNlZTJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==)
* [Terminal Block (5pc kit)](https://www.amazon.com/Positions-Terminal-Pre-Insulated-Barrier-MILAPEAK/dp/B07CLY5N9T/ref=sr_1_1_sspa?crid=8QE3FW3UKMTL&dchild=1&keywords=terminal+block&qid=1635967806&sprefix=terminal+%2Caps%2C191&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExMk9YR1k5T1M5Vk41JmVuY3J5cHRlZElkPUEwOTQ5NjY1MU04REVJVTFJVTFVVyZlbmNyeXB0ZWRBZElkPUEwMzc4NDgwVDBGNVlLWkZQVzBaJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==)
* TO-DO

## Installation
<a name="installation"/>

### Hardware
  TO-DO: Include wiring diagram of system

### Software
  + Setup Intel Celeron PC with Windows 10 profile after first boot
  + Install [Leap Motion Controller Orion Beta SDK](https://api.leapmotion.com/orion)
  + Install Arduino IDE Editor
      Libraries needed by arduino software: [TSL2591](https://github.com/adafruit/Adafruit_TSL2591_Library)
  + Install python library dependencies ([ikpy](https://github.com/Phylliade/ikpy) and pyserial) which may not be part of Python2.7:
     ```
     pip install numpy pyserial ikpy==3.0
     ```


---
## Usage
TO-DO


```
TO-DO
```
_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [x] Make Github Repo
- [x] Test Python2.7 API in Windows 10
- [ ] Add Additional Templates w/ Examples
    - [ ] Electrical Diagram
    - [ ] Assembly Instructions

## Contact
Jonathan Shulgach - [@jshulgach](https://twitter.com/jshulgach) - jshulgac@andrew.cmu.edu
Project Link: [https://github.com/Jshulgac/Flexi-Luminaire](https://github.com/Jshulgach/Flexi-Luminaire)

<p align="right">(<a href="#top">back to top</a>)</p>
