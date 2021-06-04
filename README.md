# DoorBirdPico

(CC) 2021 by Andreas Frisch <fraxinas@purplegecko.de>

## OwO what's this?
**`DoorBirdPico` is a custom Raspberry Pi Pico PCB design and code to add functionality to the DoorBird Voip entryphone.**

## Hardware Features
* ports for 3 RGB-lit doorbell buttons
* ports for 2 reed sensors (door/letterbox)
* hardware debouncing
* power / UART communication with Doorbird D2100E
* 2 input ports for the Doorbird relays
* 1 potential-free relay output (for door opener)
* RS485 transceiver

## Software
* using Rasperry Pi Pico C API
* interrupt services routines for buttons
* color-fading PWM

## Project Diary
| Date       | Version | Milestone                                       |
| :--------- | :------ | :---------------------------------------------- |
| 2021-04-06 |         | Pico on Breadboard, pushbutton irq & pwm        | 
| 2021-04-13 |         | Implemented UART communication with Doorbird    |
| 2021-04-16 |         | Started git repository                          |
| 2021-05-12 | v0.1    | Commission of 1st prototype PCB with aisler.net |

## TODOs rev 0.2
- [ ] Use 3V3 pin instead of Vsys
- [ ] Silkscreen Reed Door / Post
- [ ] Resistor values R1-R4
- [ ] Fix mounting hole diameter
- [ ] Fix C1
