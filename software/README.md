# DoorBirdPico Software

(CC) 2021 by Andreas Frisch <fraxinas@purplegecko.de>

## Prerequisites
Build requirements:
* `cmake`
* `gcc-arm-none-eabi`
* `meta-group-base-devel` (Arch) / `build-essential` (debian)

### Raspberry Pi Pico SDK
* get https://github.com/raspberrypi/pico-sdk
* build following readme
* `export PICO_SDK_PATH="/your-pico-sdk-path"`
 
## Building
```
cd software
cmake . -B build/
cd build
make
```

## Programming
* use https://github.com/raspberrypi/picotool
```
sudo /path-to/picotool/build/picotool load doorbirdpico.uf2
```

## Documentation of Doorbird UART Communication
* UART baud rate is 9600
* startup sequence needs to be answered with registration string within 2 cycles

| Bytes (HEX)                        | ASCII              | Direction    | Command                             |
| :--------------------------------- | :----------------- | :----------- | :---------------------------------- |
| 0x0201                             | [STX][SOH]         | Main -> PICO | Startup sequence                    |
| 0x4D54545F31345F3030315F3030310D0A | MTT_14_001_001\r\n | PICO -> Main | Registration as Multi Tenant Module |
| 0x313031230D0A                     | 101#\r\n           | PICO -> Main | Button 1 pressed                    |
| 0x313032230D0A                     | 102#\r\n           | PICO -> Main | Button 2 pressed                    |
| 0x313...230D0A                     | 1nn#\r\n           | PICO -> Main | Button n pressed                    |
| 0x313134230D0A                     | 114#\r\n           | PICO -> Main | Button 14 pressed                   |
