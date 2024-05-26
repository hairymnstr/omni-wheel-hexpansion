# Motor Hexpansion / Omni Wheel Hexpansion

![An omni wheel attached to a group of circuit boards all rendered in 3D CAD](overview.png)

This project is a "hexpansion" for the [EMF Camp Tildagon badge](https://tildagon.badge.emfcamp.org/).
It adds a motor driver, encoder reader and mounting points for an N20 motor.  The motor is mounted
straight out from the slot the hexpansion is mounted in.  By installing 3 of these hexpansions and
programming the badge appropriately you should be able to make it rotate or move in any direction
desired.

# Things you need to make a complete Omni Wheel Hexpansion

1. A Motor Hexpansion board as described in the kicad\_project folder
2. A 60mm Omni Wheel, see [my Omni Wheel repository](https://github.com/hairymnstr/OmniWheel) for a 3D printable design
3. A 110:1 geared micro metal motor with extended back shaft [COM0818 from Pimoroni](https://shop.pimoroni.com/products/micro-metal-gearmotor-extended-back-shaft?variant=39421592043603)
4. A long mounting bracket for the motor [COM0817 from Pimoroni](https://shop.pimoroni.com/products/long-micro-metal-gearmotor-bracket-pair?variant=39861109850195)
5. An encoder for the motor [PIM604 from Pimoroni](https://shop.pimoroni.com/products/micro-metal-motor-encoder?variant=39888423321683)
6. A JST-SH 6 way cable to connect the encoder/motor to the hexpansion [CAB1009 from Pimoroni](https://shop.pimoroni.com/products/jst-sh-cable-6-pin?variant=39292536455251)
7. A couple of short M2 bolts to secure the hexpansion to your badge [M2 x 5mm Torx Pan head bolt](https://www.westfieldfasteners.co.uk/Bolts_Screws_Metric/Torx_Pan_Screw_M2x5_A2_Stainless.html)

You'll need 3 of each part to make a complete hexpansion.

# Programming the hexpansion

To program a blank hexpansion I used the Raspberry Pi debug probe [(available from Pimoroni)](https://shop.pimoroni.com/products/raspberry-pi-debug-probe?variant=40511574999123).
The probe can be connected to both the SWD pins and the UART for test and debug.  The firmware
is also in this repository and can be built easily using Visual Studio Code or just by running
the Makefile provided you have an ARM embedded compiler installed.

# Hexpansion API

The hexpansion has a very simple interface, everything is controlled via I2C.

There are 2 devices presented (both in software on the STM32):

* 0x50 is a 256 byte fake EEPROM which has a valid "hexpansion" header identifying the board
* 0x42 has a register based control and status interface

Both devices operate in "memory" mode, i.e. they take an address then data to be read or written.
Both use single byte addressing.  Using the Micropython `readfrom_mem` and `writeto_mem` functions
will work.

There are only 2 registers defined, they can both be read and written, reading or writing any
other part of the 256 byte address space will be stored but won't make any difference to the
operation of the device.  Reads are guaranteed to be coherent so you won't get part of a register
that's updated between starting the I2C transaction and finishing it.

* Address 0, length 16 bit little endian: PWM value, a value in the range -100 to 100
* Address 2, length 16 bit little endian: Encoder position

Using the badge software you can use `machine.I2C` and `struct` to access these registers.
Note that on the Tildagon badge there are I2C buses for each hexpansion so you will need
to change the paramter to `I2C` for each slot you connect to.

This trivial example sets the speed to full reverse then prints out the current
encoder value.

    import struct
    import machine

    bus = machine.I2C(2)
    bus.writeto_mem(0x42, 0, struct.pack("<h", -100))
    print(struct.unpack("<h", bus.readfrom_mem(0x42, 2, 2))[0])

# Questions I think might be asked
## Why all the fuss about encoders?

Well originally I was planning a pen holding hexpansion as well and making a floor-turtle type
system.  To get any kind of accurate and repeatable motions I needed to know how far the wheels
were actually turning as there are variations between motors especially when under load.

## What's the point of the external MCU?

You could drive the motor driver and maybe do the encoder reading all from the MCU on the badge
itself.  But, I wanted to make sure I as catching all that encoder information on all three
motors.  To do that I decided a dedicated encoder controller would be good but couldn't find
one.  I also looked at using an I2C H-Bridge and adding an I2C EEPROM for board identification
and realised using one of the very cheap STM32G0 MCUs I could emulate an EEPROM and make an I2C
controlled motor driver and have a dedicated hardware encoder interface all in one.
