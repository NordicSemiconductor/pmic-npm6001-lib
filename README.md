# nPM6001 PMIC library

This repository contains a sample library for the nPM6001 PMIC.

The library builds on top of the register map and bit value files, and offers a higher level abstraction for configration and control of the nPM6001 PMIC.

Hardware-specific functions for Two-Wire (TWI) communication and GPIO signalling is not implemented here. This functionality must be provided as function pointers given to the library.

## nRF Connect SDK compatibility

Although the library source code is platform agnostic, this repository is structured as a Zephyr module to ease integration with nRF Connect SDK.

## Platform porting

The library considers portability, such that the core nPM6001 functions can be easily used on platforms other than nRF Connect SDK.
The hardware-dependent functions relate to the TWI communication interface.

To port the sample driver to a different platform, you need to implement the following functions:

* `lib_npm6001_platform_init`
* `lib_npm6001_twi_read`
* `lib_npm6001_twi_write`

These functions are described in `lib_npm6001.h`.

You can optionally implement the following functionality:

* Using GPIO output to control BUCK_MODE pins.
* Using GPIO input to detect interrupts signals from nPM6001 nINT pin.

When the nINT pin signal is asserted, the `lib_npm6001_int_read` function can be called to read and clear the interrupt source.
