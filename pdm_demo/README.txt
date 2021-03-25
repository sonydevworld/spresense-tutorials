
Usage of pdm_demo
=================

Usage
---------------------------

Use the pdm_demo default configuration

$ ./tools/config.py examples/pdm_demo

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

Configure
--------------------------

You need an sdcard on your spresense board for storing measurements.
It is also used for the app support files, which are setup like this:

- Modify the file config/pdm.config to contain your wifi connection
  details, senseye.io credentials and sensor id to use.

- Copy the file config/pdm.config to the root of your sdcard.

- Copy the FFT binary examples/pdm_demo/fft/dsp/DSPPDM to a
  folder named "BIN" on your sdcard.

Also note that the file config/certs.h contains the CA certificates
of the Senseye.io API. Update this in case you implement support for
some other API.

Execute
--------------------------

The pdm_demo configuration includes the Nuttx configuration
CONFIG_USER_ENTRYPOINT="pdm_demo_main" to make the app run
automatically at boot.

If you remove the USER_ENTRYPOINT config, type 'pdm_demo' on nsh:
nsh> pdm_demo

Note that you need a Spresense WiFi-module mounted on the board for
network access.

Also note that the application tries to sync the board system time
over the internet using NTP. If this is unsuccessful the board will
reboot to reset the network connection and try again.

Sampling
--------------------------

PCM capture is done using the following settings.

Sampling rate 48KHz
Channel number MONO
Bit length 16 bits

