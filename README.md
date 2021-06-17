# Python nrf52 OTA BLE

This is a python program to perform a Device Firmware Update over BLE on nRF52832 devices.
It has been developed on Linux 14.04.
The target device should have SDK version 11 and must have a working DFU service.

## Prerequisite

This project depends by several modules:

- bluepy
	sudo apt-get install python-pip libglib2.0-dev
	sudo pip install bluepy

- nrfutil (version 0.5.2)
	sudo pip install --pre nrfutil==0.5.2

- pyqt4
	sudo apt-get install python-qt4

## Usage

Launch the program with the following command:

	sudo python gui.py

"sudo" is needed to scan BLE devices.

- Select a file to be uploaded by pressing the "Select File" button. The file can be a .bin (or .hex) together with .dat, a .zip (containing the bin and dat files), just an .hex file or just a .bin file (nrfutil will be used to generate the .dat file in the latest cases).
- Scan the BLE devices (the blank field below the scan button will be filled with the found devices).
- Select a device from the list: ota programming will start immediately.

## DFU only usage

In order to only use the dfu.py file to flash a firmware via OTA, use:

`python2 dfu.py -a YOURMAC -z YOURFIRMWARE.zip`

## Docker container

There is a Dockerfile which sets up an environment (no GUI) to flash OTA via DFU.

To build the image use e.g.

`docker build -t nrf52py2 .`

To run the image use e.g.

`docker run --rm -it --device=/dev/bus --net=host -v /local/path/to/app_dfu_package.zip:/app/app_dfu_package.zip nrf52py2 -a YOURMAC -z /app/app_dfu_package.zip`
