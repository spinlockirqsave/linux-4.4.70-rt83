#!/bin/sh

# remove all comedi modules to be able to use custom pcie215 driver
sudo rmmod amplc_dio200_pci
sudo rmmod amplc_dio200_common
sudo rmmod comedi_pci
sudo rmmod comedi_8254
sudo rmmod comedi_8255
sudo rmmod comedi

