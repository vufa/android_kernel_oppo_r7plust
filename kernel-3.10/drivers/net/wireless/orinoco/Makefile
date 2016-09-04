#
# Makefile for the orinoco wireless device drivers.
#
orinoco-objs := main.o fw.o hw.o mic.o scan.o wext.o r7plust_dld.o r7plust.o cfg.o

obj-$(CONFIG_R7PLUST)		+= orinoco.o
obj-$(CONFIG_PCMCIA_R7PLUST)	+= orinoco_cs.o
obj-$(CONFIG_APPLE_AIRPORT)	+= airport.o
obj-$(CONFIG_PLX_R7PLUST)	+= orinoco_plx.o
obj-$(CONFIG_PCI_R7PLUST)	+= orinoco_pci.o
obj-$(CONFIG_TMD_R7PLUST)	+= orinoco_tmd.o
obj-$(CONFIG_NORTEL_R7PLUST)	+= orinoco_nortel.o
obj-$(CONFIG_PCMCIA_SPECTRUM)	+= spectrum_cs.o
obj-$(CONFIG_ORINOCO_USB)	+= orinoco_usb.o

# Orinoco should be endian clean.
ccflags-y += -D__CHECK_ENDIAN__
