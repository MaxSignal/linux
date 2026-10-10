# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Casio fx-CG10/fx-CG20 (Prizm)
#
obj-y	:= setup.o console.o
obj-$(CONFIG_FXCG_STORAGE)	+= storage.o
obj-$(CONFIG_FXCG_XIP_ROMFS)	+= xip.o
obj-$(CONFIG_FXCG_XIP_INSTALL)	+= nor.o xipflash.o handoff.o
