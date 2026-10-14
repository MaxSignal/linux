# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Casio fx-CG10/fx-CG20 (Prizm)
#
obj-y	:= setup.o console.o sound.o
obj-$(CONFIG_FXCG_STORAGE)	+= storage.o
obj-$(CONFIG_FXCG_ROM_INSTALL)	+= xipflash.o
ifneq ($(CONFIG_FXCG_STORAGE)$(CONFIG_FXCG_ROM_INSTALL),)
obj-y	+= nor.o
endif
