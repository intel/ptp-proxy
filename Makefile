# SPDX-License-Identifier: GPL-2.0
#
# Makefile for PTP Proxy kernel module
#
# Build:
#   make
#   sudo insmod ptp_proxy.ko
#
# Optional parameter:
#   sudo insmod ptp_proxy.ko cmd_timeout_ms=200
#

obj-m := ptp_proxy.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)
UDEV_RULES_DIR ?= /etc/udev/rules.d
PTP_PROXY_GROUP ?= ptp-proxy

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a
	getent group $(PTP_PROXY_GROUP) >/dev/null || groupadd --system $(PTP_PROXY_GROUP)
	install -D -m 0644 99-ptp-proxy.rules $(UDEV_RULES_DIR)/99-ptp-proxy.rules
	udevadm control --reload-rules
	udevadm trigger --subsystem-match=misc --sysname-match=ptp_proxy

.PHONY: all clean install
