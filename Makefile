# Makefile taken from "FreeBSD Device Drivers: A Guide for the Intrepid" - Joseph Kong
KMOD= led_printer
SRCS= ./src/gpio_led.c
SRCS+=  bus_if.h device_if.h ofw_bus_if.h
SRCS+=  opt_platform.h gpio_if.h
.include <bsd.kmod.mk>