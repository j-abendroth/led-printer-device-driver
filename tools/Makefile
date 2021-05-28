CC = cc
CFLAGS  = -Wall -Wextra -Wpedantic -Wshadow -std=c99 -O2

default: ledtextctl

ledtextctl:  ledtextctl.c
	$(CC) $(CFLAGS) -o ledtextctl ledtextctl.c

clean: 
	rm ledtextctl
