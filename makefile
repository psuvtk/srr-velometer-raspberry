CC = gcc
CFLAGS = --std=gnu99 -Wall
LDFLAGS = -lm -lwiringPi
OBJS = main.o radar_device.o bytearray.o radar_packet.o display.o

srr-velometer-raspberry: $(OBJS)
	$(CC) $(CFLAGS)  $(OBJS) $(LDFLAGS) -o srr-velometer-raspberry

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean install
clean:
	rm -f $(OBJS)

install:
	mv srr-velometer-raspberry ~/bin/
