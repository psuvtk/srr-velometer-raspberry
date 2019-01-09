CC = gcc
CFLAGS = --std=gnu99
LDFLAGS = -lm -lwiringPi
OBJS = main.o radar_device.o bytearray.o radar_packet.o display.o

srr-velometer-raspberry: $(OBJS)
	$(CC) $(CFLAGS)  $(OBJS) $(LDFLAGS) -o srr-velometer-raspberry

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean install
clean:
	rm -f $(OBJS)
	rm -f srr-velometer-raspberry

install:
	mv srr-velometer-raspberry /home/pi/bin/
