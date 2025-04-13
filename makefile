# Makefile for lidar_reader

CC = gcc
CFLAGS = -O2 -Wall
TARGET = lidar_reader
SRC = lidar_reader.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) -lm

clean:
	rm -f $(TARGET)
