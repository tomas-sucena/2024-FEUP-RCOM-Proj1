# Makefile to build the project
# NOTE: This file must not be changed.

# Parameters
CC = gcc
CFLAGS = -Wall

SRC = src/
INCLUDE = include/
BIN = bin/
CABLE_DIR = cable/

TX_SERIAL_PORT = /dev/ttyS10
RX_SERIAL_PORT = /dev/ttyS11

BAUD_RATE = 9600

TX_FILE = penguin.gif
RX_FILE = penguin-received.gif

TX_LOGS = logs_tx.txt
RX_LOGS = logs_rx.txt

# Targets
.PHONY: all
all: $(BIN)/main $(BIN)/cable

$(BIN)/main: main.c $(SRC)/*.c
	@ mkdir -p $(BIN)
	$(CC) $(CFLAGS) -o $@ $^ -I$(INCLUDE)

$(BIN)/cable: $(CABLE_DIR)/cable.c
	@ mkdir -p $(BIN)
	$(CC) $(CFLAGS) -o $@ $^

.PHONY: run_tx
run_tx: $(BIN)/main
	./$(BIN)/main $(TX_SERIAL_PORT) $(BAUD_RATE) tx $(TX_FILE)

.PHONY: run_rx
run_rx: $(BIN)/main
	./$(BIN)/main $(RX_SERIAL_PORT) $(BAUD_RATE) rx $(RX_FILE)

.PHONY: run_cable
run_cable: $(BIN)/cable
	sudo ./$(BIN)/cable

.PHONY: check_files
check_files:
	diff -s $(TX_FILE) $(RX_FILE) || exit 0

.PHONY: clean
clean:
	@ rm -rf $(BIN)
	@ rm -f $(RX_FILE) $(TX_LOGS) $(RX_LOGS)
