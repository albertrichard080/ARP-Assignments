CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=gnu99
LDLIBS  = -lm

BIN = bin/master bin/blackboard bin/drone bin/input bin/obstacles \
      bin/targets bin/watchdog bin/network

all: $(BIN)

bin/%: src/%.c src/common.c src/common.h
	@mkdir -p bin logs
	$(CC) $(CFLAGS) -o $@ $< src/common.c $(LDLIBS)

bin/input: src/input.c src/common.c src/common.h
	@mkdir -p bin logs
	$(CC) $(CFLAGS) -o $@ $< src/common.c $(LDLIBS) -lncurses

clean:
	rm -rf bin logs/*.log logs/pids.txt

.PHONY: all clean
