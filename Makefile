CC=gcc
CFLAGS=-Wall -std=gnu99
LDFLAGS=-lwiiuse -lm -lbluetooth

.PHONY: all
all: kwii-input

kwii-input: kwii-input.o
	${CC} ${CFLAGS} $^ ${LDFLAGS} -o $@

kwii-input.o: kwii-input.c
	${CC} ${CFLAGS} $^ -c -o $@

.PHONY: clean
clean:
	rm *.o kwii-input