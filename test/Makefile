TARGET	= Test_PID
CC		= cc
CFLAGS	= -g -O0 -W -Wall
INCLUDE	= -I../ -I./TesUT
SOURCES	= ./Test_PID.c ./TesUT/tesut.c ./TesUT/additions/tesut-console.c ./TesUT/additions/tesut-lib.c
OBJS	= $(notdir $(SOURCES:.c=.o))

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

$(OBJS): $(SOURCES)
	$(CC) $(CFLAGS) $(INCLUDE) -c $(SOURCES)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
