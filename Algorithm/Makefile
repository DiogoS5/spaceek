# Makefile para o programa ndn.c

# Compiladpor
CC = gcc

# Flags para compilar
CFLAGS = -Wall
LDFLAGS = -lm

# Ficheiros que serão compilados
SRCS = main.c

# Objetos a criar
OBJS = $(SRCS:.c=.o)

# Nome do executável
TARGET = target

# O target default
all: $(TARGET)

# Compilar o executável
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compila os objetos
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Apagar executável
clean:
	rm -f $(OBJS) $(TARGET)