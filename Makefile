# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -Wall -std=c++11 -g

# the list of header files
#INCL   = server.h
#the list of source files
SRC = cs.cc
#OBJ = $(SRC:.cpp=.o)

# the build target executable:
TARGET = cs

$(TARGET): $(OBJ)
#	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC)

$(OBJ): $(INCL)

clean:
	$(RM) $(TARGET) $(OBJ)
