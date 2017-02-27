CC=g++
CFLAGS=-g -c -Wall -pthread
LDFLAGS=
LIBS=-lboost_system -lpthread
SOURCES=findip.cpp request.cpp stdafx.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=findip

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(EXECUTABLE) $(OBJECTS)
