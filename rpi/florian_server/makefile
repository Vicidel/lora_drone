CXXFLAGS=-Wall -std=c++11

sources = server.cpp pid.cpp tools.cpp navigator.cpp
objects = $(sources:.cpp=.o)

%: %.o
	$(LINK.cpp) -o $@ $^

server: -lpthread $(objects)
server.o: server.cpp server.h navigator.h
pid.o: pid.cpp pid.h
tools.o: tools.cpp tools.h
navigator.o: navigator.cpp navigator.h


clean:
	rm -f *~ *.o server

depend:
	makedepend $(sources)

