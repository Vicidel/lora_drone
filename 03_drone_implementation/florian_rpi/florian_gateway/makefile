CXXFLAGS=-Wall

sources = run.cpp serial.cpp tools.cpp tcp_client.cpp
objects = $(sources:.cpp=.o)

%: %.o
	$(LINK.cpp) -o $@ $^

run: -lpthread $(objects)
run.o: run.cpp serial.h tcp_client.h
tcp_client.o: tcp_client.cpp tcp_client.h
serial.o: serial.cpp serial.h tcp_client.h
tool.o: tools.cpp tools.h


clean:
	rm -f *~ *.o run

depend:
	makedepend $(sources)

