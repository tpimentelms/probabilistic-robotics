CXX = g++
CPPFLAGS = -Wall

OBJECTS = Config.o ProcessLogger.o
HEADERS = $(patsubst %.cpp, %.h, $(SOURCES))

.PHONY: all
all: $(OBJECTS) 

.cpp.o: $(HEADERS)
	$(CXX) $(CPPFLAGS) -c -g $< -I.

.PHONY: print
print:
	@echo Objects: $(OBJECTS)
	@echo Headers: $(HEADERS)

.PHONY: clean
clean:
	-rm -f *.o
