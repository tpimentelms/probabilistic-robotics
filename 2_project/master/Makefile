CXX = g++
CPPFLAGS = -Wall

SOURCES = Master.cpp
OBJECTS = $(patsubst %.cpp, %.o, $(SOURCES))
EXECUTABLE = Master

SHARED_OBJ = ../shared/Config.o ../shared/ProcessLogger.o
BOOST_LIBS = -lboost_thread -lboost_system -lboost_filesystem -lboost_regex -lboost_program_options

INC = -I. -I../shared

.PHONY: all
all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CPPFLAGS) $(INC) $(SHARED_OBJ) $(BOOST_LIBS) -lcurses $^ -o $@

.cpp.o:
	$(CXX) $(CPPFLAGS) $(INC) -c -g $< -o $@

.PHONY: print
print:
	@echo Sources: $(SOURCES)
	@echo Objects: $(OBJECTS)
	@echo Executable: $(EXECUTABLE)

.PHONY: clean
clean:
	-rm -f $(EXECUTABLE) *.o
