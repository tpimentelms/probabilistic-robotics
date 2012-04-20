CXX = g++
CPPFLAGS = -Wall

SOURCES = project1.cpp
OBJECTS = $(patsubst %.cpp, %.o, $(SOURCES))
EXECUTABLE = project1

SHARED_OBJ = ../shared/Config.o ../shared/ProcessLogger.o
BOOST_LIBS = -lboost_thread -lboost_system -lboost_filesystem -lboost_regex -lboost_program_options
PLAYER_LIBS = `pkg-config --cflags playerc++` `pkg-config --libs playerc++`

INC = -I. -I../shared

.PHONY: all
all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CPPFLAGS) $(INC) $(SHARED_OBJ) $(BOOST_LIBS) $(PLAYER_LIBS) -lcurses $^ -o $@

.cpp.o:
	$(CXX) $(CPPFLAGS) $(INC) $(PLAYER_LIBS) -c -g $< -o $@

.PHONY: print
print:
	@echo Sources: $(SOURCES)
	@echo Objects: $(OBJECTS)
	@echo Executable: $(EXECUTABLE)

.PHONY: clean
clean:
	-rm -f $(EXECUTABLE) *.o
