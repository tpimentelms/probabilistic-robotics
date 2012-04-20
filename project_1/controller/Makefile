CXX = g++
CPPFLAGS = -Wall

SOURCES = project1.cpp
EXECUTABLE = project1

PKG_CONFIG = `pkg-config --cflags playerc++` `pkg-config --libs playerc++`

.PHONY: all
all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	$(CXX) $(CPPFLAGS) $(PKG_CONFIG) $(SOURCES) -o $(EXECUTABLE)

.PHONY: print
print:
	@echo Sources: $(SOURCES)
	@echo Executable: $(EXECUTABLE)

.PHONY: clean
clean:
	-rm -f $(EXECUTABLE) *.o
