# Compile all cpp files in src folder to produce fw file

all: fw

fw: $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))
	g++ -o $@ $^

%.o: %.cpp
	g++ -c $< -o $@

clean:
	rm -rf src/*.o fw

.PHONY: all clean