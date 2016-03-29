PREPROCESS=gcc -E -C -x c -iquote ./libraries

# -E : Stop after preprocessing.
# -C : Don't discard comments.
# -x c : Treat the file as C code.
# -iquote ./src : Use ./src for the non-system include path.

TARGETS=drone.h drone.bin

all: $(TARGETS)

clean:
	rm $(TARGETS)

%.h: %.h.in libraries/*/*.h libraries/*/*.cpp
	$(PREPROCESS) $< -o $@

%.bin: %.cpp libraries/*/*.h libraries/*/*.cpp
	g++ -g -iquote ./sketches -iquote ./libraries $< -o $@