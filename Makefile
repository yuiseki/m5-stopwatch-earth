CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude

.PHONY: test clean

test: build/test_globe_math
	./build/test_globe_math

build/test_globe_math: test/test_globe_math.cpp include/globe_math.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

build/host_render: tools/host_render.cpp include/globe_math.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -rf build

build/interp_eval: tools/interp_eval.cpp include/globe_math.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<
