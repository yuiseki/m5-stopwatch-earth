CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude
PYTHON ?= python3

.PHONY: test test-cpp test-py clean

test: test-cpp test-py

test-cpp: build/test_globe_math build/test_frame
	./build/test_globe_math
	./build/test_frame

test-py:
	$(PYTHON) -I -W error::ResourceWarning test/test_tools.py

build/test_globe_math: test/test_globe_math.cpp include/globe_math.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

build/test_frame: test/test_frame.cpp include/frame.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

build/host_render: tools/host_render.cpp include/globe_math.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

build/interp_eval: tools/interp_eval.cpp include/globe_math.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -rf build
