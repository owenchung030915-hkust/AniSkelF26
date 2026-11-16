# macOS / Linux build. The Visual Studio project remains the course build.
FLTK_CXX := $(shell fltk-config --use-gl --cxxflags)
FLTK_LD  := $(shell fltk-config --use-gl --ldflags)

CXX      := c++
CXXFLAGS := -std=c++14 -O2 -Wall -Wno-unused-function -Wno-deprecated-declarations -Wno-reorder -Wno-unused-variable $(FLTK_CXX)
LDFLAGS  := $(FLTK_LD)

SRCS := \
	beziercurveevaluator.cpp \
	bsplinecurveevaluator.cpp \
	c2curveevaluator.cpp \
	camera.cpp \
	catmullromcurveevaluator.cpp \
	curve.cpp \
	curveevaluator.cpp \
	curveoptions.cpp \
	bitmap.cpp \
	graphwidget.cpp \
	indicatorwindow.cpp \
	linearcurveevaluator.cpp \
	modelerapp.cpp \
	modelerdraw.cpp \
	modelerui.cpp \
	modeleruiwindows.cxx \
	modelerview.cpp \
	particle.cpp \
	particleSystem.cpp \
	point.cpp \
	rect.cpp \
	robotarm.cpp \
	rulerwindow.cpp \
	scene.cpp \
	tga.cpp

OBJS := $(SRCS:.cpp=.o)
OBJS := $(OBJS:.cxx=.o)

.PHONY: all clean test

all: animator

animator: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS)

test: test_curves
	./test_curves

test_curves: test_curves.cpp beziercurveevaluator.cpp bsplinecurveevaluator.cpp \
		catmullromcurveevaluator.cpp c2curveevaluator.cpp curveevaluator.cpp \
		linearcurveevaluator.cpp point.cpp
	$(CXX) -std=c++14 -O2 -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.cxx
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) animator test_curves
