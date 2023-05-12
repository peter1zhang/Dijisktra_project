CXX = g++
CXXFLAGS = -std=c++98 -Wall -Wextra -pedantic

OBJS1 = path-step1.o graph.o point.o path.o edge.o
OBJS2 = path-step2.o graph.o point.o path.o edge.o
OBJS3 = path-step3.o graph.o point.o path.o edge.o
OBJS4 = path-step4.o graph.o point.o path.o edge.o

all: path-step1 path-step2 path-step3 path-step4

path-step1: $(OBJS1)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS1)

path-step1.o: path-step1.cpp graph.h point.h path.h edge.h
	$(CXX) $(CXXFLAGS) -c $<

path-step2: $(OBJS2)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS2)

path-step2.o: path-step2.cpp graph.h point.h path.h edge.h
	$(CXX) $(CXXFLAGS) -c $<

path-step3: $(OBJS3)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS3)

path-step3.o: path-step3.cpp graph.h point.h path.h edge.h
	$(CXX) $(CXXFLAGS) -c $<

path-step4: $(OBJS4)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS4)

path-step4.o: path-step4.cpp graph.h point.h path.h edge.h
	$(CXX) $(CXXFLAGS) -c $<

graph.o: graph.cpp graph.h point.h path.h edge.h
	$(CXX) $(CXXFLAGS) -c $<

edge.o: edge.cpp edge.h point.h
	$(CXX) $(CXXFLAGS) -c $<

point.o: point.cpp point.h
	$(CXX) $(CXXFLAGS) -c $<

path.o: path.cpp path.h
	$(CXX) $(CXXFLAGS) -c $<

.PHONY: clean
clean:
	rm -f *.o path-step1 path-step2 path-step3 path-step4
