#compiler
CXX= g++
CXXFLAGS= -I/usr/local/include
LDFLAGS= -L/usr/local/lib -lompl -lboost_system-mt

all:
	$(CXX) $(CXXFLAGS) $(LDFLAGS) RigidBodyPlanning.cpp -o rigidbody	
