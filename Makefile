
INCLUDE_DIRS = -I/usr/local/include

LIB_PATH_GLOBAL = /usr/local/lib
LIB_PATH_GLOBAL = .

OPENCV_CPPFLAGS = $(shell pkg-config --cflags opencv)
OPENCV_LDLIBS = $(shell pkg-config --libs opencv)
CXX=g++
CXXFLAGS += -O2 -DLINK -Wall $(INCULDE_DIRS) $(OPENCV_CPPFLAGS)
LFLAGS += -I$(LIB_PATH_GLOBAL) -I$(LIB_PATH_LOCAL) -lpthread $(OPENCV_LDLIBS)


Test_HTM:   Test_HTM.o HTM.o KinChain.o



HTM.o:	HTM.cpp
	$(CXX) -c $(CXXFLAGS) HTM.cpp


KinChain.o: KinChain.cpp
	$(CXX) -c $(CXXFLAGS) KinChain.cpp
	
	
Test_HTM.o:	Test_HTM.cpp HTM.o KinChain.o
	$(CXX) -c $(CXXFLAGS) Test_HTM.cpp

Test_HTM:	Test_HTM.o HTM.o KinChain.o
	$(CXX) -o Test_HTM Test_HTM.o HTM.o KinChain.o $(LFLAGS)


test:	Test_HTM

obj:    HTM.o KinChain.o

run:	Test_HTM
	./Test_HTM
	
	
doc:	
	doxygen


clean:
	rm -rf DOXYGENDOC *.o Test_HTM 
	
