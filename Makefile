
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


Cam.o:	Cam.cpp
	$(CXX) -c $(CXXFLAGS) Cam.cpp

KinChain.o: KinChain.cpp
	$(CXX) -c $(CXXFLAGS) KinChain.cpp
	
	
Test_HTM.o:	Test_HTM.cpp HTM.o KinChain.o
	$(CXX) -c $(CXXFLAGS) Test_HTM.cpp

Test_HTM:	Test_HTM.o HTM.o KinChain.o Cam.o
	$(CXX) -o Test_HTM Test_HTM.o HTM.o KinChain.o Cam.o  $(LFLAGS)


Test_Cam.o:	Test_Cam.cpp Cam.o 
	$(CXX) -c $(CXXFLAGS) Test_Cam.cpp


Test_Cam:	Test_Cam.o Cam.o 
	$(CXX) -o Test_Cam Test_Cam.o Cam.o $(LFLAGS)


Test_KinChain.o:	Test_KinChain.cpp HTM.o Cam.o KinChain.o
	$(CXX) -c $(CXXFLAGS) Test_KinChain.cpp


Test_KinChain:	Test_KinChain.o Cam.o KinChain.o HTM.o 
	$(CXX) -o Test_KinChain Test_KinChain.o Cam.o KinChain.o HTM.o $(LFLAGS)

test:	Test_HTM 

obj:    HTM.o KinChain.o Cam.o 

test:	Test_HTM Test_Cam Test_KinChain
	echo "done"
	
run:	
	echo "done"
	
doc:	
	doxygen


clean:
	rm -rf DOXYGENDOC *.o Test_HTM Test_Cam Test_KinChain
	
