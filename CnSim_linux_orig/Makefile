CXX = g++

TARGET = cnSim
OBJECTS = $(patsubst %.cpp, %.o, $(wildcard *.cpp)) #./*.c
CFLAGS = -W -Wall -g -O2 -std=c++11

INCLUDE_DIR = -I.
LIBS = -lpthread

all : ${TARGET}

${TARGET} : ${OBJECTS}
	${CXX} ${CFLAGS} ${INCLUDE_DIR} ${LIB_DIR} -o  $@ ${OBJECTS} ${LIBS}

.SUFFIXES : .o .cpp

%.o : %.cpp
	${CXX} ${CFLAGS} ${INCLUDE_DIR} -c $<
	
clean :
	rm -rf *.o ${TARGET}