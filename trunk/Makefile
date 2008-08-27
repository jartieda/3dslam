OBJS=datacam.o \
estimator.o \
map.o \
particlefilter.o \
tracker_surf.o \
tracker_harris.o \
xmlParser.o \
dataout.o \
freecam.o \
modelcam.o \
surf.o \
updater.o \
elempunto.o \
kalman.o \
model.o \
trackerfile.o \
tracker.o

LDFLAGS=`pkg-config --libs opencv` -lgsl -lgslcblas -L"/opt/jdk1.6.0_06/jre/lib/i386/client" -ljvm
CFLAGS=`pkg-config --cflags opencv` -I"/opt/jdk1.6.0_06/include" -I"/opt/jdk1.6.0_06/include/linux" -g 

%.o:./src/%.cpp
	g++ $(CFLAGS) -c $^ -o $@

all:$(OBJS) test2.o
	g++ $(OBJS) test2.o -o slam $(LDFLAGS) $(CFLAGS)

test2.o:./test1/test2.cpp
	g++ $(CFLAGS) -c $? -o $@

