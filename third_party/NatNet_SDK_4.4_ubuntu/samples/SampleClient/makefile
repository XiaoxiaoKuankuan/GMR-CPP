include=-I../../include
libpath=-L../../lib
libs=-lNatNet

all:build/SampleClient

build/SampleClient: 
	$(CXX) SampleClient.cpp $(include) $(libpath) $(libs) -o build/SampleClient

.PHONY: clean
clean:
	@rm -f ./build/SampleClient
