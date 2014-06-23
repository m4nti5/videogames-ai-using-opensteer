all: cpphop block_world_example simple_travel_example

simple_travel_example: cpphop simple_travel_example.cpp
	g++ -g -o simple_travel_example simple_travel_example.cpp cpphop.o -lboost_system -lboost_thread
	
block_world_example: block_world_example.cpp
	g++ -g -o block_world_example block_world_example.cpp cpphop.o -lboost_system -lboost_thread

cpphop: cpphop.cpp cpphop.hpp
	g++ -g -c cpphop.cpp
	
clean:
	rm -rf cpphop.o simple_travel_example block_world_example
