CC = g++  
ERR =  -Wall -Werror -Wextra
OPT1 = -O3 -flto -fuse-linker-plugin 
CFLAGS = $(ERR) $(OPT1) 

OBJECTS = obj/KC_heap.o obj/KC_searching.o obj/KC_structs.o obj/KC_testing.o obj/KC_search_params.o obj/KC_algorithms.o
HEADERS = include/rassert.hpp include/common.hpp include/KC_astar.hpp include/KC_heap.hpp include/KC_searching.hpp include/KC_structs.hpp include/KC_search_params.hpp include/KC_algorithms.hpp

OUTPUT = mesh_astar

$(OBJECTS): obj/%.o  : src/%.cpp $(HEADERS) 
	$(CC) $(CFLAGS) -I include -c  $< -o $@

$(OUTPUT): obj $(OBJECTS)
	$(CC) $(CFLAGS) -I include -o $@ $(OBJECTS)

obj: 
	mkdir -p obj
	mkdir -p res

clean:
	rm -rf obj
	rm -rf $(OUTPUT)
