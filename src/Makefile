CXXFLAGS = -g -O2 -std=c++11 -Wall -pipe

user_freq_loc: user_freq_loc.o geo_decoder.o aoi_index.o addr_dict.o
	g++ -pthread -std=c++11 $^ -o $@ /usr/local/lib/libs2.a -lcrypto

clean:
	rm -f *.o
