#include <iostream>
#include <fstream>
#include "progress.h"
#include "geo_decoder.h"

int main(int argc, char **argv) {
  GeoDecoder decoder("data");
  double lat, lng;
  std::ifstream ifs(argv[1]);
  Progress pr(100000, std::cerr);
  while(ifs>>lat>>lng) {
    Address addr;
    decoder.Decode(lat, lng, &addr);
    if (addr.adcode > 0) {
      std::cout << std::dec << lat << "," << lng
        << "," << addr.adcode
        << "," << addr.province
        << "," << addr.city
        << "," << addr.district
        << "," << addr.street;
      for (int i = 0; i < 10; ++ i) {
        if (addr.poi_list[i].poiid)
          std::cout << "," << std::hex << addr.poi_list[i].poiid;
      }
      std::cout << "\n";
    }
    ++ pr;
  }

  ifs.close();
  return 0;
}
