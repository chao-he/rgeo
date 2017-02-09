#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "progress.h"
#include "geo_decoder.h"

struct Record {
  uint64_t uid;
  double lat;
  double lng;
  uint32_t n;
  uint32_t c;
};

std::istream& operator>>(std::istream& os, Record& rec) {
  os>>rec.uid
    >>rec.lat
    >>rec.lng
    >>rec.n
    >>rec.c;
  return os;
}

static Progress pr(100000, std::cerr);

void Process(const GeoDecoder& decoder, const std::string& fname) {
  std::ifstream ifs(fname);
  std::ofstream ofs(fname + ".ext");
  ofs.precision(8);
  Record rec;
  Address addr;
  std::string line;
  while(std::getline(ifs, line)) {
    std::istringstream iss(line);
    if ( iss >> rec ) {
      addr.street.clear();
      addr.poi_list[0].poiid = 0;
      decoder.Decode(rec.lat, rec.lng, &addr);
      ofs <<std::dec << rec.uid
        << "\t" << rec.lat
        << "\t" << rec.lng
        << "\t" << addr.adcode
        << "\t" << addr.street
        << "\t";
      for (int i = 0; i < 10; ++ i) {
        if (addr.poi_list[i].poiid) {
          if (i > 0)
            ofs << "," << std::hex << addr.poi_list[i].poiid;
          else
            ofs << std::hex << addr.poi_list[i].poiid;
        } else break;
      }
      ofs << "\n";
    } else {
      std::cerr << "invalid " << line << "\n";
    }
    ++ pr;
  }
  ifs.close();
  std::cerr << "process " << fname << " done\n";
}

int main(int argc, char** argv) {
  GeoDecoder decoder("data");
  std::vector<std::thread> ws;
  for (int i = 1; i < argc; ++ i) {
    ws.emplace_back([&decoder, i, argv]{ Process(decoder, argv[i]); });
  }
  for (auto& w: ws) {
    w.join();
  }
  return 0;
}
