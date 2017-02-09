

#include <sdsl/int_vector_mapper.hpp>
#include <iostream>
#include <fstream>
#include <unordered_map>

typedef sdsl::int_vector_mapper<64, std::ios::in> ivm64;
typedef sdsl::int_vector_mapper<32, std::ios::in> ivm32;

#include <geometry/s2/s2cellid.h>
#include <geometry/s2/s2latlng.h>

void Id2LatLng(uint64_t id, float *lat, float *lng, int *level) {
  S2CellId sid(id);
  S2LatLng ll = sid.ToLatLng();
  if (lat) *lat = ll.lat().degrees();
  if (lng) *lng = ll.lng().degrees();
  if (level) *level = sid.level();
}

int main(int argc, char **argv) {

  std::string path(argv[1]);
  const ivm64 a(path + ".loc");
  const ivm32 b(path + ".pos");

  std::unordered_map<uint32_t, std::string> names;
  std::ifstream ifs(path + ".dic");
  int code;
  std::string name;
  while (ifs>>code>>name) {
    names[code] = name;
  }

  float lat, lng;
  for (size_t i = 0; i < a.size(); ++ i) {
    Id2LatLng(a[i], &lat, &lng, NULL);
    std::cout <<  std::setprecision(8)
      << names[b[i]] << "\t" << lat << "\t" << lng << "\n";
  }
}
