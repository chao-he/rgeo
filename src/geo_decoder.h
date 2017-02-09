#ifndef _H_QMAP_ADDRESS_H_
#define _H_QMAP_ADDRESS_H_

#include <map>
#include <string>
#include <memory>
#include <sstream>

struct Address {
  int adcode;
  std::string province;
  std::string city;
  std::string district;
  std::string street;
  struct {
    uint64_t poiid;
    int distance;
  } poi_list[10];

  Address(): adcode(0) {
    for (size_t i = 0; i < sizeof(poi_list) / sizeof(poi_list[0]); ++ i) {
      poi_list[i].poiid = 0;
    }
  }

  std::string str() const {
    std::stringstream  s;
    s << province << "," << city << "," << district << "," << street << "," << std::to_string(adcode);
    for (auto& poi: poi_list) {
      if (poi.poiid != 0) {
        s << "," << std::hex << poi.poiid << std::dec;
      }
    }
    return s.str();
  }
};

class GeoDecoder {
  public:
    GeoDecoder(const std::string& dbpath);
    ~GeoDecoder();

  public:
    void Decode(double lat, double lng, Address* addr) const;

  private:
    class GeoDecoderImpl;
    std::unique_ptr<GeoDecoderImpl> impl_;
};

#endif
