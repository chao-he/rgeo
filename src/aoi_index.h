#ifndef _H_AOI_INDEX_H_
#define _H_AOI_INDEX_H_

#include <map>
#include <string>
#include <geometry/s2/s2loop.h>
// #include <sdsl/int_vector_mapper.hpp>
#include "addr_dict.h"

// typedef sdsl::int_vector<64> iv64;
// typedef sdsl::int_vector<32> iv32;
// typedef sdsl::int_vector<16> iv16;
// typedef sdsl::int_vector<8>  iv8;
// typedef sdsl::int_vector<1>  bitv;
//

// typedef sdsl::int_vector_mapper<64, std::ios::in> ivm64;
// typedef sdsl::int_vector_mapper<32, std::ios::in> ivm32;
// typedef sdsl::int_vector_mapper<16, std::ios::in> ivm16;
// typedef sdsl::int_vector_mapper<8, std::ios::in>  ivm8;
// typedef sdsl::int_vector_mapper<1, std::ios::in>  bitvm;

enum AOIType {
  kUnknown = 0,
  kAdmin = 1, // 行政区划
  kBArea = 2, // 功能面
  kTArea = 3, // 商圈
  kBuild = 4  // 大厦
};

struct AOI {
  int id;
  int adcode; // 行政编码
  int clazz;  // 类型
  int area;   // 面积
  int circum; // 周长
  int latE6;  // 中心点的经纬度
  int lngE6;  // 

  double lat_lo;
  double lat_hi;
  double lng_lo;
  double lng_hi;

  char name[64];
  S2Loop* bounds;

  AOI():
    id(0),
    adcode(0),
    clazz(0),
    area(0),
    circum(0),
    latE6(0),
    lngE6(0),
    bounds(NULL) { }

  AOI(int id):
    id(id),
    adcode(0),
    clazz(0),
    area(0),
    circum(0),
    latE6(0),
    lngE6(0),
    bounds(NULL) { }

  AOI(int id, int adcode, int clazz, int area, int circum,
      int latE6, int lngE6, double lat_lo, double lat_hi, double lng_lo, double lng_hi,
      const std::string& an):
    id(id),
    adcode(adcode),
    clazz(clazz),
    area(area),
    circum(circum),
    latE6(latE6),
    lngE6(lngE6),
    lat_lo(lat_lo),
    lat_hi(lat_hi),
    lng_lo(lng_lo),
    lng_hi(lng_hi),
    bounds(NULL) {
      strncpy(name, an.c_str(), sizeof(name));
    }

  ~AOI() {
    if (bounds) {
      delete bounds;
      bounds = NULL;
    }
  }

  bool operator==(const AOI &other) const {
    return id < other.id;
  }

  bool operator<(const AOI &other) const {
    return id < other.id;
  }
};

class RTree;

class AreaIndex {
  public:
    AreaIndex(const std::string& path, const std::string& tag);
    ~AreaIndex();
    // 返回包含Point(lat, lng)的区域ID, 如果有多个点,返回面积最小的区域
    int  Find(double lat, double lng) const;
    void Find(double lat, double lng, std::vector<int>& res) const;
    const AOI* GetEntry(int id) const;
  private:
    std::unique_ptr<RTree>      index;
    std::vector<AOI>            entries;
};

//
// class POIIndex {
//   public:
//     POIIndex(const std::string& path, const std::string& tag);
//     bool GetAddress(float lat, float lng, int tolerent, std::string* addr) const;
//     int GetCode(double lat, double lng, int tolerent) const;
//     int Nearest(double lat, double lng, int &distance) const;
//     const std::string& GetName(int idx) const;
//
//   private:
//     ivm64 locs;
//     ivm32 code;
//     AddressBook names;
// };

class LocIndex {
  public:
    struct Slot {
      uint64_t loc;
      uint64_t sid;
      std::string name;
    };

    LocIndex(const std::string& path, const std::string& tag);
    void Near(double lat, double lng, int max_distance_m, size_t n, std::vector<Slot>* res) const;
    uint64_t Nearest(double lat, double lng) const;

  private:
    std::vector<Slot> loc_index_;
};

#endif
