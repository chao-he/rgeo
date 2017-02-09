
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <geometry/s2/s2loop.h>
#include <geometry/util/coding/coder.h>
#include <geometry/s2/s2cellid.h>
#include <geometry/s2/s2latlng.h>
#include "aoi_index.h"
// #include "geo.h"

namespace {
  static const double kEarthRadius = 6378137;
  static const double kEarthCircumferenceMeters = 2 * M_PI * kEarthRadius; //40075017;

  inline double RadiansToEarthMeters(double radians) {
    return radians * kEarthRadius;
  }

  int DistanceOfId(uint64_t c1, uint64_t c2) {
    return RadiansToEarthMeters(
        S2CellId(c1).ToLatLng().GetDistance(
          S2CellId(c2).ToLatLng()).radians());
  }

  uint64_t LatLng2Id(double lat, double lng, int level) {
    return S2CellId::FromLatLng(S2LatLng::FromDegrees(lat, lng)).parent(level).id();
  }
}

typedef boost::geometry::model::d2::point_xy<double> point_t;
typedef boost::geometry::model::box<boost::geometry::model::d2::point_xy<double>>  box_t;
typedef std::pair<box_t, int> value_t;
typedef boost::geometry::index::rtree< value_t, boost::geometry::index::rstar<16,4> > rtree_t;

class RTree: public rtree_t { };

AreaIndex::~AreaIndex() {}

AreaIndex::AreaIndex(const std::string& path, const std::string& tag)
  : index(new RTree) {
  
  // load aoi data
  std::string index_path = path + "/" + tag + ".idx";
  std::ifstream ifs(index_path.c_str());
  std::string line;
  std::vector<std::string> parts; 
  bool sorted = true;
  while(std::getline(ifs, line)) {
    parts.clear();
    boost::split(parts, line, boost::is_any_of("\t,"));
    int id     = atoi(parts[0].c_str());
    int adcode = atoi(parts[1].c_str());
    int clazz  = atoi(parts[2].c_str());
    int area   = atoi(parts[3].c_str());
    int circum = atoi(parts[4].c_str());
    int latE6  = atoi(parts[5].c_str());
    int lngE6  = atoi(parts[6].c_str());

    if (!entries.empty() && id > entries.back().id) {
      sorted = false;
    }

    double lat_lo = strtod(parts[7].c_str(),  NULL);
    double lng_lo = strtod(parts[8].c_str(),  NULL);
    double lat_hi = strtod(parts[9].c_str(),  NULL);
    double lng_hi = strtod(parts[10].c_str(), NULL);
    entries.emplace_back(AOI(id, adcode, clazz, area, circum, latE6, lngE6, lat_lo, lat_hi, lng_lo, lng_hi, parts[11]));
  }
  ifs.close();

  if (!sorted) {
    std::sort(entries.begin(), entries.end());
  }
 
  // load bounds
  std::string bound_path = path + "/" + tag + ".dat";
  ifs.open(bound_path.c_str(), std::ios::binary);
  uint64_t hdr = 0;
  std::string coords;
  Decoder coder;
  while(ifs.read((char *)&hdr, 8)) {
    int len = hdr & 0xffffffff;
    int rid = (hdr >> 32) & 0xffffffff;
    coords.resize(len);
    ifs.read((char*)(coords.c_str()), len);
    auto it = std::lower_bound(entries.begin(), entries.end(), AOI(rid));
    if (NULL == it->bounds) {
      it->bounds = new S2Loop();
    }
    coder.reset(coords.c_str(), coords.length());
    it->bounds->Decode(&coder);
  }
  ifs.close();

  // build index
  std::vector<value_t> boxes;
  for (auto& aoi: entries) {
    boxes.push_back(std::make_pair(
          box_t(
            point_t(aoi.lat_lo, aoi.lng_lo),
            point_t(aoi.lat_hi, aoi.lng_hi)),
          aoi.id));
  }
  index->insert(boxes.begin(), boxes.end());

  // printf("aoi count = %ld\n", boxes.size());
}

const double kMaxArea = 4 * M_PI;

int AreaIndex::Find(double lat, double lng) const {
  std::vector<int> res;
  Find(lat, lng, res);
  if (res.empty()) {
    return 0;
  }

  size_t i,j;
  int min_area = 1000000000;
  for(i = 0, j = 0; i < res.size(); ++ i) {
    const AOI *aoi = GetEntry(res[i]);
    if (aoi->area < min_area) {
      min_area = aoi->area;
      j = i;
    }
  }
  return res[j];
}

void AreaIndex::Find(double lat, double lng, std::vector<int>& res) const {
  res.clear();
  std::vector<value_t> cands;
  point_t pt(lat, lng);
  index->query(boost::geometry::index::contains(pt), std::back_inserter(cands));

  if (cands.empty()) {
    double eps = 1e-5;
    box_t target(point_t(lat-eps, lng-eps), point_t(lat+eps, lng+eps));
    index->query(boost::geometry::index::intersects(target), std::back_inserter(cands));
  }

  // std::cout<<"find "<<cands.size()<<" candidates\n";
  S2Point point = S2LatLng::FromDegrees(lat, lng).ToPoint();
  for(const value_t &c: cands) {
    const AOI *aoi = GetEntry(c.second);
    assert (aoi != NULL);
    assert (aoi->bounds != NULL);
    // printf(" aoi = %d\n", aoi->id);
    if (aoi && aoi->bounds && aoi->bounds->Contains(point))
      res.push_back(c.second);
  }

  if (res.empty()) {
    S2Point point = S2LatLng::FromDegrees(lat+1e-4, lng+1e-4).ToPoint();
    for(const value_t &c: cands) {
      const AOI *aoi = GetEntry(c.second);
      assert (aoi != NULL);
      if (aoi && aoi->bounds && aoi->bounds->Contains(point))
        res.push_back(c.second);
    }
  }
}

const AOI* AreaIndex::GetEntry(int id) const {
  auto it = std::lower_bound(entries.begin(), entries.end(), AOI(id));
  if (it != entries.end() && it->id == id) {
    return &(*it);
  } else return NULL;
}

// POIIndex::POIIndex(const std::string& path, const std::string& tag)
//   : locs(path + "/" + tag + ".loc")
//   , code(path + "/" + tag + ".pos")
//   , names(path, tag) {
//   }
//
// bool POIIndex::GetAddress(float lat, float lng, int tolerent, std::string* addr) const {
//   int distance = tolerent + 1;
//   int idx = Nearest(lat, lng, distance);
//   if (distance > tolerent) return false;
//   *addr = GetName(idx);
//   return true;
// }
//
// const std::string& POIIndex::GetName(int idx) const {
//   return names.Id2Addr(idx);
// }
//
// int POIIndex::Nearest(double lat, double lng, int& distance) const {
//
//   uint64_t id = LatLng2Id(lat, lng, 22);
//   auto iter = std::lower_bound(locs.begin(), locs.end(), id);
//   if (*iter == id) {
//     distance = 0;
//     return code[iter-locs.begin()];
//   }
//   int step = 2;
//   int min_dist = 1<<30;
//   int acode = 0;
//
//   if (iter == locs.end())
//     iter = locs.begin() + std::max<int>(0, locs.size() - 2 * step);
//
//   int t = 0;
//   for(auto piter = iter; t < step && piter != locs.begin(); --piter, ++t) {
//     // debug("%lx - %lx\n", *piter, id);
//     int d = DistanceOfId(*piter, id);
//     if (d < min_dist) {
//       min_dist = d;
//       acode = code[piter-locs.begin()];
//     }
//   }
//
//   t = 0;
//   for(auto piter = iter + 1; t < step && piter != locs.end(); ++piter, ++t) {
//     // debug("%lx - %lx\n", *piter, id);
//     int d = DistanceOfId(*piter, id);
//     if (d < min_dist) {
//       min_dist = d;
//       acode = code[piter-locs.begin()];
//     }
//   }
//
//   distance = min_dist;
//   return acode;
// }
//
// int POIIndex::GetCode(double lat, double lng, int tolerent) const {
//
//   uint64_t id = LatLng2Id(lat, lng, 22);
//
//   auto iter = std::lower_bound(locs.begin(), locs.end(), id);
//
//   if (iter == locs.end())
//     iter = locs.end() - 1;
//
//   if (*iter == id) return code[iter-locs.begin()];
//   else {
//     int t = 0;
//     int step = 2;
//     for(auto piter = iter; t < step && piter != locs.begin(); --piter, ++t) {
//       double d = DistanceOfId(*piter, id);
//       // printf("%lx - %lx - %g\n",id, *piter, d);
//       if (d < tolerent) {
//         return code[piter-locs.begin()];
//       }
//     }
//     t = 0;
//     for(auto piter = iter + 1; t < step && piter != locs.end(); ++piter, ++t) {
//       double d = DistanceOfId(*piter, id);
//       // printf("%lx - %lx - %g\n",id, *piter, d);
//       if (d < tolerent) {
//         return code[piter-locs.begin()];
//       }
//     }
//   }
//
//   return 0;
// }

LocIndex::LocIndex(const std::string& dir, const std::string& tag) {
  std::string path = dir + "/" + tag + ".txt";
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error(path + " not found");
  }
  Slot s;
  std::string name;
  double x,y;
  do {
    if (ifs>>name>>std::dec>>x>>y) {
      s.name = name;
      s.loc = LatLng2Id(x, y, 22);
      loc_index_.emplace_back(s);
    }
  } while(!ifs.eof());
  ifs.close();

  std::sort(loc_index_.begin(), loc_index_.end(),
      [](const Slot& a, const Slot& b) { return a.loc < b.loc; });
  std::cerr << path << " -> " << loc_index_.size() << "\n";
}

uint64_t LocIndex::Nearest(double lat, double lng) const {
  std::vector<Slot> res;
  Near(lat, lng, 5000, 1, &res);
  return res.empty() ? 0 : res[0].sid;
}

void LocIndex::Near(double lat, double lng, int max_distance_m, size_t n, std::vector<Slot>* res) const {

  Slot t;
  t.loc = LatLng2Id(lat, lng, 22);
  auto iter = std::lower_bound(loc_index_.begin(), loc_index_.end(), t,
      [](const Slot& a, const Slot& b) { return a.loc <= b.loc; });

  // int tolerent = 1.5 * max_distance_m;

  size_t idx = iter - loc_index_.begin();
  size_t a = idx > n ? idx - n : 0;
  size_t b = idx + n < loc_index_.size() ? idx + n : loc_index_.size();
  for (size_t idx = a; idx < b; ++ idx) {
    int d = DistanceOfId(loc_index_[idx].loc, t.loc);
    // std::cout << idx << "\t" << loc_index_[idx].loc << "\t" << d << "\n";
    if (d <= max_distance_m) {
      res->emplace_back(loc_index_[idx]);
    }
    // if (d > tolerent || res->size() > 2 * n) break;
  }

  std::sort(res->begin(), res->end(),
      [&t] (const Slot& a, const Slot& b) {
        return DistanceOfId(a.loc, t.loc) < DistanceOfId(b.loc, t.loc);
        });
  if (res->size() > n) {
    res->erase(res->begin() + n);
  }
}

