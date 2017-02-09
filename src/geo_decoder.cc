
#include <geometry/s2/s2cellid.h>
#include "geo_decoder.h"
#include "addr_dict.h"
#include "aoi_index.h"

// translate between lat, lng and address
class GeoDecoder::GeoDecoderImpl {
  public:
    GeoDecoderImpl(const std::string& dbpath)
      : admin_index_(dbpath, "admin")
      , admin_name_(dbpath, "admin")
      // , town_(dbpath, "town")
      // , village_(dbpath, "village")
      , street_(dbpath, "road")
      , poi_(dbpath, "poi") {
      }

    const std::string& Adcode2Name(int adcode) const{
      static std::string kEmpty;
      if (adcode == 0) return kEmpty;
      else return admin_name_.Id2Addr(adcode);
    }

    int Decode(double lat, double lng, Address* addr) const {
      int adid = admin_index_.Find(lat, lng);

      if (adid == 0) {
        return adid;
      }

      int aid = (adid / 100) % 100;
      int cid = (adid / 10000) % 100;
      int pid = (adid / 1000000) % 100;
      addr->adcode = pid * 10000 + cid * 100 + aid;

      addr->province = Adcode2Name(pid * 10000);
      addr->district = Adcode2Name(addr->adcode);
      if (pid == 11 || pid == 12 || pid == 31 || pid == 50) {
        addr->city = addr->province;
      } else {
        addr->city = Adcode2Name(pid * 10000 + cid * 100);
      }
      if (addr->city.empty()) {
        addr->city = addr->district;
      }

      // resolve the street
      // street_.GetAddress(lat, lng, 300, &addr->street);
      //
      // if (addr->street.empty()) {
      //   village_.GetAddress(lat, lng, 2000, &addr->street);
      // }
      //
      // if (addr->street.empty()) {
      //   town_.GetAddress(lat, lng, 5000, &addr->street);
      // }

      std::vector<LocIndex::Slot> pois;
      street_.Near(lat, lng, 300, 5, &pois);
      if (!pois.empty()) {
        addr->street = pois[0].name;
      }
      pois.clear();
      poi_.Near(lat, lng, 200, 10, &pois);
      for (size_t i = 0; i < pois.size() && i < 10; ++ i) {
        addr->poi_list[i].poiid = std::strtoul(pois[i].name.c_str(), NULL, 16);
      }
      return adid;
    }

  private:
    AreaIndex   admin_index_;
    AddressBook admin_name_;
    LocIndex street_;
    LocIndex poi_;
    // POIIndex town_;
    // POIIndex village_;
    // POIIndex street_;
};


GeoDecoder::GeoDecoder(const std::string& dbpath)
  : impl_(new GeoDecoderImpl(dbpath)) {
  }

void GeoDecoder::Decode(double lat, double lng, Address* addr) const {
  impl_->Decode(lat, lng, addr);
}

GeoDecoder::~GeoDecoder() { }
