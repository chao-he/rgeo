
#include <fstream>
#include "addr_dict.h"

AddressBook::AddressBook(const std::string& path, const std::string& tag) {
  std::ifstream ifs(path + "/" + tag + ".dic");
  std::string name;
  uint32_t id;
  while(ifs>>id>>name) {
    id2addr_[id] = name;
    addr2id_[name] = id;
  }
  ifs.close();
}

uint32_t AddressBook::Addr2Id(const std::string &addr) const {
  auto it = addr2id_.find(addr);
  if (it == addr2id_.end()) return 0;
  return it->second;
}

const std::string& AddressBook::Id2Addr(uint32_t id) const {
  static std::string kEmpty;
  auto it = id2addr_.find(id);
  if (it == id2addr_.end()) return kEmpty;
  else return it->second;
}
