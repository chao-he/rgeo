#pragma once

#include <string>
#include <unordered_map>

class AddressBook {
  public:
    AddressBook(const std::string& path, const std::string& tag);
    uint32_t Addr2Id(const std::string &addr) const;
    const std::string& Id2Addr(uint32_t id) const;

  private:
    std::unordered_map<uint32_t, std::string> id2addr_;
    std::unordered_map<std::string, uint32_t> addr2id_;
};
