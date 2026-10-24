// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace forklift_hardware {
inline uint32_t crc32(const std::string &s) {
  uint32_t crc=0xffffffffu;
  for (unsigned char c:s) {
    crc^=c;
    for(int i=0;i<8;++i) crc=(crc>>1)^((crc&1)?0xedb88320u:0u);
  }
  return crc^0xffffffffu;
}
inline std::string frame(const std::string &s) {
  std::ostringstream o; o<<s<<'*'<<std::hex<<std::setfill('0')<<std::setw(8)<<crc32(s)<<'\n';
  return o.str();
}
inline uint64_t number(const std::string &s) {
  if(s.empty() || s.find_first_not_of("0123456789")!=std::string::npos) throw std::invalid_argument("uint");
  size_t n; auto v=std::stoull(s,&n); if(n!=s.size()) throw std::invalid_argument("uint"); return v;
}
inline int64_t signed_number(const std::string &s) {
  size_t n; auto v=std::stoll(s,&n); if(n!=s.size()) throw std::invalid_argument("int"); return v;
}
struct State {
  uint64_t boot=0, seq=0, us=0, ack=0;
  std::array<int64_t,4> ticks{};
  int64_t lift=0;
  uint64_t fault=0;
};
inline bool parse_state(std::string s,State &state) {
  try {
    if(!s.empty() && s.back()=='\n') s.pop_back();
    if(s.size()>256) return false;
    const auto at=s.find('*');
    if(at==std::string::npos || s.size()-at-1!=8) return false;
    auto payload=s.substr(0,at); size_t n;
    auto checksum=std::stoul(s.substr(at+1),&n,16);
    if(n!=8 || crc32(payload)!=checksum) return false;
    std::vector<std::string> fields; std::stringstream input(payload); std::string part;
    while(std::getline(input,part,',')) fields.push_back(part);
    if(fields.size()!=12 || fields[0]!="S" || fields[1]!="1") return false;
    State out;
    out.boot=number(fields[2]); out.seq=number(fields[3]); out.us=number(fields[4]); out.ack=number(fields[5]);
    for(size_t i=0;i<4;++i) out.ticks[i]=signed_number(fields[i+6]);
    out.lift=signed_number(fields[10]); out.fault=number(fields[11]); state=out;
    return true;
  } catch(const std::exception &) {return false;}
}
}  // namespace forklift_hardware
