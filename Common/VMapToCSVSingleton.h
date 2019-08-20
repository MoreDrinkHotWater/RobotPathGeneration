//
// Created by zhihui on 4/29/19.
//

#ifndef HDMAPS_COMMON_VMAPTOCSVSINGLETON_H_
#define HDMAPS_COMMON_VMAPTOCSVSINGLETON_H_

#include "DataStructure.h"

class VMapToCSVSingleton {
 public:
  static VMapToCSVSingleton *getInstance();

  void transAllTraceLinesToCSV(const std::string &dirPath) const;

 public:
  VMapToCSVSingleton(const VMapToCSVSingleton &) = delete;
  VMapToCSVSingleton &operator=(const VMapToCSVSingleton &) = delete;
  VMapToCSVSingleton(VMapToCSVSingleton &&) noexcept = delete;
  VMapToCSVSingleton &operator=(VMapToCSVSingleton &&) noexcept = delete;

 private:
//  void generateHighPrecisionTrace(const mdc::TraceLine &traceLine,
//                                  size_t &pID,
//                                  size_t &tlID,
//                                  std::vector<mdc::TraceLineHP> &traceLineHPs,
//                                  std::vector<mdc::CSVPoint> &CSVPoints,
//                                  std::vector<std::pair<std::vector<size_t>, mdc::TraceLine>> &HPLineIDs,
//                                  std::vector<std::pair<std::vector<size_t>, mdc::TraceLine>> &HPPointIDs) const;

  void generateHighPrecisionTrace(std::pair<std::vector<mdc::TraceLine>, int> &robotTraceLinePointNumPair,
                                  std::vector<std::pair<int, std::vector<mdc::CSVPoint>>> &robotTracePoints,
                                  int k) const;

  void updateTraceLineHP(const mdc::TraceLine &traceLine,
                         std::vector<mdc::TraceLineHP> &traceLineHPs,
                         const std::vector<std::pair<std::vector<size_t>, mdc::TraceLine>> &HPLineIDs) const;

  void saveTraceLineToDir(const std::string &dirPath,
                          const std::vector<std::pair<std::vector<mdc::TraceLine>, int>> &robotTraceLinePointNumPairs,
                          const std::vector<std::pair<int, std::vector<mdc::CSVPoint>>> &robotTracePoints) const;

 private:
  VMapToCSVSingleton() = default;
  ~VMapToCSVSingleton() = default;

 private:
  static VMapToCSVSingleton *instance;
};

std::ostream &operator<<(std::ostream &os, const mdc::TraceLineHP &obj);

std::ostream &operator<<(std::ostream &os, const mdc::CSVPoint &obj);

#endif //HDMAPS_COMMON_VMAPTOCSVSINGLETON_H_
