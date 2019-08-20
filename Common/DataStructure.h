//
// Created by zhihui on 3/28/19.
//

#ifndef POINTCLOUDAPPLICATION_DATASTRUCTURE_H
#define POINTCLOUDAPPLICATION_DATASTRUCTURE_H

#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <iostream>
#include <fstream>

namespace mdc {

// 所有ID从1开始计数，0表示不合法

struct Point {
  size_t pID;
  double b, l, h;
  double bx, ly;

  // 以该点为起始的线ID
  std::vector<size_t> fromPointLineID;
  // 以该点为终点的线ID
  std::vector<size_t> toPointLineID;

  size_t ref, mcode1, mcode2, mcode3;

  Point(size_t ID, double x, double y, double h) : pID(ID), b(0), l(0), h(h), bx(x), ly(y), ref(0), mcode1(0),
                                                   mcode2(0), mcode3(0) {}

  Point() : pID(0), b(0), l(0), h(0), bx(0), ly(0), fromPointLineID(), toPointLineID(), ref(0), mcode1(0),
            mcode2(0),
            mcode3(0) {}

  Point &operator=(const Point &point) = default;

  Point(const Point &point) = default;
};

struct Line {
  size_t lID;
  size_t sPID, ePID;

  // 从该线出去的线ID
  std::vector<size_t> fromThisLineID;
  // 到该线的线ID
  std::vector<size_t> toThisLineID;

  Line(size_t lID, size_t sPID, size_t ePID) : lID(lID), sPID(sPID), ePID(ePID) {}

  Line() : lID(0), sPID(0), ePID(0), fromThisLineID(), toThisLineID() {}

  Line &operator=(const Line &line) = default;

  Line(const Line &line) = default;
};

//  和上面的line完全一致，traceLine用于表示机器人行驶路径（不会分叉、也不会有环），还有后续处理
struct TraceLine {
  size_t tlID;
  size_t sPID, ePID;

  std::vector<size_t> fromThisLineID;
  std::vector<size_t> toThisLineID;

  TraceLine(size_t tlID, size_t sPID, size_t ePID) : tlID(tlID), sPID(sPID), ePID(ePID) {}

  TraceLine() : tlID(0), sPID(0), ePID(0), fromThisLineID(), toThisLineID() {}

  TraceLine &operator=(const TraceLine &traceLine) = default;

  TraceLine(const TraceLine &traceLine) = default;
};

struct CSVPoint {
  size_t pID;
  double x, y, z;

  int laneType;

  CSVPoint(size_t pID, double x, double y, double z, int laneType) : pID(pID), x(x), y(y), z(z), laneType(laneType) {}

  CSVPoint() : pID(0), x(0), y(0), z(0), laneType(0) {}

  CSVPoint &operator=(const CSVPoint &point) = default;

  CSVPoint(const CSVPoint &point) = default;
};

// 从trace得到的高精度线段
struct TraceLineHP {
  size_t tlID;
  size_t sPID, ePID;

  // 一般只有四个方向（左转、右转、直行、掉头）
  // 0表示无效
  size_t sTLID1, sTLID2, sTLID3, sTLID4;
  size_t eTLID1, eTLID2, eTLID3, eTLID4;

  TraceLineHP(size_t tlID, size_t sPID, size_t ePID)
          : tlID(tlID),
            sPID(sPID),
            ePID(ePID),
            sTLID1(0),
            sTLID2(0),
            sTLID3(0),
            sTLID4(0),
            eTLID1(0),
            eTLID2(0),
            eTLID3(0),
            eTLID4(0) {}

  TraceLineHP() : tlID(0),
                  sPID(0),
                  ePID(0),
                  sTLID1(0),
                  sTLID2(0),
                  sTLID3(0),
                  sTLID4(0),
                  eTLID1(0),
                  eTLID2(0),
                  eTLID3(0),
                  eTLID4(0) {}

  TraceLineHP &operator=(const TraceLineHP &traceLineHP) = default;

  TraceLineHP(const TraceLineHP &traceLineHP) = default;
};

struct Pavement {
  size_t pID;
  // 逆时针方向的四个点
  size_t upperLeftCorner, lowerLeftCorner, upperRightCorner, lowerRightCorner;

  explicit Pavement(size_t pID) : pID(pID), upperLeftCorner(0), lowerLeftCorner(0), upperRightCorner(0),
                                  lowerRightCorner(0) {}

  Pavement() : pID(0), upperLeftCorner(0), lowerLeftCorner(0), upperRightCorner(0), lowerRightCorner(0) {}

  Pavement &operator=(const Pavement &pavement) = default;

  Pavement(const Pavement &pavement) = default;
};

struct DrivingArrow {
  size_t daID;
  size_t centerPoint;
  std::string type;
  int typeIndex;
  // Y轴顺时针弧度制
  double rotation;

  explicit DrivingArrow(size_t daID) : daID(daID), centerPoint(0), type(), typeIndex(-1), rotation(0) {}

  DrivingArrow() : daID(0), centerPoint(0), type(), typeIndex(-1), rotation(0) {}

  DrivingArrow &operator=(const DrivingArrow &drivingArrow) = default;

  DrivingArrow(const DrivingArrow &drivingArrow) = default;
};

struct TrafficLights {
  size_t tlID;
  size_t lightLocationPoint;

  double rotation;

  explicit TrafficLights(size_t tlID) : tlID(tlID), lightLocationPoint(0), rotation(0) {}

  TrafficLights() : tlID(0), lightLocationPoint(0), rotation(0) {}

  TrafficLights &operator=(const TrafficLights &trafficLights) = default;

  TrafficLights(const TrafficLights &trafficLights) = default;
};

struct RoadLines {
  size_t rlID;

  std::string type;

  std::vector<size_t> allPointsID;

  explicit RoadLines(size_t rlID) : rlID(rlID), type(), allPointsID() {}

  RoadLines() : rlID(0), type(), allPointsID() {}

  RoadLines &operator=(const RoadLines &roadLine) = default;

  RoadLines(const RoadLines &roadLine) = default;
};

template<class T>
class Key {

 public:
  Key() = default;

  explicit Key(size_t ID) : ID(ID) {}

  void setId(size_t id) {
      ID = id;
  }

  size_t getID() const {
      return ID;
  }

  bool operator<(const Key<T> &right) const {
      return ID < right.getID();
  }

  bool operator==(const Key<T> &right) const {
      return ID == right.getID();
  }

 private:
  size_t ID;
};

template<class T>
using Filter = std::function<bool(const T &)>;

template<class T>
class Container {
 private:
  std::map<Key<T>, T> dataMap;

 public:
  Container() = default;

  void insert(const Key<T> &key, const T &t) {
      dataMap[key] = t;
      //C++17
      //auto [it, inserted] = dataMap.insert_or_assign(key, t);
  }

  void update(const Key<T> &key, const T &t) {
      dataMap[key] = t;
  }

  void remove(const Key<T> &key) {
      dataMap.erase(key);
  }

  void clear() {
      dataMap.clear();
  }

  T findByKey(const Key<T> &key) const {
      auto it = dataMap.find(key);
      if (it == dataMap.end())
          return T();
      return it->second;
  }

  std::vector<T> findByFilter(const Filter<T> &filter) const {
      std::vector<T> vector;
      for (const auto &pair : dataMap) {
          if (filter(pair.second))
              vector.push_back(pair.second);
      }
      return vector;
  }

  bool empty() const {
      return dataMap.empty();
  }

  size_t findMaxIndex() const {
      size_t index = 0;
      for (const auto &pair : dataMap) {
          size_t curIndex = std::get<0>(pair).getID();
          if (curIndex > index) index = curIndex;
      }
      return index;
  }

  void printAll() const {
      std::cout << "---------" << std::endl;
      for (const auto &pair : dataMap) {
          const Key<T> &key = std::get<0>(pair);
          const T &obj = std::get<1>(pair);
          std::cout << key.getID() << " --- " << obj << std::endl;
      }

      //C++17
      //for (const auto& [key, value] : dataMap) {
      //   std::cout << key.getID() << "--"<< value << std::endl;
      //}
  }

  void output(const std::string &filePath, const std::string &header) const {
      std::ofstream ofs(filePath);
      ofs << header << std::endl;

      for (const auto &pair : dataMap) {
          const T &obj = std::get<1>(pair);
          ofs << obj << std::endl;
      }
      ofs.close();
  }

  size_t size() const {
      return dataMap.size();
  }
};

template<class T>
std::vector<T> parse(const std::string &filePath) {
    std::ifstream ifs(filePath);
    std::string line;
    std::getline(ifs, line);
    std::vector<T> objs;
    while (std::getline(ifs, line)) {
        T obj;
        std::istringstream iss(line);
        iss >> obj;
        objs.push_back(obj);
    }
    return objs;
}
}

#endif //POINTCLOUDAPPLICATION_DATASTRUCTURE_H
