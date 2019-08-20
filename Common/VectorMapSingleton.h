//
// Created by zhihui on 3/27/19.
//

#ifndef POINTCLOUDAPPLICATION_VECTORMAPSINGLETON_H
#define POINTCLOUDAPPLICATION_VECTORMAPSINGLETON_H

#include "DataStructure.h"

enum Category : unsigned long {
  NONE = 0LLU,

  POINT = 1LLU << 0,
  LINE = 1LLU << 1,

  TraceLine = 1LLU << 2,

  Pavement = 1LLU << 3,
  DrivingArrow = 1LLU << 4,
  TrafficLights = 1LLU << 5,
  RoadLines = 1LLU << 6,

  ALL = (1LLU << 16) - 1
};

class VectorMapSingleton {
 public:
  static VectorMapSingleton *getInstance();

  size_t getMaxPointIndex() const;

  size_t getMaxLineIndex() const;

  size_t getMaxTraceLinesIndex() const;

  size_t getMaxPavementIndex() const;

  size_t getMaxDrivingArrowIndex() const;

  size_t getMaxTrafficLightsIndex() const;

  size_t getMaxRoadLinesIndex() const;

  void insert(const std::vector<mdc::Point> &points);

  void insert(const std::vector<mdc::Line> &lines);

  void update(const std::vector<mdc::Point> &points);

  void update(const mdc::Point &point);

  void update(const std::vector<mdc::Line> &lines);

  void update(const mdc::Line &line);

  void update(const std::vector<mdc::TraceLine> &traceLines);

  void update(const mdc::TraceLine &traceLine);

  void update(const std::vector<mdc::Pavement> &pavements);

  void update(const mdc::Pavement &pavement);

  void update(const std::vector<mdc::DrivingArrow> &drivingArrows);

  void update(const mdc::DrivingArrow &drivingArrow);

  void update(const std::vector<mdc::TrafficLights> &trafficLights);

  void update(const mdc::TrafficLights &trafficLights);

  void update(const std::vector<mdc::RoadLines> &roadLines);

  void update(const mdc::RoadLines &roadLine);

  void remove(const std::vector<mdc::Point> &points);

  void remove(const std::vector<mdc::Line> &lines);

  void remove(const mdc::Point &point);

  void remove(const mdc::Line &line);

  void remove(const mdc::TraceLine &traceLine);

  void remove(const mdc::Pavement &pavement);

  void remove(const mdc::DrivingArrow &drivingArrow);

  void remove(const mdc::TrafficLights &trafficLights);

  void remove(const mdc::RoadLines &roadLine);

  void clear();

  mdc::Point findByID(const mdc::Key<mdc::Point> &key) const;

  mdc::Line findByID(const mdc::Key<mdc::Line> &key) const;

  mdc::TraceLine findByID(const mdc::Key<mdc::TraceLine> &key) const;

  mdc::Pavement findByID(const mdc::Key<mdc::Pavement> &key) const;

  mdc::DrivingArrow findByID(const mdc::Key<mdc::DrivingArrow> &key) const;

  mdc::TrafficLights findByID(const mdc::Key<mdc::TrafficLights> &key) const;

  mdc::RoadLines findByID(const mdc::Key<mdc::RoadLines> &key) const;

  std::vector<mdc::Point> findByFilter(const mdc::Filter<mdc::Point> &filter) const;

  std::vector<mdc::Line> findByFilter(const mdc::Filter<mdc::Line> &filter) const;

  std::vector<mdc::TraceLine> findByFilter(const mdc::Filter<mdc::TraceLine> &filter) const;

  std::vector<mdc::Pavement> findByFilter(const mdc::Filter<mdc::Pavement> &filter) const;

  std::vector<mdc::DrivingArrow> findByFilter(const mdc::Filter<mdc::DrivingArrow> &filter) const;

  std::vector<mdc::TrafficLights> findByFilter(const mdc::Filter<mdc::TrafficLights> &filter) const;

  std::vector<mdc::RoadLines> findByFilter(const mdc::Filter<mdc::RoadLines> &filter) const;

  void saveToDir(const std::string &dirPath) const;

  void printAllPoints() const;

  void printAllLines() const;

  void printAllTraceLines() const;

  void printAllPavements() const;

  void printAllDrivingArrows() const;

  void printAllTrafficLights() const;

  void printAllRoadLines() const;

 public:
  VectorMapSingleton(const VectorMapSingleton &) = delete;

  VectorMapSingleton &operator=(const VectorMapSingleton &) = delete;

  VectorMapSingleton(VectorMapSingleton &&) noexcept = delete;

  VectorMapSingleton &operator=(VectorMapSingleton &&) noexcept = delete;

 private:
  VectorMapSingleton() = default;

  ~VectorMapSingleton() = default;

 private:
  static VectorMapSingleton *instance;

  mdc::Container<mdc::Point> points;
  mdc::Container<mdc::Line> lines;

  mdc::Container<mdc::TraceLine> traceLines;

  mdc::Container<mdc::Pavement> pavements;
  mdc::Container<mdc::DrivingArrow> drivingArrows;

  mdc::Container<mdc::TrafficLights> trafficLights;
  mdc::Container<mdc::RoadLines> roadLines;

};

std::ostream &operator<<(std::ostream &os, const mdc::Point &obj);

std::ostream &operator<<(std::ostream &os, const mdc::Line &obj);

std::ostream &operator<<(std::ostream &os, const mdc::TraceLine &obj);

std::ostream &operator<<(std::ostream &os, const mdc::Pavement &obj);

std::ostream &operator<<(std::ostream &os, const mdc::DrivingArrow &obj);

std::ostream &operator<<(std::ostream &os, const mdc::TrafficLights &obj);

std::ostream &operator<<(std::ostream &os, const mdc::RoadLines &obj);

std::istream &operator>>(std::istream &is, mdc::Point &obj);

std::istream &operator>>(std::istream &is, mdc::Line &obj);

std::istream &operator>>(std::istream &is, mdc::TraceLine &obj);

std::istream &operator>>(std::istream &is, mdc::Pavement &obj);

std::istream &operator>>(std::istream &is, mdc::DrivingArrow &obj);

std::istream &operator>>(std::istream &is, mdc::TrafficLights &obj);

std::istream &operator>>(std::istream &is, mdc::RoadLines &obj);

#endif //POINTCLOUDAPPLICATION_VECTORMAPSINGLETON_H
