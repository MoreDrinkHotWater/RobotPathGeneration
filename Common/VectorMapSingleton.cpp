//
// Created by zhihui on 3/27/19.
//

#include <iostream>
#include <iomanip>

#include "VectorMapSingleton.h"

VectorMapSingleton *VectorMapSingleton::instance = new VectorMapSingleton();

VectorMapSingleton *VectorMapSingleton::getInstance() {
    return instance;
}

size_t VectorMapSingleton::getMaxPointIndex() const {
    return points.findMaxIndex();
}

size_t VectorMapSingleton::getMaxLineIndex() const {
    return lines.findMaxIndex();
}

size_t VectorMapSingleton::getMaxTraceLinesIndex() const {
    return traceLines.findMaxIndex();
}

size_t VectorMapSingleton::getMaxPavementIndex() const {
    return pavements.findMaxIndex();
}

size_t VectorMapSingleton::getMaxDrivingArrowIndex() const {
    return drivingArrows.findMaxIndex();
}

size_t VectorMapSingleton::getMaxTrafficLightsIndex() const {
    return trafficLights.findMaxIndex();
}

size_t VectorMapSingleton::getMaxRoadLinesIndex() const {
    return roadLines.findMaxIndex();
}

void VectorMapSingleton::insert(const std::vector<mdc::Point> &points) {
    for (const auto &point : points) {
        if (point.pID == 0) {
            continue;
        }
        this->points.insert(mdc::Key<mdc::Point>(point.pID), point);
    }
}

void VectorMapSingleton::insert(const std::vector<mdc::Line> &lines) {
    for (const auto &line : lines) {
        if (line.lID == 0) {
            continue;
        }
        this->lines.insert(mdc::Key<mdc::Line>(line.lID), line);
    }
}

void VectorMapSingleton::update(const std::vector<mdc::Point> &points) {
    for (const auto &point : points) {
        if (point.pID == 0) {
            continue;
        }
        this->points.update(mdc::Key<mdc::Point>(point.pID), point);
    }
}

void VectorMapSingleton::update(const mdc::Point &point) {
    points.update(mdc::Key<mdc::Point>(point.pID), point);
}

void VectorMapSingleton::update(const std::vector<mdc::Line> &lines) {
    for (const auto &line : lines) {
        if (line.lID == 0) {
            continue;
        }
        this->lines.update(mdc::Key<mdc::Line>(line.lID), line);
    }
}

void VectorMapSingleton::update(const mdc::Line &line) {
    lines.update(mdc::Key<mdc::Line>(line.lID), line);
}

void VectorMapSingleton::update(const mdc::TraceLine &traceLine) {
    traceLines.update(mdc::Key<mdc::TraceLine>(traceLine.tlID), traceLine);
}

void VectorMapSingleton::update(const std::vector<mdc::TraceLine> &traceLines) {
    for (const auto &traceLine : traceLines) {
        if (traceLine.tlID == 0) {
            continue;
        }
        this->traceLines.update(mdc::Key<mdc::TraceLine>(traceLine.tlID), traceLine);
    }
}

void VectorMapSingleton::update(const std::vector<mdc::Pavement> &pavements) {
    for (const auto &pavement : pavements) {
        if (pavement.pID == 0) {
            continue;
        }
        this->pavements.update(mdc::Key<mdc::Pavement>(pavement.pID), pavement);
    }
}

void VectorMapSingleton::update(const mdc::Pavement &pavement) {
    pavements.update(mdc::Key<mdc::Pavement>(pavement.pID), pavement);
}

void VectorMapSingleton::update(const std::vector<mdc::DrivingArrow> &drivingArrows) {
    for (const auto &drivingArrow : drivingArrows) {
        if (drivingArrow.daID == 0) {
            continue;
        }
        this->drivingArrows.update(mdc::Key<mdc::DrivingArrow>(drivingArrow.daID), drivingArrow);
    }
}

void VectorMapSingleton::update(const mdc::DrivingArrow &drivingArrow) {
    drivingArrows.update(mdc::Key<mdc::DrivingArrow>(drivingArrow.daID), drivingArrow);
}

void VectorMapSingleton::update(const std::vector<mdc::TrafficLights> &trafficLights) {
    for (const auto &trafficLight : trafficLights) {
        if (trafficLight.tlID == 0) {
            continue;
        }
        this->trafficLights.update(mdc::Key<mdc::TrafficLights>(trafficLight.tlID), trafficLight);
    }
}

void VectorMapSingleton::update(const mdc::TrafficLights &trafficLights) {
    this->trafficLights.update(mdc::Key<mdc::TrafficLights>(trafficLights.tlID), trafficLights);
}

void VectorMapSingleton::update(const std::vector<mdc::RoadLines> &roadLines) {
    for (const auto &roadLine : roadLines) {
        if (roadLine.rlID == 0) {
            continue;
        }
        this->roadLines.update(mdc::Key<mdc::RoadLines>(roadLine.rlID), roadLine);
    }
}

void VectorMapSingleton::update(const mdc::RoadLines &roadLine) {
    this->roadLines.update(mdc::Key<mdc::RoadLines>(roadLine.rlID), roadLine);
}

void VectorMapSingleton::remove(const std::vector<mdc::Point> &points) {
    for (const auto &point : points) {
        if (point.pID == 0) {
            continue;
        }
        this->points.remove(mdc::Key<mdc::Point>(point.pID));
    }
}

void VectorMapSingleton::remove(const std::vector<mdc::Line> &lines) {
    for (const auto &line : lines) {
        if (line.lID == 0) {
            continue;
        }
        this->lines.remove(mdc::Key<mdc::Line>(line.lID));
    }
}

void VectorMapSingleton::remove(const mdc::Point &point) {
    if (point.pID != 0) {
        points.remove(mdc::Key<mdc::Point>(point.pID));
    }
}

void VectorMapSingleton::remove(const mdc::Line &line) {
    if (line.lID != 0) {
        lines.remove(mdc::Key<mdc::Line>(line.lID));
    }
}

void VectorMapSingleton::remove(const mdc::TraceLine &traceLine) {
    if (traceLine.tlID != 0) {
        traceLines.remove(mdc::Key<mdc::TraceLine>(traceLine.tlID));
    }
}

void VectorMapSingleton::remove(const mdc::Pavement &pavement) {
    if (pavement.pID != 0) {
        pavements.remove(mdc::Key<mdc::Pavement>(pavement.pID));
    }
}

void VectorMapSingleton::remove(const mdc::DrivingArrow &drivingArrow) {
    if (drivingArrow.daID != 0) {
        drivingArrows.remove(mdc::Key<mdc::DrivingArrow>(drivingArrow.daID));
    }
}

void VectorMapSingleton::remove(const mdc::TrafficLights &trafficLights) {
    if (trafficLights.tlID != 0) {
        this->trafficLights.remove(mdc::Key<mdc::TrafficLights>(trafficLights.tlID));
    }
}

void VectorMapSingleton::remove(const mdc::RoadLines &roadLine) {
    if (roadLine.rlID != 0) {
        roadLines.remove(mdc::Key<mdc::RoadLines>(roadLine.rlID));
    }
}

void VectorMapSingleton::clear() {
    points.clear();
    lines.clear();

    traceLines.clear();

    pavements.clear();
    drivingArrows.clear();

    trafficLights.clear();
    roadLines.clear();
}

mdc::Point VectorMapSingleton::findByID(const mdc::Key<mdc::Point> &key) const {
    return points.findByKey(key);
}

mdc::Line VectorMapSingleton::findByID(const mdc::Key<mdc::Line> &key) const {
    return lines.findByKey(key);
}

mdc::TraceLine VectorMapSingleton::findByID(const mdc::Key<mdc::TraceLine> &key) const {
    return traceLines.findByKey(key);
}

mdc::Pavement VectorMapSingleton::findByID(const mdc::Key<mdc::Pavement> &key) const {
    return pavements.findByKey(key);
}

mdc::DrivingArrow VectorMapSingleton::findByID(const mdc::Key<mdc::DrivingArrow> &key) const {
    return drivingArrows.findByKey(key);
}

mdc::TrafficLights VectorMapSingleton::findByID(const mdc::Key<mdc::TrafficLights> &key) const {
    return trafficLights.findByKey(key);
}

mdc::RoadLines VectorMapSingleton::findByID(const mdc::Key<mdc::RoadLines> &key) const {
    return roadLines.findByKey(key);
}

std::vector<mdc::Point> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::Point> &filter) const {
    return points.findByFilter(filter);
}

std::vector<mdc::Line> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::Line> &filter) const {
    return lines.findByFilter(filter);
}

std::vector<mdc::TraceLine> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::TraceLine> &filter) const {
    return traceLines.findByFilter(filter);
}

std::vector<mdc::Pavement> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::Pavement> &filter) const {
    return pavements.findByFilter(filter);
}

std::vector<mdc::DrivingArrow> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::DrivingArrow> &filter) const {
    return drivingArrows.findByFilter(filter);
}

std::vector<mdc::TrafficLights> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::TrafficLights> &filter) const {
    return trafficLights.findByFilter(filter);
}

std::vector<mdc::RoadLines> VectorMapSingleton::findByFilter(const mdc::Filter<mdc::RoadLines> &filter) const {
    return roadLines.findByFilter(filter);
}

void VectorMapSingleton::saveToDir(const std::string &dirPath) const {
    std::string header, filePath;

    header = "PID, B, L, H, BX, LY, REF, MCODE1, MCODE2, MCODE3, fromPointLineID, toPointLineID";
    filePath = dirPath + "/point.data";
    points.output(filePath, header);

    header = "LID, SPID, EPID, fromThisLineID, toThisLineID";
    filePath = dirPath + "/line.data";
    lines.output(filePath, header);

    header = "pID, upperLeftCorner, lowerLeftCorner, upperRightCorner, lowerRightCorner";
    filePath = dirPath + "/pavement.data";
    pavements.output(filePath, header);

    header = "daID, centerPoint, type, typeIndex, rotation";
    filePath = dirPath + "/drivingArrow.data";
    drivingArrows.output(filePath, header);

    header = "tlID, lightLocationPoint, rotation";
    filePath = dirPath + "/trafficLights.data";
    trafficLights.output(filePath, header);

    header = "rlID, type, allPointsID";
    filePath = dirPath + "/roadLines.data";
    roadLines.output(filePath, header);

    header = "TLID, SPID, EPID, fromThisLineID, toThisLineID";
    filePath = dirPath + "/traceLine.data";
    traceLines.output(filePath, header);
}

void VectorMapSingleton::printAllPoints() const {
    points.printAll();
}

void VectorMapSingleton::printAllLines() const {
    lines.printAll();
}

void VectorMapSingleton::printAllTraceLines() const {
    traceLines.printAll();
}

void VectorMapSingleton::printAllPavements() const {
    pavements.printAll();
}

void VectorMapSingleton::printAllDrivingArrows() const {
    drivingArrows.printAll();
}

void VectorMapSingleton::printAllTrafficLights() const {
    trafficLights.printAll();
}

void VectorMapSingleton::printAllRoadLines() const {
    roadLines.printAll();
}

std::ostream &operator<<(std::ostream &os, const mdc::Point &obj) {
    os << obj.pID << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.b << "," << obj.l << ",";
    os.unsetf(std::ios::fixed);

    os << obj.h << "," << obj.bx << "," << obj.ly << "," << obj.ref << "," << obj.mcode1 << "," << obj.mcode2 << ","
       << obj.mcode3 << ",";

    os << "*,";

    for (auto const &id : obj.fromPointLineID) {
        os << id << ",";
    }

    os << "*,";

    for (const auto &id : obj.toPointLineID) {
        os << id << ",";
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::Line &obj) {
    os << obj.lID << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.sPID << "," << obj.ePID << ",";
    os.unsetf(std::ios::fixed);

    os << "*,";
    for (auto const &id : obj.fromThisLineID) {
        os << id << ",";
    }

    os << "*,";
    for (const auto &id : obj.toThisLineID) {
        os << id << ",";
    }

    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::TraceLine &obj) {
    os << obj.tlID << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.sPID << "," << obj.ePID << ",";
    os.unsetf(std::ios::fixed);

    os << "*,";
    for (auto const &id : obj.fromThisLineID) {
        os << id << ",";
    }

    os << "*,";
    for (const auto &id : obj.toThisLineID) {
        os << id << ",";
    }

    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::Pavement &obj) {
    os << obj.pID << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.upperLeftCorner << "," << obj.lowerLeftCorner << ","
       << obj.upperRightCorner << "," << obj.lowerRightCorner << ",";
    os.unsetf(std::ios::fixed);

    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::DrivingArrow &obj) {
    os << obj.daID << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.centerPoint << "," << obj.type << "," << obj.typeIndex << "," << obj.rotation
       << ",";
    os.unsetf(std::ios::fixed);

    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::TrafficLights &obj) {
    os << obj.tlID << ",";

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.lightLocationPoint << "," << obj.rotation << ",";
    os.unsetf(std::ios::fixed);

    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::RoadLines &obj) {
    os << obj.rlID << ",";

    os << obj.type << ",";

    for (auto const &id : obj.allPointsID) {
        os << id << ",";
    }

    return os;
}

std::istream &operator>>(std::istream &is, mdc::Point &obj) {
    std::vector<std::string> columns;
    std::string column;
    size_t num;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    num = columns.size();

    obj.pID = std::stoi(columns[0]);
    obj.b = std::stod(columns[1]);
    obj.l = std::stod(columns[2]);
    obj.h = std::stod(columns[3]);
    obj.bx = std::stod(columns[4]);
    obj.ly = std::stod(columns[5]);
    obj.ref = std::stoi(columns[6]);
    obj.mcode1 = std::stoi(columns[7]);
    obj.mcode2 = std::stoi(columns[8]);
    obj.mcode3 = std::stoi(columns[9]);

    if (num > 12) {
        if (columns[11] == "*") {
            for (size_t i = 12; i < num; ++i) {
                obj.toPointLineID.push_back(std::stoi(columns[i]));
            }
        } else {
            size_t i;
            for (i = 11; i < num; ++i) {
                if (columns[i] == "*") {
                    break;
                }
                obj.fromPointLineID.push_back(std::stoi(columns[i]));
            }
            if (i != num - 1) {
                for (size_t j = i + 1; j < num; ++j) {
                    if (columns[j] == "*") {
                        break;
                    }
                    obj.toPointLineID.push_back(std::stoi(columns[j]));
                }
            }
        }
    }

    return is;
}

std::istream &operator>>(std::istream &is, mdc::Line &obj) {
    std::vector<std::string> columns;
    std::string column;
    size_t num;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    num = columns.size();

    obj.lID = std::stoi(columns[0]);
    obj.sPID = std::stoi(columns[1]);
    obj.ePID = std::stoi(columns[2]);

    if (num > 5) {
        if (columns[4] == "*") {
            for (size_t i = 5; i < num; ++i) {
                obj.toThisLineID.push_back(std::stoi(columns[i]));
            }
        } else {
            size_t i;
            for (i = 4; i < num; ++i) {
                if (columns[i] == "*") {
                    break;
                }
                obj.fromThisLineID.push_back(std::stoi(columns[i]));
            }
            if (i != num - 1) {
                for (size_t j = i + 1; j < num; ++j) {
                    if (columns[j] == "*") {
                        break;
                    }
                    obj.toThisLineID.push_back(std::stoi(columns[j]));
                }
            }
        }
    }

    return is;
}

std::istream &operator>>(std::istream &is, mdc::TraceLine &obj) {
    std::vector<std::string> columns;
    std::string column;
    size_t num;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    num = columns.size();

    obj.tlID = std::stoi(columns[0]);
    obj.sPID = std::stoi(columns[1]);
    obj.ePID = std::stoi(columns[2]);

    if (num > 5) {
        if (columns[4] == "*") {
            for (size_t i = 5; i < num; ++i) {
                obj.toThisLineID.push_back(std::stoi(columns[i]));
            }
        } else {
            size_t i;
            for (i = 4; i < num; ++i) {
                if (columns[i] == "*") {
                    break;
                }
                obj.fromThisLineID.push_back(std::stoi(columns[i]));
            }
            if (i != num - 1) {
                for (size_t j = i + 1; j < num; ++j) {
                    if (columns[j] == "*") {
                        break;
                    }
                    obj.toThisLineID.push_back(std::stoi(columns[j]));
                }
            }
        }
    }

    return is;
}

std::istream &operator>>(std::istream &is, mdc::Pavement &obj) {
    std::vector<std::string> columns;
    std::string column;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    obj.pID = std::stoi(columns[0]);

    obj.upperLeftCorner = std::stoi(columns[1]);
    obj.lowerLeftCorner = std::stoi(columns[2]);
    obj.upperRightCorner = std::stoi(columns[3]);
    obj.lowerRightCorner = std::stoi(columns[4]);

    return is;
}

std::istream &operator>>(std::istream &is, mdc::DrivingArrow &obj) {
    std::vector<std::string> columns;
    std::string column;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    obj.daID = std::stoi(columns[0]);

    obj.centerPoint = std::stoi(columns[1]);
    obj.type = columns[2];
    obj.typeIndex = std::stoi(columns[3]);
    obj.rotation = std::stod(columns[4]);

    return is;
}

std::istream &operator>>(std::istream &is, mdc::TrafficLights &obj) {
    std::vector<std::string> columns;
    std::string column;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    obj.tlID = std::stoi(columns[0]);

    obj.lightLocationPoint = std::stoi(columns[1]);
    obj.rotation = std::stod(columns[2]);

    return is;
}

std::istream &operator>>(std::istream &is, mdc::RoadLines &obj) {
    std::vector<std::string> columns;
    std::string column;
    size_t num;

    while (std::getline(is, column, ',')) {
        columns.push_back(column);
    }

    num = columns.size();

    obj.rlID = std::stoi(columns[0]);
    obj.type = columns[1];

    for (int i = 2; i < num; ++i) {
        obj.allPointsID.push_back(std::stoi(columns[i]));
    }

    return is;
}

