//
// Created by zhihui on 4/29/19.
//

#include <iostream>
#include <iomanip>

#include <osg/Vec3d>

#include "VectorMapSingleton.h"
#include "VMapToCSVSingleton.h"

VMapToCSVSingleton *VMapToCSVSingleton::instance = new VMapToCSVSingleton();

VMapToCSVSingleton *VMapToCSVSingleton::getInstance() {
    return instance;
}

void VMapToCSVSingleton::transAllTraceLinesToCSV(const std::string &dirPath) const {
    std::vector<mdc::TraceLine> traceLines;
    // 机器人行驶路线（路径组成<->点的个数）
    std::vector<std::pair<std::vector<mdc::TraceLine>, int>> robotTraceLinePointNumPairs;
    // 路径及路径上的点
    std::vector<std::pair<int, std::vector<mdc::CSVPoint>>> robotTracePoints;
    traceLines = VectorMapSingleton::getInstance()->findByFilter([](const mdc::TraceLine &traceLine) { return true; });
    std::vector<mdc::TraceLine> tempVector;
    for (const auto &traceLine : traceLines) {
        tempVector.push_back(traceLine);
        if (traceLine.fromThisLineID.empty()) {
            robotTraceLinePointNumPairs.emplace_back(std::make_pair(tempVector, 0));
            tempVector.clear();
        }
    }
    // 所有的线段条数
    int numRobotTraceLine = robotTraceLinePointNumPairs.size();
    for (int i = 0; i < numRobotTraceLine; ++i) {
        generateHighPrecisionTrace(robotTraceLinePointNumPairs[i], robotTracePoints, i);
    }

    saveTraceLineToDir(dirPath, robotTraceLinePointNumPairs, robotTracePoints);
}

void VMapToCSVSingleton::generateHighPrecisionTrace(std::pair<std::vector<mdc::TraceLine>,
                                                              int> &robotTraceLinePointNumPair,
                                                    std::vector<std::pair<int,
                                                                          std::vector<mdc::CSVPoint>>> &robotTracePoints,
                                                    int k) const {
    int pID = 0;

    std::vector<mdc::CSVPoint> temp;
    std::vector<mdc::CSVPoint> allHPPoints;

    for (int i = 0; i < robotTraceLinePointNumPair.first.size(); ++i) {
        std::vector<osg::Vec3d> points;

        mdc::Point sPoint;
        mdc::Point ePoint;

        osg::Vec3d startPoint;
        osg::Vec3d endPoint;

        osg::Vec3d direction;

        sPoint =
                VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(robotTraceLinePointNumPair.first[i].sPID));
        ePoint =
                VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(robotTraceLinePointNumPair.first[i].ePID));

        startPoint = osg::Vec3d(sPoint.ly, sPoint.bx, sPoint.h);
        endPoint = osg::Vec3d(ePoint.ly, ePoint.bx, ePoint.h);

        direction = osg::Vec3d(endPoint - startPoint);

        double distance = direction.length();
        double precision = 1;

        direction.normalize();
        points.push_back(startPoint);
        if (distance > precision) {
            for (int i = 1; i < distance / precision; ++i) {
                osg::Vec3d point = startPoint + direction / precision * i;
                points.push_back(point);
            }
        }
        points.push_back(endPoint);

        int laneType;

        for (int j = 0; j < points.size(); ++j) {
            osg::Vec3d point = points[j];
            auto iter = std::find_if(temp.begin(),
                                     temp.end(),
                                     [&point](const mdc::CSVPoint &CSVpoint) {
                                       return point.x() == CSVpoint.x && point.y() == CSVpoint.y
                                               && point.z() == CSVpoint.z;
                                     });

            if (i == 0 && j == 0) {
                laneType = 1;
            } else if (i == robotTraceLinePointNumPair.first.size() - 1 && j == points.size() - 1) {
                laneType = 2;
            } else {
                laneType = 0;
            }

            if (iter == temp.end()) {
                allHPPoints.emplace_back(pID++, point.x(), point.y(), point.z(), laneType);
                temp.emplace_back(pID, point.x(), point.y(), point.z(), laneType);
            }

        }

    }
    robotTracePoints.emplace_back(std::make_pair(k, allHPPoints));
    robotTraceLinePointNumPair.second = allHPPoints.size();
}

//void VMapToCSVSingleton::transAllTraceLinesToCSV(const std::string &dirPath) const {
//    std::vector<mdc::TraceLine> traceLines;
//
//    std::vector<mdc::TraceLineHP> traceLineHPs;
//    std::vector<mdc::CSVPoint> CSVPoints;
//
//    // 原线段中所有小线段的ID
//    std::vector<std::pair<std::vector<size_t>, mdc::TraceLine>> HPLineIDs;
//    // 原线段中所有新点的ID
//    std::vector<std::pair<std::vector<size_t>, mdc::TraceLine>> HPPointIDs;
//
//    // 从1开始计数，0表示无效
//    size_t pID, tlID;
//
//    VectorMapSingleton::getInstance()->printAllTraceLines();
//    pID = tlID = 1;
//    traceLines = VectorMapSingleton::getInstance()->findByFilter([](const mdc::TraceLine &traceLines) { return true; });
//    for (const auto &traceLine : traceLines) {
//        generateHighPrecisionTrace(traceLine, pID, tlID, traceLineHPs, CSVPoints, HPLineIDs, HPPointIDs);
//    }
//
//    for (const auto &traceLine : traceLines) {
//        updateTraceLineHP(traceLine, traceLineHPs, HPLineIDs);
//    }
//
//    saveTraceLineToDir(dirPath, traceLineHPs, CSVPoints);
//}

//void VMapToCSVSingleton::generateHighPrecisionTrace(const mdc::TraceLine &traceLine,
//                                                    size_t &pID,
//                                                    size_t &tlID,
//                                                    std::vector<mdc::TraceLineHP> &traceLineHPs,
//                                                    std::vector<mdc::CSVPoint> &CSVPoints,
//                                                    std::vector<std::pair<std::vector<size_t>,
//                                                                          mdc::TraceLine>> &HPLineIDs,
//                                                    std::vector<std::pair<std::vector<size_t>,
//                                                                          mdc::TraceLine>> &HPPointIDs) const {
//    std::vector<size_t> allHPPointIDs;
//    std::vector<size_t> allHPLineIDs;
//
//    std::vector<osg::Vec3d> points;
//
//    mdc::Point sPoint;
//    mdc::Point ePoint;
//
//    osg::Vec3d startPoint;
//    osg::Vec3d endPoint;
//
//    osg::Vec3d direction;
//
//    sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.sPID));
//    ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.ePID));
//
//    startPoint = osg::Vec3d(sPoint.ly, sPoint.bx, sPoint.h);
//    endPoint = osg::Vec3d(ePoint.ly, ePoint.bx, ePoint.h);
//
//    direction = osg::Vec3d(endPoint - startPoint);
//
//    double distance = direction.length();
//    double precision = 1;
//
//    direction.normalize();
//    points.push_back(startPoint);
//    if (distance > precision) {
//        for (int i = 1; i < distance / precision; ++i) {
//            osg::Vec3d point = startPoint + direction / precision * i;
//            points.push_back(point);
//        }
//    }
//    points.push_back(endPoint);
//
//    for (int i = 0; i < points.size(); ++i) {
//        osg::Vec3d point = points[i];
//        auto iter = std::find_if(CSVPoints.begin(),
//                                 CSVPoints.end(),
//                                 [&point](const mdc::CSVPoint &CSVpoint) {
//                                   return point.x() == CSVpoint.x && point.y() == CSVpoint.y && point.z() == CSVpoint.z;
//                                 });
//        if (i == points.size() - 1) {
//            if (iter == CSVPoints.end()) {
//                CSVPoints.emplace_back(pID, point.x(), point.y(), point.z());
//                allHPPointIDs.push_back(pID);
//                pID++;
//            } else {
//                allHPPointIDs.push_back(iter->pID);
//            }
//        } else {
//            if (iter == CSVPoints.end()) {
//                CSVPoints.emplace_back(pID, point.x(), point.y(), point.z());
//                allHPPointIDs.push_back(pID);
//                traceLineHPs.emplace_back(tlID, pID, pID + 1);
//                allHPLineIDs.push_back(tlID);
//                tlID++;
//                pID++;
//            } else {
//                allHPPointIDs.push_back(iter->pID);
//                traceLineHPs.emplace_back(tlID, iter->pID, iter->pID + 1);
//                allHPLineIDs.push_back(tlID);
//                tlID++;
//            }
//        }
//    }
//
//    HPPointIDs.emplace_back(std::make_pair(allHPPointIDs, traceLine));
//    HPLineIDs.emplace_back(std::make_pair(allHPLineIDs, traceLine));
//}

void VMapToCSVSingleton::updateTraceLineHP(const mdc::TraceLine &traceLine,
                                           std::vector<mdc::TraceLineHP> &traceLineHPs,
                                           const std::vector<std::pair<std::vector<size_t>,
                                                                       mdc::TraceLine>> &HPLineIDs) const {
    std::vector<size_t> allHPLineIDs;
    auto iter = std::find_if(HPLineIDs.begin(),
                             HPLineIDs.end(),
                             [&traceLine](const std::pair<std::vector<size_t>, mdc::TraceLine> &pair) {
                               return pair.second.tlID == traceLine.tlID;
                             });
    allHPLineIDs = iter->first;

    if (allHPLineIDs.size() == 1) {
        size_t line = allHPLineIDs[0];
        auto iter = std::find_if(traceLineHPs.begin(),
                                 traceLineHPs.end(),
                                 [line](const mdc::TraceLineHP &traceLineHP) { return traceLineHP.tlID == line; });

        mdc::Point sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.sPID));
        std::vector<size_t> allSHPLineIDs;
        if (!sPoint.toPointLineID.empty()) {
            for (const auto &ID : sPoint.toPointLineID) {
                auto iterHPLine = std::find_if(HPLineIDs.begin(),
                                               HPLineIDs.end(),
                                               [&ID](const std::pair<std::vector<size_t>,
                                                                     mdc::TraceLine> &pair) {
                                                 return pair.second.tlID == ID;
                                               });
                allSHPLineIDs.push_back(iterHPLine->first[iterHPLine->first.size() - 1]);
            }
            if (allSHPLineIDs.size() >= 4) {
                iter->sTLID1 = allSHPLineIDs[0];
                iter->sTLID2 = allSHPLineIDs[1];
                iter->sTLID3 = allSHPLineIDs[2];
                iter->sTLID4 = allSHPLineIDs[3];
            }
            if (allSHPLineIDs.size() == 3) {
                iter->sTLID1 = allSHPLineIDs[0];
                iter->sTLID2 = allSHPLineIDs[1];
                iter->sTLID3 = allSHPLineIDs[2];
            }
            if (allSHPLineIDs.size() == 2) {
                iter->sTLID1 = allSHPLineIDs[0];
                iter->sTLID2 = allSHPLineIDs[1];
            }
            if (allSHPLineIDs.size() == 1) {
                iter->sTLID1 = allSHPLineIDs[0];
            }
        }

        mdc::Point ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.ePID));
        std::vector<size_t> allEHPLineIDs;
        if (!ePoint.fromPointLineID.empty()) {
            for (const auto &ID : ePoint.fromPointLineID) {
                auto iterHPLine = std::find_if(HPLineIDs.begin(),
                                               HPLineIDs.end(),
                                               [&ID](const std::pair<std::vector<size_t>,
                                                                     mdc::TraceLine> &pair) {
                                                 return pair.second.tlID == ID;
                                               });
                allEHPLineIDs.push_back(iterHPLine->first[0]);
            }
            if (allEHPLineIDs.size() >= 4) {
                iter->eTLID1 = allEHPLineIDs[0];
                iter->eTLID2 = allEHPLineIDs[1];
                iter->eTLID3 = allEHPLineIDs[2];
                iter->eTLID4 = allEHPLineIDs[3];
            }
            if (allEHPLineIDs.size() == 3) {
                iter->eTLID1 = allEHPLineIDs[0];
                iter->eTLID2 = allEHPLineIDs[1];
                iter->eTLID3 = allEHPLineIDs[2];
            }
            if (allEHPLineIDs.size() == 2) {
                iter->eTLID1 = allEHPLineIDs[0];
                iter->eTLID2 = allEHPLineIDs[1];
            }
            if (allEHPLineIDs.size() == 1) {
                iter->eTLID1 = allEHPLineIDs[0];
            }
        }
    } else {
        for (int i = 0; i < allHPLineIDs.size(); ++i) {
            size_t line = allHPLineIDs[i];
            auto iter = std::find_if(traceLineHPs.begin(),
                                     traceLineHPs.end(),
                                     [line](const mdc::TraceLineHP &traceLineHP) { return traceLineHP.tlID == line; });
            if (i == 0) {
                mdc::Point sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.sPID));
                std::vector<size_t> allSHPLineIDs;
                if (!sPoint.toPointLineID.empty()) {
                    for (const auto &ID : sPoint.toPointLineID) {
                        auto iterHPLine = std::find_if(HPLineIDs.begin(),
                                                       HPLineIDs.end(),
                                                       [&ID](const std::pair<std::vector<size_t>,
                                                                             mdc::TraceLine> &pair) {
                                                         return pair.second.tlID == ID;
                                                       });
                        allSHPLineIDs.push_back(iterHPLine->first[iterHPLine->first.size() - 1]);
                    }
                    if (allSHPLineIDs.size() >= 4) {
                        iter->sTLID1 = allSHPLineIDs[0];
                        iter->sTLID2 = allSHPLineIDs[1];
                        iter->sTLID3 = allSHPLineIDs[2];
                        iter->sTLID4 = allSHPLineIDs[3];
                    }
                    if (allSHPLineIDs.size() == 3) {
                        iter->sTLID1 = allSHPLineIDs[0];
                        iter->sTLID2 = allSHPLineIDs[1];
                        iter->sTLID3 = allSHPLineIDs[2];
                    }
                    if (allSHPLineIDs.size() == 2) {
                        iter->sTLID1 = allSHPLineIDs[0];
                        iter->sTLID2 = allSHPLineIDs[1];
                    }
                    if (allSHPLineIDs.size() == 1) {
                        iter->sTLID1 = allSHPLineIDs[0];
                    }
                }
                iter->eTLID1 = allHPLineIDs[i + 1];
            } else if (i == allHPLineIDs.size() - 1) {
                mdc::Point ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.ePID));
                iter->sTLID1 = allHPLineIDs[i - 1];
                std::vector<size_t> allEHPLineIDs;
                if (!ePoint.fromPointLineID.empty()) {
                    for (const auto &ID : ePoint.fromPointLineID) {
                        auto iterHPLine = std::find_if(HPLineIDs.begin(),
                                                       HPLineIDs.end(),
                                                       [&ID](const std::pair<std::vector<size_t>,
                                                                             mdc::TraceLine> &pair) {
                                                         return pair.second.tlID == ID;
                                                       });
                        allEHPLineIDs.push_back(iterHPLine->first[0]);
                    }
                    if (allEHPLineIDs.size() >= 4) {
                        iter->eTLID1 = allEHPLineIDs[0];
                        iter->eTLID2 = allEHPLineIDs[1];
                        iter->eTLID3 = allEHPLineIDs[2];
                        iter->eTLID4 = allEHPLineIDs[3];
                    }
                    if (allEHPLineIDs.size() == 3) {
                        iter->eTLID1 = allEHPLineIDs[0];
                        iter->eTLID2 = allEHPLineIDs[1];
                        iter->eTLID3 = allEHPLineIDs[2];
                    }
                    if (allEHPLineIDs.size() == 2) {
                        iter->eTLID1 = allEHPLineIDs[0];
                        iter->eTLID2 = allEHPLineIDs[1];
                    }
                    if (allEHPLineIDs.size() == 1) {
                        iter->eTLID1 = allEHPLineIDs[0];
                    }
                }
            } else {
                iter->sTLID1 = allHPLineIDs[i - 1];
                iter->eTLID1 = allHPLineIDs[i + 1];
            }
        }
    }
}

void VMapToCSVSingleton::saveTraceLineToDir(const std::string &dirPath,
                                            const std::vector<std::pair<std::vector<mdc::TraceLine>,
                                                                        int>> &robotTraceLinePointNumPairs,
                                            const std::vector<std::pair<int,
                                                                        std::vector<mdc::CSVPoint>>> &robotTracePoints) const {
    std::string filePath;

    filePath = dirPath + "/RobotMap.csv";
    std::ofstream ofsRobotTrace(filePath);

    ofsRobotTrace << robotTraceLinePointNumPairs.size() << std::endl;
    for (int i = 0; i < robotTracePoints.size(); ++i) {
        ofsRobotTrace << robotTraceLinePointNumPairs[i].second << std::endl;
        ofsRobotTrace << "0 " << i << " " << std::string("lane" + std::to_string(i + 1)) << std::endl;
        for (const auto &point:robotTracePoints[i].second) {
            ofsRobotTrace << point << std::endl;
        }
    }

    ofsRobotTrace.close();
}

//void VMapToCSVSingleton::saveTraceLineToDir(const std::string &dirPath,
//                                            const std::vector<mdc::TraceLineHP> &traceLineHPs,
//                                            const std::vector<mdc::CSVPoint> &CSVPoints) const {
//    std::string header, filePath;
//
//    header = "id, x, y, z";
//    filePath = dirPath + "/Point.csv";
//    std::ofstream ofsPoint(filePath);
//    ofsPoint << header << std::endl;
//    for (const auto &point : CSVPoints) {
//        ofsPoint << point << std::endl;
//    }
//
//    ofsPoint.close();
//
//    header = "id, fpid, bpid, flid1, flid2, flid3, flid4, blid1, blid2, blid3, blid4";
//    filePath = dirPath + "/Line.csv";
//    std::ofstream ofsLine(filePath);
//    ofsLine << header << std::endl;
//    for (const auto &traceLineHP : traceLineHPs) {
//        ofsLine << traceLineHP << std::endl;
//    }
//
//    ofsLine.close();
//}

std::ostream &operator<<(std::ostream &os, const mdc::TraceLineHP &obj) {
    os << obj.tlID << ",";

    os << obj.sPID << "," << obj.ePID << ",";

    os << obj.sTLID1 << "," << obj.sTLID2 << "," << obj.sTLID3 << "," << obj.sTLID4 << ",";
    os << obj.eTLID1 << "," << obj.eTLID2 << "," << obj.eTLID3 << "," << obj.eTLID4 << ",";

    return os;
}

std::ostream &operator<<(std::ostream &os, const mdc::CSVPoint &obj) {

    os.setf(std::ios::fixed);
    os << std::setprecision(9) << obj.x << " " << obj.y << " " << obj.z << " " << obj.pID << " " << obj.laneType << " " << 0;
    os.unsetf(std::ios::fixed);

    return os;
}