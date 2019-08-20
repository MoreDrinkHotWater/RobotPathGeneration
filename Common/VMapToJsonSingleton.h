//
// Created by zhihui on 4/18/19.
//

#ifndef HDMAPS_VMAPTOJSONSINGLETON_H
#define HDMAPS_VMAPTOJSONSINGLETON_H

#include <QJsonArray>

#include "DataStructure.h"

class VMapToJsonSingleton {

public:
    static VMapToJsonSingleton *getInstance();

    void transAllPointsToJson(const std::string &dirPath) const;

    void transAllLinesToJson(const std::string &dirPath) const;

    void transAllPavementsToJson(const std::string &dirPath) const;

    void transAllDrivingArrowToJson(const std::string &dirPath) const;

    void transAllTrafficLightsToJson(const std::string &dirPath) const;

    void transAllRoadLinesToJson(const std::string &dirPath) const;

public:
    VMapToJsonSingleton(const VMapToJsonSingleton &) = delete;

    VMapToJsonSingleton &operator=(const VMapToJsonSingleton &) = delete;

    VMapToJsonSingleton(VMapToJsonSingleton &&) noexcept = delete;

    VMapToJsonSingleton &operator=(VMapToJsonSingleton &&) noexcept = delete;

private:

    QJsonArray pointToArray(const mdc::Point &point) const;

    QJsonArray pointsToArray(const std::vector<mdc::Point> &points) const;

    QJsonArray lineToArray(const mdc::Point &point1, const mdc::Point &point2) const;

    QJsonArray pavementToArray(const mdc::Pavement &pavement) const;

    QJsonArray drivingArrowToArray(const mdc::DrivingArrow &drivingArrow) const;

    QJsonArray trafficLightsToArray(const mdc::TrafficLights &trafficLight) const;

    QJsonArray roadLinesToArray(const mdc::RoadLines &roadLines) const;

    QJsonObject getJsonObject(const QString &type, size_t ID, const QJsonArray &pos) const;

    QJsonObject getDrivingArrowJsonObject(const QString &type, size_t ID, const QJsonArray &pos, int typeIndex,
                                          double rotation) const;

    QJsonObject getTrafficLightsJsonObject(size_t ID, const QJsonArray &pos, double rotation) const;

    QJsonObject getRoadLinesJsonObject(const QString &type, size_t ID, const QJsonArray &pos) const;

    void outputJsonObject(const std::string &dirPath, const QString &fileName, const QJsonArray &array) const;

private:
    VMapToJsonSingleton() = default;

    ~VMapToJsonSingleton() = default;

private:
    static VMapToJsonSingleton *instance;

};


#endif //HDMAPS_VMAPTOJSONSINGLETON_H
