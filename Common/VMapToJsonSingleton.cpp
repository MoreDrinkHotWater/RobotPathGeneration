//
// Created by zhihui on 4/18/19.
//

#include <iostream>

#include <QJsonObject>
#include <QJsonDocument>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QtWidgets/QMessageBox>

#include "VMapToJsonSingleton.h"
#include "VectorMapSingleton.h"

VMapToJsonSingleton *VMapToJsonSingleton::instance = new VMapToJsonSingleton();

VMapToJsonSingleton *VMapToJsonSingleton::getInstance() {
    return instance;
}

void VMapToJsonSingleton::transAllPointsToJson(const std::string &dirPath) const {}

void VMapToJsonSingleton::transAllLinesToJson(const std::string &dirPath) const {}

void VMapToJsonSingleton::transAllPavementsToJson(const std::string &dirPath) const {
    QString fileName = "CrossWalk";
    QJsonArray jsonArray;

    std::vector<mdc::Pavement> pavements = VectorMapSingleton::getInstance()->findByFilter(
            [](const mdc::Pavement &pavement) { return true; });

    for (const auto &pavement : pavements) {
        size_t ID = pavement.pID;
        auto pos = pavementToArray(pavement);
        auto pavementObject = getJsonObject(fileName, ID, pos);
        jsonArray.append(QJsonValue(pavementObject));
    }

    outputJsonObject(dirPath, fileName, jsonArray);
}

void VMapToJsonSingleton::transAllDrivingArrowToJson(const std::string &dirPath) const {
    QString fileName = "TrafficSign";
    QJsonArray jsonArray;

    std::vector<mdc::DrivingArrow> drivingArrows = VectorMapSingleton::getInstance()->findByFilter(
            [](const mdc::DrivingArrow &drivingArrow) { return true; });

    for (const auto &drivingArrow : drivingArrows) {
        size_t ID = drivingArrow.daID;
        auto pos = drivingArrowToArray(drivingArrow);
        auto drivingArrowObject = getDrivingArrowJsonObject(QString::fromStdString(drivingArrow.type), ID, pos,
                                                            drivingArrow.typeIndex, drivingArrow.rotation);
        jsonArray.append(QJsonValue(drivingArrowObject));
    }

    outputJsonObject(dirPath, fileName, jsonArray);
}

void VMapToJsonSingleton::transAllTrafficLightsToJson(const std::string &dirPath) const {
    QString fileName = "Signal";
    QJsonArray jsonArray;

    std::vector<mdc::TrafficLights> trafficLights = VectorMapSingleton::getInstance()->findByFilter(
            [](const mdc::TrafficLights &drivingArrow) { return true; });

    for (const auto &trafficLight : trafficLights) {
        size_t ID = trafficLight.tlID;
        auto pos = trafficLightsToArray(trafficLight);
        auto trafficLightsObject = getTrafficLightsJsonObject(ID, pos, trafficLight.rotation);
        jsonArray.append(QJsonValue(trafficLightsObject));
    }

    outputJsonObject(dirPath, fileName, jsonArray);
}

void VMapToJsonSingleton::transAllRoadLinesToJson(const std::string &dirPath) const {
    QString fileName = "RoadEdge";
    QJsonArray jsonArray;

    std::vector<mdc::RoadLines> roadLines = VectorMapSingleton::getInstance()->findByFilter(
            [](const mdc::RoadLines &roadLine) { return true; });

    for (const auto &roadLine : roadLines) {
        size_t ID = roadLine.rlID;
        auto pos = roadLinesToArray(roadLine);
        auto roadLinesObject = getRoadLinesJsonObject(QString::fromStdString(roadLine.type), ID, pos);
        jsonArray.append(QJsonValue(roadLinesObject));
    }

    outputJsonObject(dirPath, fileName, jsonArray);
}

QJsonArray VMapToJsonSingleton::pointToArray(const mdc::Point &point) const {
    QJsonArray posArray;

    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    return posArray;
}

QJsonArray VMapToJsonSingleton::pointsToArray(const std::vector<mdc::Point> &points) const {
    QJsonArray posArray;
    for (const mdc::Point &point : points) {
        posArray.append(point.ly);
        posArray.append(point.bx);
        posArray.append(point.h);
    }

    return posArray;
}

QJsonArray VMapToJsonSingleton::lineToArray(const mdc::Point &point1, const mdc::Point &point2) const {
    QJsonArray posArray;

    posArray.append(point1.ly);
    posArray.append(point1.bx);
    posArray.append(point1.h);

    posArray.append(point2.ly);
    posArray.append(point2.bx);
    posArray.append(point2.h);

    return posArray;
}

QJsonArray VMapToJsonSingleton::pavementToArray(const mdc::Pavement &pavement) const {
    QJsonArray posArray;
    mdc::Point point;

    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(pavement.upperLeftCorner));
    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(pavement.lowerLeftCorner));
    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(pavement.upperRightCorner));
    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(pavement.lowerRightCorner));
    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    return posArray;
}

QJsonArray VMapToJsonSingleton::drivingArrowToArray(const mdc::DrivingArrow &drivingArrow) const {
    QJsonArray posArray;
    mdc::Point point;

    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(drivingArrow.centerPoint));
    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    return posArray;
}

QJsonArray VMapToJsonSingleton::trafficLightsToArray(const mdc::TrafficLights &trafficLight) const {
    QJsonArray posArray;
    mdc::Point point;

    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(trafficLight.lightLocationPoint));
    posArray.append(point.ly);
    posArray.append(point.bx);
    posArray.append(point.h);

    return posArray;
}

QJsonArray VMapToJsonSingleton::roadLinesToArray(const mdc::RoadLines &roadLines) const {
    QJsonArray posArray;
    mdc::Point point;

    for (const auto &id : roadLines.allPointsID) {
        point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(id));
        posArray.append(point.ly);
        posArray.append(point.bx);
        posArray.append(point.h);
    }

    return posArray;
}

QJsonObject VMapToJsonSingleton::getJsonObject(const QString &type, size_t ID, const QJsonArray &pos) const {
    QJsonObject object;

    object.insert("Type", type);
    object.insert("id", QString::number(ID));
    object.insert("Pos", pos);

    return object;
}

QJsonObject VMapToJsonSingleton::getDrivingArrowJsonObject(const QString &type, size_t ID, const QJsonArray &pos,
                                                           int typeIndex, double rotation) const {
    QJsonObject object;

    object.insert("Type", type);
    object.insert("id", QString::number(ID));
    object.insert("Pos", pos);
    object.insert("TypeIndex", typeIndex);
    object.insert("rotate", rotation);

    return object;
}

QJsonObject VMapToJsonSingleton::getTrafficLightsJsonObject(size_t ID, const QJsonArray &pos,
                                                            double rotation) const {
    QJsonObject object;

    object.insert("id", QString::number(ID));
    object.insert("Pos", pos);
    object.insert("rotate", rotation);

    return object;
}

QJsonObject VMapToJsonSingleton::getRoadLinesJsonObject(const QString &type, size_t ID, const QJsonArray &pos) const {
    QJsonObject object;

    object.insert("Type", type);
    object.insert("id", QString::number(ID));
    object.insert("Pos", pos);

    return object;
}

void
VMapToJsonSingleton::outputJsonObject(const std::string &dirPath, const QString &fileName,
                                      const QJsonArray &array) const {
    QJsonObject json;

    json.insert("Name", "Data");
    json.insert("Company", "Zhihui");
    json.insert("Date", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    json.insert("Data", QJsonValue(array));
    json.insert("Version", "1.0");

    QJsonDocument document;
    document.setObject(json);
    QByteArray byteArray = document.toJson(QJsonDocument::Indented);

    QString stdDirPath = QString::fromStdString(dirPath);
    QString filePath = stdDirPath + "/" + fileName + ".json";
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::warning(nullptr, "warning", "Can't save json file");
        return;
    }

    QTextStream in(&file);
    in << byteArray << "\n";
    file.close();
}