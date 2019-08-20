//
// Created by zhihui on 6/25/19.
//

#include <QDir>
#include <QDebug>
#include <QThread>

#include <osg/Switch>
#include <osg/Geode>
#include <osg/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "ReadPCDataFiles.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "ground_extraction.h"
#include "VertexVisitor.h"

ReadPCDataFiles::ReadPCDataFiles(osg::Switch *rootNode,
                                 const QString &filesDirectory,
                                 bool hasBeenModified, QObject *parent)
        : rootNode(rootNode),

          groundNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, groundNodeName))),
          buildingNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, buildingNodeName))),

          filesDirectory(filesDirectory),
          hasBeenModified(hasBeenModified),
          allPoints(new osg::Vec3Array),
          QObject(parent) {}

void ReadPCDataFiles::readPCDataFromFiles() {
    qDebug() << "ReadPCDataFiles->ThreadID: " << QThread::currentThreadId();

    QDir dir(filesDirectory);
    QString fileSuffix;
    dir.setNameFilters({"*.pcd", "*.txt", "*.las"});
    QFileInfoList list = dir.entryInfoList();
    for (const QFileInfo &fileInfo : list) {
        fileSuffix = fileInfo.suffix();
        break;
    }

    osg::ref_ptr<osg::Switch> pointCloudNode =
            dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointCloudNodeName));
    osg::ref_ptr<osg::Geode> geode;

    if (fileSuffix == "pcd") {
        geode = readPCDataFromPCDFile();
    } else if (fileSuffix == "txt") {
        geode = readPCDataFromTXTFile();
    } else if (fileSuffix == "las") {
        geode = readPCDataFromLASFile();
    }

    if (geode != nullptr) {
        geode->setName("CloudPoints");
        // pointCloudNode 存储的是 pcd 的所有点数据
        pointCloudNode->addChild(geode.get());
    }

    emit readPCDataFinishSignal();
}

osg::ref_ptr<osg::Geode> ReadPCDataFiles::readPCDataFromPCDFile() {
    QString cloudPointType;

    QDir dir(filesDirectory);
    dir.setNameFilters({"*.pcd"});
    QFileInfoList list = dir.entryInfoList();

    // 读取PCD文件类型
    for (const QFileInfo &fileInfo : list) {
        pcl::PCLPointCloud2 cloud;
        QString filePath = fileInfo.absoluteFilePath();
        pcl::io::loadPCDFile(filePath.toStdString(), cloud);
        for (const auto &field : cloud.fields) {
            cloudPointType.append(QString(field.name.data()));
        }
        break;
    }

    if (cloudPointType.contains("rgb")) {
        // pcl::PointCloud<pcl::PointXYZRGB>
        mapXYZRGBPointClouds.clear();
        for (const QFileInfo &fileInfo : list) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PCDReader reader;
            reader.read(fileInfo.filePath().toStdString(), *pointCloud);
            mapXYZRGBPointClouds.push_back(pointCloud);
        }
        osg::ref_ptr<osg::Geode> geode = addXYZRGBMapPointCloud(osg::Vec3(0.4, 0.4, 0.4));
        return geode;
    } else {
        // pcl::PointCloud<pcl::PointXYZI>
        mapXYZIPointClouds.clear();
        for (const QFileInfo &fileInfo : list) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PCDReader reader;
            reader.read(fileInfo.filePath().toStdString(), *pointCloud);
            mapXYZIPointClouds.push_back(pointCloud);
        }
        osg::ref_ptr<osg::Geode> geode = addXYZIMapPointCloud(osg::Vec3(0.4, 0.4, 0.4));

        return geode;
    }
}

osg::ref_ptr<osg::Geode> ReadPCDataFiles::readPCDataFromTXTFile() {

}

osg::ref_ptr<osg::Geode> ReadPCDataFiles::readPCDataFromLASFile() {

}

osg::ref_ptr<osg::Geode> ReadPCDataFiles::addXYZIMapPointCloud(osg::Vec3 color) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    dataZ.clear();
    dataIntensity.clear();
    dataColor.clear();
    allPoints->clear();

    // 合并多份点云文件用于地面提取
    pcl::PointCloud<pcl::PointXYZI>::Ptr allPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    for (auto &mapPointCloud : mapXYZIPointClouds) {
        for (const auto &point : mapPointCloud->points) {
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            allPointCloud->push_back(point);
            allPoints->push_back(osg::Vec3(point.x, point.y, point.z));
            dataZ.push_back(point.z);
            dataIntensity.push_back(point.intensity);
        }
    }

    geom->setVertexArray(vertices.get());

    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
    colors->push_back(color);
    geom->setColorArray(colors.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(vertices->size())));
    geode->addDrawable(geom.get());



    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noGroundCloud(new pcl::PointCloud<pcl::PointXYZI>);

    pmProcessUrban::CGroundExtraction groundExtraction;
    groundExtraction.SetGridResolution(0.2);
    groundExtraction.SetMinPointNumInGrid(20);
    groundExtraction.SetMaxHeightDifference(0.1);

    groundExtraction.ExtractGroundPoint(allPointCloud, groundCloud, noGroundCloud);

    noGroundCloud->width = noGroundCloud->points.size();
    noGroundCloud->height = 1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr noGroundCloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(noGroundCloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.5);
    sor.filter(*noGroundCloudFiltered);

    // QString datafilePath = "";
    // if(readXmlFile())

    pcl::io::savePCDFile("/home/zhihui/TempPointCloud/ground.pcd", *groundCloud);
    pcl::io::savePCDFile("/home/zhihui/TempPointCloud/no_ground_filtered.pcd", *noGroundCloudFiltered);

    // 分割后的地面点加入到 groundNode 结点
    osg::ref_ptr<osg::Geode> tempGroundGeode;
    tempGroundGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> test_geom = new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
    // std::cout << "groundCloud'size: " << groundCloud->points.size() << std::endl;
    for (const auto point : groundCloud->points) {
        coords->push_back(osg::Vec3(point.x, point.y, point.z));
    }
    test_geom->setVertexArray(coords.get());

    test_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(coords->size())));

    tempGroundGeode->setName("GroundPoints");

    tempGroundGeode->addDrawable(test_geom.get());

    groundNode->addChild(tempGroundGeode);

    // 分割后的非地面点加入到 buildingNode 结点
    osg::ref_ptr<osg::Geode> tempNO_GroundGeode;
    tempNO_GroundGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> test_no_geom = new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> no_coords = new osg::Vec3Array();
    // std::cout << "groundCloud'size: " << noGroundCloudFiltered->points.size() << std::endl;
    for (const auto point : noGroundCloudFiltered->points) {
        no_coords->push_back(osg::Vec3(point.x, point.y, point.z));
    }
    test_no_geom->setVertexArray(no_coords.get());

    test_no_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(no_coords->size())));
    tempNO_GroundGeode->addDrawable(test_no_geom.get());

    buildingNode->addChild(tempNO_GroundGeode);



      return geode;
}

osg::ref_ptr<osg::Geode> ReadPCDataFiles::addXYZRGBMapPointCloud(osg::Vec3 color) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    dataZ.clear();
    dataIntensity.clear();
    dataColor.clear();
    allPoints->clear();

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    for (auto &mapPointCloud : mapXYZRGBPointClouds) {
        for (const auto &point : mapPointCloud->points) {
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            allPoints->push_back(osg::Vec3(point.x, point.y, point.z));
            dataZ.push_back(point.z);
            // 颜色位数应该从头文件中读取
            dataColor.emplace_back(osg::Vec3(point.r * 1.0 / 255, point.g * 1.0 / 255, point.b * 1.0 / 255));
        }
    }

    geom->setVertexArray(vertices.get());

    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
    colors->push_back(color);
    geom->setColorArray(colors.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(vertices->size())));
    geode->addDrawable(geom.get());

    return geode;
}

bool ReadPCDataFiles::readXmlFile(QString &DataFilesPath)
{

}