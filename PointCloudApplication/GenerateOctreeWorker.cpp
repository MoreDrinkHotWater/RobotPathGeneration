//
// Created by zhihui on 5/10/19.
//

#include <QFileInfo>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/outofcore/boost.h>

#include "GenerateOctreeWorker.h"

GenerateOctreeWorker::GenerateOctreeWorker(QObject *parent) : QObject(parent) {}

void GenerateOctreeWorker::generateOctreeData(const QString &filePath, bool hasRGB) {
    int depth = 6;

    if (hasRGB) {
        pcl::PointXYZRGB minPoint, maxPoint;

        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
        pcl::io::loadPCDFile(filePath.toStdString(), *cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

        fromPCLPointCloud2(*cloud, *cloudXYZRGB);

        getMinMax3D(*cloudXYZRGB, minPoint, maxPoint);

        const Eigen::Vector3d min(minPoint.x, minPoint.y, minPoint.z);
        const Eigen::Vector3d max(maxPoint.x, maxPoint.y, maxPoint.z);

        QString fileDir =
                QFileInfo(filePath).absolutePath() + "/.dataOctree/" + QFileInfo(filePath).fileName() + ".tree"
                        + "/tree.oct_idx";

        boost::filesystem::path indexFileLocation(fileDir.toStdString());

        pcl::outofcore::OutofcoreOctreeBase<>
                *octree = new pcl::outofcore::OutofcoreOctreeBase<>(depth, min, max, indexFileLocation, "ECEF");

        octree->addPointCloud_and_genLOD(cloud);

        delete octree;
    } else {

        pcl::PointXYZI minPoint, maxPoint;

        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
        pcl::io::loadPCDFile(filePath.toStdString(), *cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);

        fromPCLPointCloud2(*cloud, *cloudXYZI);

        getMinMax3D(*cloudXYZI, minPoint, maxPoint);

        const Eigen::Vector3d min(minPoint.x, minPoint.y, minPoint.z);
        const Eigen::Vector3d max(maxPoint.x, maxPoint.y, maxPoint.z);

        QString fileDir =
                QFileInfo(filePath).absolutePath() + "/.dataOctree/" + QFileInfo(filePath).fileName() + ".tree"
                        + "/tree.oct_idx";

        boost::filesystem::path indexFileLocation(fileDir.toStdString());

        pcl::outofcore::OutofcoreOctreeBase<>
                *octree = new pcl::outofcore::OutofcoreOctreeBase<>(depth, min, max, indexFileLocation, "ECEF");

        octree->addPointCloud_and_genLOD(cloud);

        delete octree;
    }
    emit doneGenerateOctreeDataSignal();
}