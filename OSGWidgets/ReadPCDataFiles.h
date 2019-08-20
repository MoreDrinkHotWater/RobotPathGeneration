//
// Created by zhihui on 6/25/19.
//

#ifndef HDMAPSFORROBOT_OSGWIDGETS_READPCDATAFILES_H_
#define HDMAPSFORROBOT_OSGWIDGETS_READPCDATAFILES_H_

#include <QObject>
#include <osg/ref_ptr>

namespace osg {
class Switch;
}

class ReadPCDataFiles : public QObject {
 Q_OBJECT
 public:
  explicit ReadPCDataFiles(osg::Switch *rootNode,
                           const QString &filesDirectory,
                           bool hasBeenModified,
                           QObject *parent = nullptr);
  ~ReadPCDataFiles() override = default;
  friend class OSGWidget;

 Q_SIGNALS:
  void readPCDataFinishSignal();

 public Q_SLOTS:
  void readPCDataFromFiles();

 private:
  osg::ref_ptr<osg::Geode> readPCDataFromPCDFile();
  osg::ref_ptr<osg::Geode> readPCDataFromTXTFile();
  osg::ref_ptr<osg::Geode> readPCDataFromLASFile();

 private:
  osg::ref_ptr<osg::Geode> addXYZIMapPointCloud(osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));

  osg::ref_ptr<osg::Geode> addXYZRGBMapPointCloud(osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));

 private:
  osg::ref_ptr<osg::Switch> rootNode;

  // 地面结点
  osg::ref_ptr<osg::Switch> groundNode;
  // 非地面结点
  osg::ref_ptr<osg::Switch> buildingNode;

  QString filesDirectory;
  bool hasBeenModified;

 private:
  // 保存多文件点云数据
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapXYZIPointClouds;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mapXYZRGBPointClouds;

  // 保存高程数据
  std::vector<float> dataZ;

  // 保存强度信息
  std::vector<float> dataIntensity;

  // 保存颜色信息
  std::vector<osg::Vec3> dataColor;

  // 保存所有点云的点
  osg::ref_ptr<osg::Vec3Array> allPoints;

  bool readXmlFile(QString &DataFilesPath);
};

#endif //HDMAPSFORROBOT_OSGWIDGETS_READPCDATAFILES_H_
