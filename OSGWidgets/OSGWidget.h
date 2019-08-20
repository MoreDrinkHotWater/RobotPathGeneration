//
// Created by zhihui on 3/27/19.
//

#ifndef POINTCLOUDAPPLICATION_OSGWIDGET_H
#define POINTCLOUDAPPLICATION_OSGWIDGET_H

#include <QWidget>
#include <QFileInfo>
#include <QThread>

#include <osgViewer/CompositeViewer>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <liblas/liblas.hpp>

#include "LineEditor.h"
#include "LineModification.h"
#include "LineDeletion.h"
#include "TraceLineEditor.h"
#include "TraceLineModification.h"
#include "TraceLineDeletion.h"
#include "PavementEditor.h"
#include "PavementDeletion.h"
#include "DrivingArrowEditor.h"
#include "DrivingArrowModification.h"
#include "DrivingArrowDeletion.h"
#include "TrafficLightsEditor.h"
#include "TrafficLightsDeletion.h"
#include "RoadLinesEditor.h"
#include "RoadLinesModification.h"
#include "RoadLinesDeletion.h"
#include "ProgressBarWorker.h"
#include "MeasurePoints.h"

class QTimer;

class QFileInfo;

class QProgressDialog;

class ProgressBarWorker;

class ClearIrrelevantPoints;

class ClearPointsEvents;

namespace osgQt {
class GraphicsWindowQt;
}

class ReadPCDataFiles;

class OSGWidget : public QWidget, public osgViewer::CompositeViewer {
 Q_OBJECT
 public:
  explicit OSGWidget(QWidget *parent = nullptr);

  ~OSGWidget() override;

 public:
  void init();

  void reset();

  void readPCDataFromFile(const QFileInfo &fileInfo,
                          bool hasIntensity,
                          bool hasRGB,
                          const QString &originalPCDFileName);

  void readPCDataFromFiles(const QString &filesDirectory, bool hasBeenModified);

  void loadVectorMap();

  void initTerrainManipulator();

  void activeLineEditor(bool isActive);

  void activeLineModification(bool isActive);

  void activeLineDeletion(bool isActive);

  void activeTraceLineEditor(bool isActive);

  void activeTraceLineModification(bool isActive);

  void activeTraceLineDeletion(bool isActive);

  void activePavementEditor(bool isActive);

  void activePavementDeletion(bool isActive);

  void activeDrivingArrowEditor(bool isActive);

  void activeDrivingArrowModification(bool isActive);

  void activeDrivingArrowDeletion(bool isActive);

  void activeTrafficLightsEditor(bool isActive);

  void activeTrafficLightsDeletion(bool isActive);

  void activeRoadLinesEditor(bool isActive);

  void activeRoadLinesModification(bool isActive);

  void activeRoadLinesDeletion(bool isActive);

  void activeMeasurePoints(bool isActive);

  void activeColorByZ(bool isActive);

  void activeColorByIntensity(bool isActive);

  void activeColorByTexture(bool isActive);

  void activeClearIrrelevantPoints(bool isActive);

  void transENU2LLH() const;

  void saveVectorMap(const std::string &dirPath) const;

  void transVectorMapToJson(const std::string &dirPath) const;

  void transVectorMapToCSV(const std::string &dirPath) const;

  void transAllPointsToJSON(const std::string &dirPath) const;

 private:
  void paintEvent(QPaintEvent *) override;

  void initSceneGraph();

  void initCamera();

  void initEditor();

  void initManipulator();

  void initVectorMap();

  osgQt::GraphicsWindowQt *
  createGraphicsWindow(int x, int y, int w, int h, const std::string &name = "", bool windowDecoration = false);

  osg::ref_ptr<osg::Geode> readPCDataFromPCDFile(const QFileInfo &fileInfo, bool hasIntensity, bool hasRGB);

  osg::ref_ptr<osg::Geode> readPCDataFromTXTFile(const QFileInfo &fileInfo, bool hasIntensity, bool hasRGB);

  osg::ref_ptr<osg::Geode> readPCDataFromLASFlile(const QFileInfo &fileInfo, bool hasIntensity, bool hasRGB);

  osg::ref_ptr<osg::Geode> readPCLDataFromXYZIFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readPCLDataFromXYZRGBFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readTXTDataFromIFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readTXTDataFromRGBFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readTXTDataFromIRGBFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readTXTDataFromFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readLASDataFromIFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readLASDataFromRGBFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readLASDataFromIRGBFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> readLASDataFromFile(const QFileInfo &fileInfo);

  osg::ref_ptr<osg::Geode> addMapPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &mapPointCloud,
                                            osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));

  osg::ref_ptr<osg::Geode> addMapPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mapPointCloud,
                                            osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0));

  osg::ref_ptr<osg::Geode> addMapPointCloud(const osg::ref_ptr<osg::Vec3Array> &vertices,
                                            osg::Vec3 color = osg::Vec3(1.0, 1.0, 1.0)) const;

  void colorPointCloudDataByZ(bool isActive) const;

  void colorPointCloudDataByIntensity(bool isActive) const;

  void colorPointCloudDataByTexture(bool isActive) const;

  osg::ref_ptr<osg::Vec3Array> calculateColorArrayZ() const;

  osg::ref_ptr<osg::Vec3Array> calculateColorArrayIntensity() const;

  osg::ref_ptr<osg::Vec3Array> calculateColorArrayTexture() const;

  // 是否有强度信息
  bool hasDataIntensity() const;

  void drawVectorItems();

  void drawTraceItems();

  void drawPavementItems();

  void drawDrivingArrowItems();

  void drawTrafficLightsItems();

  void drawRoadLinesItems();

  void clearIrrelevantPiont();

  template<class T>
  void drawVectorItems(const std::vector<T> &objects);

 Q_SIGNALS:
  void showProgressBarSignal(const QString &progressDialogLabelText);
  void closeProgressBarSignal();

  void readPCDataFromFilesSignal();

  void clearIrrelevantPointsSignal(osg::Switch *rootnode, osgViewer::View *mainview, bool isActive);

 private Q_SLOTS:

  void updateFrame();

 private:
  osg::ref_ptr<osgViewer::View> mainView;
  osg::ref_ptr<osg::Switch> rootNode;

  // bool isactive;

  osg::ref_ptr<LineEditor> lineEditor;
  osg::ref_ptr<LineModification> lineModification;
  osg::ref_ptr<LineDeletion> lineDeletion;

  osg::ref_ptr<TraceLineEditor> traceLineEditor;
  osg::ref_ptr<TraceLineModification> traceLineModification;
  osg::ref_ptr<TraceLineDeletion> traceLineDeletion;

  osg::ref_ptr<PavementEditor> pavementEditor;
  osg::ref_ptr<PavementDeletion> pavementDeletion;

  osg::ref_ptr<DrivingArrowEditor> drivingArrowEditor;
  osg::ref_ptr<DrivingArrowModification> drivingArrowModification;
  osg::ref_ptr<DrivingArrowDeletion> drivingArrowDeletion;

  osg::ref_ptr<TrafficLightsEditor> trafficLightsEditor;
  osg::ref_ptr<TrafficLightsDeletion> trafficLightsDeletion;

  osg::ref_ptr<RoadLinesEditor> roadLinesEditor;
  osg::ref_ptr<RoadLinesModification> roadLinesModification;
  osg::ref_ptr<RoadLinesDeletion> roadLinesDeletion;

  osg::ref_ptr<MeasurePoints> measurePointsTool;

  // osg::ref_ptr<ClearPointsEvents> clearPointsEvents;

  // 保存高程数据
  std::vector<float> dataZ;

  // 保存强度信息
  std::vector<float> dataIntensity;

  // 保存颜色信息
  std::vector<osg::Vec3> dataColor;

  // 保存所有点云的点
  osg::ref_ptr<osg::Vec3Array> allPoints;

  QTimer *updateTimer;

  QFileInfo openFileInfo;

  QString originalPCDFileName;

  // QProgressDialog *progressDialog;

  ProgressBarWorker *progressBarWorker;

  QThread progressBarThread;

  ReadPCDataFiles *readPCDataFiles;
  QThread readPCDataFilesThread;

  ClearIrrelevantPoints *clearIrrelevantPoints;

  QThread clearPointThread;
};



#endif //POINTCLOUDAPPLICATION_OSGWIDGET_H
