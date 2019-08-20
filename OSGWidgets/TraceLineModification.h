//
// Created by zhihui on 4/28/19.
//

#ifndef HDMAPS_OSGWIDGETS_TRACELINEMODIFICATION_H_
#define HDMAPS_OSGWIDGETS_TRACELINEMODIFICATION_H_

#include <osgGA/GUIEventHandler>

namespace osgViewer {
class View;
}

class TraceLineModification : public osgGA::GUIEventHandler {
 public:
  explicit TraceLineModification(osg::Switch *rootNode);

  ~TraceLineModification() override;

  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

 private:
  void cleanup();

  double calculateApproximateZ(const std::vector<osg::Vec3d> &points) const;

 private:
  osg::ref_ptr<osg::Switch> rootNode;
  osg::ref_ptr<osg::Switch> traceItemNode;
  osg::ref_ptr<osg::Switch> pointCloudNode;

  osg::ref_ptr<osg::Switch> virtualPlaneNode;
  osg::ref_ptr<osg::Geode> tempModTraceLineGeode;

  std::pair<size_t, osg::Vec3d> selectedPoint;
  // 与所选点链接的点
  std::vector<std::pair<size_t, osg::Vec3d>> involvedPoints;

  // 需要重新绘制线段的ID
  std::vector<size_t> redrawTraceLines;

  // 记录是第一次点击还是第二次点击
  int count;

  // 待删除点所在节点
  std::set<std::pair<osg::ref_ptr<osg::Switch>, osg::ref_ptr<osg::Geode>>> deletedPoint;

  float x, y;
};

#endif //HDMAPS_OSGWIDGETS_TRACELINEMODIFICATION_H_
