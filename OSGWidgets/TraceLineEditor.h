//
// Created by zhihui on 4/28/19.
//

#ifndef HDMAPS_OSGWIDGETS_TRACELINEEDITOR_H_
#define HDMAPS_OSGWIDGETS_TRACELINEEDITOR_H_

#include <osgGA/GUIEventHandler>

namespace osgViewer {
class View;
}

class TraceLineEditor : public osgGA::GUIEventHandler {
 public:
  explicit TraceLineEditor(osg::Switch *rootNode);
  ~TraceLineEditor() override;

  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

 private:
  void updateIndex();

  // 回退函数
  void rollback();

  void cleanup();

  double calculateApproximateZ(const std::vector<osg::Vec3d> &points) const;

 private:
  osg::ref_ptr<osg::Switch> rootNode;
  osg::ref_ptr<osg::Switch> tempNode;
  osg::ref_ptr<osg::Switch> virtualPlaneNode;
  osg::ref_ptr<osg::Geode> tempLineGeode;

  std::vector<std::pair<size_t, osg::Vec3d>> selectedPoints;

  size_t curPointIndex;

  float x, y;
};

std::ostream &operator<<(std::ostream &os, const osg::Vec3d &point);

#endif //HDMAPS_OSGWIDGETS_TRACELINEEDITOR_H_
