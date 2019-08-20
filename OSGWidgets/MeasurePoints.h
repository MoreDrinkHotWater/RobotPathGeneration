//
// Created by zhihui on 6/13/19.
//

#ifndef HDMAPSFORROBOT_OSGWIDGETS_MEASUREPOINTS_H_
#define HDMAPSFORROBOT_OSGWIDGETS_MEASUREPOINTS_H_

#include <osgGA/GUIEventHandler>

namespace osgViewer {
class View;
}

class MeasurePoints : public osgGA::GUIEventHandler {
 public:
  explicit MeasurePoints(osg::Switch *rootNode);

  ~MeasurePoints() override;

  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

 private:

  double calculateDistance(const osg::Vec3d &p1, const osg::Vec3d &p2) const;

  void cleanup();

 private:
  osg::ref_ptr<osg::Switch> rootNode;
  osg::ref_ptr<osg::Switch> measureNode;
  osg::ref_ptr<osg::Geode> tempLineGeode;

  std::vector<std::pair<size_t, osg::Vec3d>> selectedPoints;

  size_t curPointIndex;

  float x, y;
};

#endif //HDMAPSFORROBOT_OSGWIDGETS_MEASUREPOINTS_H_
