//
// Created by zhihui on 4/24/19.
//

#ifndef HDMAPS_OSGWIDGETS_ROADLINESDELETION_H_
#define HDMAPS_OSGWIDGETS_ROADLINESDELETION_H_

#include <osgGA/GUIEventHandler>

namespace osgViewer {
class View;
}

namespace osgFX {
class Outline;
}

class RoadLinesDeletion : public osgGA::GUIEventHandler {
 public:
  explicit RoadLinesDeletion(osg::Switch *rootNode);

  ~RoadLinesDeletion() override = default;

  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

 private:
  osg::ref_ptr<osg::Switch> rootNode;
  osg::ref_ptr<osg::Switch> roadLinesItemNode;

  osg::ref_ptr<osgFX::Outline> outline;
  osg::ref_ptr<osg::Geode> childNode;
  float x, y;
};

#endif //HDMAPS_OSGWIDGETS_ROADLINESDELETION_H_
