//
// Created by zhihui on 4/28/19.
//

#ifndef HDMAPS_OSGWIDGETS_TRACELINEDELETION_H_
#define HDMAPS_OSGWIDGETS_TRACELINEDELETION_H_

#include <osgGA/GUIEventHandler>

namespace osgViewer {
class View;
}

namespace osgFX {
class Outline;
}

class TraceLineDeletion : public osgGA::GUIEventHandler {
 public:
  explicit TraceLineDeletion(osg::Switch *rootNode);

  ~TraceLineDeletion() override = default;

  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

 private:
  // not necessary
  void cleanup();

 private:
  osg::ref_ptr<osg::Switch> rootNode;
  osg::ref_ptr<osg::Switch> traceItemNode;
  osg::ref_ptr<osg::Geode> selectedTraceLine;
  osg::ref_ptr<osgFX::Outline> outline;
  osg::ref_ptr<osg::Geode> childNode;
  float x, y;
};

#endif //HDMAPS_OSGWIDGETS_TRACELINEDELETION_H_
