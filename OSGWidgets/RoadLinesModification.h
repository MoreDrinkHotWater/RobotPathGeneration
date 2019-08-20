//
// Created by zhihui on 4/24/19.
//

#ifndef HDMAPS_ROADLINESMODIFICATION_H
#define HDMAPS_ROADLINESMODIFICATION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

namespace osgFX {
    class Outline;
}

class RoadLinesModification : public osgGA::GUIEventHandler {
public:
    explicit RoadLinesModification(osg::Switch *rootNode);

    ~RoadLinesModification() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> roadLinesItemNode;

    osg::ref_ptr<osgFX::Outline> outline;
    osg::ref_ptr<osg::Geode> childNode;
    float x, y;
};


#endif //HDMAPS_ROADLINESMODIFICATION_H
