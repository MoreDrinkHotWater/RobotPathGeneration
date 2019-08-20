//
// Created by zhihui on 4/23/19.
//

#ifndef HDMAPS_DRIVINGARROWMODIFICATION_H
#define HDMAPS_DRIVINGARROWMODIFICATION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class DrivingArrowModification : public osgGA::GUIEventHandler {
public:
    explicit DrivingArrowModification(osg::Switch *rootNode);

    ~DrivingArrowModification() override = default;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> drivingArrowItemNode;

    osg::ref_ptr<osg::Geode> childNode;
    float x, y;
};


#endif //HDMAPS_DRIVINGARROWMODIFICATION_H
