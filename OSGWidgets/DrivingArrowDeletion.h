//
// Created by zhihui on 4/22/19.
//

#ifndef HDMAPS_DRIVINGARROWDELETION_H
#define HDMAPS_DRIVINGARROWDELETION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class DrivingArrowDeletion : public osgGA::GUIEventHandler {
public:
    explicit DrivingArrowDeletion(osg::Switch *rootNode);

    ~DrivingArrowDeletion() override = default;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> drivingArrowItemNode;

    osg::ref_ptr<osg::Geode> childNode;
    float x, y;
};


#endif //HDMAPS_DRIVINGARROWDELETION_H
