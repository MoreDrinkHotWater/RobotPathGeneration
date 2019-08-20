//
// Created by zhihui on 4/17/19.
//

#ifndef HDMAPS_PAVEMENTDELETION_H
#define HDMAPS_PAVEMENTDELETION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

namespace osgFX {
    class Outline;
}

class PavementDeletion : public osgGA::GUIEventHandler {
public:
    explicit PavementDeletion(osg::Switch *rootNode);

    ~PavementDeletion() override = default;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> pavementItemNode;

    osg::ref_ptr<osgFX::Outline> outline;
    osg::ref_ptr<osg::Geode> childNode;
    float x, y;
};


#endif //HDMAPS_PAVEMENTDELETION_H
