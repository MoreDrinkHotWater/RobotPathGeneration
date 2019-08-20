//
// Created by zhihui on 4/23/19.
//

#ifndef HDMAPS_TRAFFICLIGHTSDELETION_H
#define HDMAPS_TRAFFICLIGHTSDELETION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class TrafficLightsDeletion : public osgGA::GUIEventHandler {
public:
    explicit TrafficLightsDeletion(osg::Switch *rootNode);

    ~TrafficLightsDeletion() override = default;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> trafficLightsItemNode;

    osg::ref_ptr<osg::Geode> childNode;

    float x, y;
};


#endif //HDMAPS_TRAFFICLIGHTSDELETION_H
