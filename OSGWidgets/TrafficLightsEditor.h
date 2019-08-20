//
// Created by zhihui on 4/22/19.
//

#ifndef HDMAPS_TRAFFICLIGHTSEDITOR_H
#define HDMAPS_TRAFFICLIGHTSEDITOR_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class TrafficLightsEditor : public osgGA::GUIEventHandler {
public:
    explicit TrafficLightsEditor(osg::Switch *rootNode);

    ~TrafficLightsEditor() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    void updateIndex();

    void rollback();

    void cleanup();

    double calculatePointXAngle(const osg::Vec2 &p1, const osg::Vec2 &p2 = osg::Vec2(1.0, 0.0)) const;

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> tempNode;
    osg::ref_ptr<osg::Geode> tempLineGeode;

    std::vector<std::pair<size_t, osg::Vec3d>> selectedPoints;

    osg::ref_ptr<osg::Switch> trafficLightsItemNode;

    size_t curPointIndex;

    float x, y;
};

std::ostream &operator<<(std::ostream &os, const osg::Vec3d &point);

#endif //HDMAPS_TRAFFICLIGHTSEDITOR_H
