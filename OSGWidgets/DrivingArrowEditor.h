//
// Created by zhihui on 4/18/19.
//

#ifndef HDMAPS_DRIVINGARROWEDITOR_H
#define HDMAPS_DRIVINGARROWEDITOR_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class DrivingArrowEditor : public osgGA::GUIEventHandler {
public:
    explicit DrivingArrowEditor(osg::Switch *rootNode);

    ~DrivingArrowEditor() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    void updateIndex();

    void rollback();

    void cleanup();

    double calculatePointYAngle(const osg::Vec2 &p1, const osg::Vec2 &p2 = osg::Vec2(0.0, 1.0)) const;

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> tempNode;
    osg::ref_ptr<osg::Geode> tempLineGeode;

    std::vector<std::pair<size_t, osg::Vec3d>> selectedPoints;

    osg::ref_ptr<osg::Switch> drivingArrowItemNode;

    size_t curPointIndex;

    float x, y;
};

std::ostream &operator<<(std::ostream &os, const osg::Vec3d &point);

#endif //HDMAPS_DRIVINGARROWEDITOR_H
