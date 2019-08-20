//
// Created by zhihui on 4/23/19.
//

#ifndef HDMAPS_ROADLINESEDITOR_H
#define HDMAPS_ROADLINESEDITOR_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class RoadLinesEditor : public osgGA::GUIEventHandler {
public:
    explicit RoadLinesEditor(osg::Switch *rootNode);

    ~RoadLinesEditor() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    void updateIndex();

    void rollback();

    void cleanup();

private:

    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> tempNode;
    osg::ref_ptr<osg::Geode> tempLineGeode;

    std::vector<std::pair<size_t, osg::Vec3d>> selectedPoints;

    size_t curPointIndex;

    float x, y;
};

std::ostream &operator<<(std::ostream &os, const osg::Vec3d &point);

#endif //HDMAPS_ROADLINESEDITOR_H
