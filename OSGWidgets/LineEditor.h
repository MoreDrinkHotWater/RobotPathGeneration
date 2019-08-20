//
// Created by zhihui on 3/28/19.
//

#ifndef POINTCLOUDAPPLICATION_LINEEDITOR_H
#define POINTCLOUDAPPLICATION_LINEEDITOR_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class LineEditor : public osgGA::GUIEventHandler {
public:
    explicit LineEditor(osg::Switch *rootNode);

    ~LineEditor() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    void updateIndex();

    // 回退函数
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

#endif //POINTCLOUDAPPLICATION_LINEEDITOR_H
