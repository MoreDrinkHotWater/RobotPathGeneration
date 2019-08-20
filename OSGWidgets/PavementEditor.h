//
// Created by zhihui on 4/16/19.
//

#ifndef HDMAPS_PAVEMENTEDITOR_H
#define HDMAPS_PAVEMENTEDITOR_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class PavementEditor : public osgGA::GUIEventHandler {
public:
    explicit PavementEditor(osg::Switch *rootNode);

    ~PavementEditor() override;

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

#endif //HDMAPS_PAVEMENTEDITOR_H
