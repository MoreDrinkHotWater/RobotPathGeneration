//
// Created by zhihui on 4/8/19.
//

#ifndef HDMAPS_LINEMODIFICATION_H
#define HDMAPS_LINEMODIFICATION_H

#include <osgGA/GUIEventHandler>

namespace osgViewer {
    class View;
}

class LineModification : public osgGA::GUIEventHandler {
public:
    explicit LineModification(osg::Switch *rootNode);

    ~LineModification() override;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

    void pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view);

private:
    void cleanup();

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> vectorItemNode;
    osg::ref_ptr<osg::Switch> pointCloudNode;

    osg::ref_ptr<osg::Geode> tempModLineGeode;

    std::pair<size_t, osg::Vec3d> selectedPoint;
    // 与所选点链接的点
    std::vector<std::pair<size_t, osg::Vec3d>> involvedPoints;

    // 需要重新绘制线段的ID
    std::vector<size_t> redrawLines;

    // 记录是第一次点击还是第二次点击
    int count;

    // 待删除点所在节点
    std::set<std::pair<osg::ref_ptr<osg::Switch>, osg::ref_ptr<osg::Geode>>> deletedPoint;

    float x, y;
};


#endif //HDMAPS_LINEMODIFICATION_H
