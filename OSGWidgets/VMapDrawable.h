//
// Created by zhihui on 4/4/19.
//

#ifndef HDMAPS_VMAPDRAWABLE_H
#define HDMAPS_VMAPDRAWABLE_H

#include <vector>

#include <osg/ref_ptr>

#include "../Common/DataStructure.h"

namespace osg {
    class Switch;

    class Node;

    class Geode;

    class Vec3d;

    class Vec4f;
}

class VMapDrawable {
public:
    explicit VMapDrawable(osg::Switch *rootNode);

    ~VMapDrawable() = default;

    template<class T>
    void drawVectorNode(const std::vector<mdc::Line> &lines, const T &object);

    template<class T>
    void setNodeValue(const T &object, osg::Node *node);

private:
    osg::ref_ptr<osg::Geode>
    drawTextGeode(const osg::Vec3d &pos, const std::string &content, const osg::Vec4f &color, float size = 0.5);

    void setNullNodeValue(osg::Node *node);

    void setLineNodeValue(const mdc::Line &object, osg::Node *node);

private:
    osg::ref_ptr<osg::Switch> rootNode;
};


#endif //HDMAPS_VMAPDRAWABLE_H
