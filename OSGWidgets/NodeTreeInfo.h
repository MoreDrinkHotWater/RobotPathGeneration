//
// Created by zhihui on 3/28/19.
//

#ifndef POINTCLOUDAPPLICATION_NODETREEINFO_H
#define POINTCLOUDAPPLICATION_NODETREEINFO_H

#include <osg/NodeVisitor>

class NodeTreeInfo : public osg::NodeVisitor {
public:
    NodeTreeInfo();

    void apply(osg::Switch &node) override;

    void apply(osg::Geode &node) override;

    void apply(osg::Camera &node) override;

private:
    int indent;
};


#endif //POINTCLOUDAPPLICATION_NODETREEINFO_H
