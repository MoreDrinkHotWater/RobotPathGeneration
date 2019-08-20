//
// Created by zhihui on 3/28/19.
//

#ifndef POINTCLOUDAPPLICATION_NODETREESEARCH_H
#define POINTCLOUDAPPLICATION_NODETREESEARCH_H

#include <string>

#include <osg/NodeVisitor>

class NodeTreeSearch : public osg::NodeVisitor {
public:
    explicit NodeTreeSearch(const char *name);

    void apply(osg::Switch &searchNode) override;

    void apply(osg::Node &searchNode) override;

    osg::Node *getNode() const { return node; };

    static osg::Node *findNodeWithName(const osg::ref_ptr<osg::Switch> &rootNode, const char *name);

private:
    osg::Node *node;
    std::string name;
};


#endif //POINTCLOUDAPPLICATION_NODETREESEARCH_H
