//
// Created by zhihui on 3/28/19.
//

#include <osg/Switch>
#include <osg/Node>

#include "NodeTreeSearch.h"

NodeTreeSearch::NodeTreeSearch(const char *name) : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), node(nullptr), name(name) {}

void NodeTreeSearch::apply(osg::Switch &searchNode) {
    if (searchNode.getName() == name) {
        node = &searchNode;
    }
    traverse(searchNode);
}

void NodeTreeSearch::apply(osg::Node &searchNode) {
    if (searchNode.getName() == name) {
        node = &searchNode;
    }
    traverse(searchNode);
}

osg::Node *NodeTreeSearch::findNodeWithName(const osg::ref_ptr<osg::Switch> &rootNode, const char *name) {
    osg::ref_ptr<NodeTreeSearch> visitor = new NodeTreeSearch(name);
    rootNode->accept(*visitor);
    osg::Node *node = visitor->getNode();
    return node;
}