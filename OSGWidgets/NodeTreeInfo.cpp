//
// Created by zhihui on 3/28/19.
//

#include <iostream>

#include <osg/Switch>
#include <osg/Geode>
#include <osg/Camera>

#include "NodeTreeInfo.h"

NodeTreeInfo::NodeTreeInfo() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), indent(0) {
    std::cout << "------------------------------" << std::endl;
    std::cout << "NodeTreeInfo: " << std::endl;
}

void NodeTreeInfo::apply(osg::Switch &node) {
    for (int i = 0; i < indent; ++i) {
        std::cout << " ";
    }
    std::cout << "[" << indent + 1 << "]" << node.libraryName() << "::" << node.className() << "::" << node.getName()
              << std::endl;
    ++indent;
    traverse(node);
    --indent;
}

void NodeTreeInfo::apply(osg::Geode &node) {
    for (int i = 0; i < indent; ++i) {
        std::cout << " ";
    }
    std::cout << "[" << indent + 1 << "]" << node.libraryName() << "::" << node.className() << "::" << node.getName()
              << std::endl;
    ++indent;
    traverse(node);
    --indent;
}

void NodeTreeInfo::apply(osg::Camera &node) {
    for (int i = 0; i < indent; ++i) {
        std::cout << " ";
    }
    std::cout << "[" << indent + 1 << "]" << node.libraryName() << "::" << node.className() << "::" << node.getName()
              << std::endl;
    ++indent;
    traverse(node);
    --indent;
}