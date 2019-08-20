//
// Created by zhihui on 3/28/19.
//

#include <osg/Node>
#include <osgViewer/Viewer>

#include "NodeTreeHandler.h"
#include "NodeTreeInfo.h"

NodeTreeHandler::NodeTreeHandler(osg::Switch *node) : rootNode(node) {}

bool NodeTreeHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case osgGA::GUIEventAdapter::KEYDOWN: {
            if (ea.getKey() == 'i') {
                NodeTreeInfo nodeTreeInfo;
                if (rootNode != nullptr) {
                    rootNode->accept(nodeTreeInfo);
                }
                return true;
            }
        }
        default:
            break;
    }
    return false;
}