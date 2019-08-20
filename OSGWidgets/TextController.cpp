//
// Created by zhihui on 3/28/19.
//

#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osgViewer/View>

#include "TextController.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"

TextController::TextController(osg::Switch *node) : rootNode(node), textNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, textNodeName))), pointTextNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointTextNodeName))), lineTextNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, lineTextNodeName))) {}

bool TextController::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case osgGA::GUIEventAdapter::KEYDOWN: {
            if (ea.getKey() == '0') {
                initChildrenMask(textNode);
            }
            if (ea.getKey() == '1') {
                reverseNodeMask(pointTextNode);
            }
            if (ea.getKey() == '2') {
                reverseNodeMask(lineTextNode);
            }
            return true;
        }
        default:
            return false;
    }
}

void TextController::initChildrenMask(osg::Switch *node) {
    auto nodeMask = node->getNodeMask();

    for (unsigned int i = 0; i < node->getNumChildren(); ++i) {
        node->getChild(i)->setNodeMask(nodeMask);
    }
}

void TextController::reverseNodeMask(osg::Switch *node) {
    if (node->getNumChildren() == 0) {
        return;
    }

    auto nodeMask = node->getNodeMask();
    auto newMask = nodeMask == 0 ? 1 : 0;
    node->setNodeMask(newMask);
}