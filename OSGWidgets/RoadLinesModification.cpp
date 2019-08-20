//
// Created by zhihui on 4/24/19.
//

#include <iostream>
#include <iomanip>

#include <osg/Geode>
#include <osg/Switch>
#include <osgViewer/View>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osgFX/Outline>

#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VectorMapSingleton.h"
#include "RoadLinesModification.h"
#include "RoadLinesEditorDialog.h"
#include "Color.h"

RoadLinesModification::RoadLinesModification(osg::Switch *rootNode)
        : rootNode(rootNode),
          roadLinesItemNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                         roadLinesItemNodeName))),
          outline(nullptr),
          childNode(nullptr),
          x(0),
          y(0) {}

RoadLinesModification::~RoadLinesModification() {
    if (outline != nullptr) {
        outline->removeChildren(0, outline->getNumChildren());
        outline->getParent(0)->removeChild(outline);
        outline = nullptr;
    }
}

bool RoadLinesModification::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {

        case (osgGA::GUIEventAdapter::MOVE) : {
            x = ea.getX();
            y = ea.getY();

            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);

            if (picker->containsIntersections()) {
                auto intersections = picker->getIntersections();
                for (const auto &intersection : intersections) {
                    childNode = dynamic_cast<osg::Geode *>(intersection.nodePath.back());
                    // 当前节点可能已经被删除
                    if (childNode == nullptr) {
                        break;
                    }
                    if (childNode->getName().compare(0, 10, "RoadLines_") == 0) {
                        if (outline == nullptr) {
                            outline = new osgFX::Outline;
                            outline->setWidth(3);
                            outline->setName("Outline");
                            childNode->getParent(0)->addChild(outline);
                        } else {
                            outline->removeChildren(0, outline->getNumChildren());
                        }
                        outline->addChild(childNode);
                        break;
                    } else {
                        if (outline != nullptr) {
                            outline->removeChildren(0, outline->getNumChildren());
                        }
                    }

                }
            } else {
                if (outline != nullptr) {
                    outline->removeChildren(0, outline->getNumChildren());
                }
            }
        }
        case (osgGA::GUIEventAdapter::RELEASE): {
            if (x == ea.getX() && y == ea.getY()) {
                pick(ea, view);
            }
            return true;
        }
        default:return false;
    }
}

void RoadLinesModification::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        if (picker->containsIntersections()) {
            auto intersections = picker->getIntersections();
            for (const auto &intersection : intersections) {
                childNode = dynamic_cast<osg::Geode *>(intersection.nodePath.back());
                if (childNode == nullptr) {
                    break;
                }
                if (childNode->getName().compare(0, 10, "RoadLines_") == 0) {
                    size_t deleteRoadLinesID = std::stoi(childNode->getName().substr(10));

                    mdc::RoadLines roadLines = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::RoadLines>(deleteRoadLinesID));

                    auto *roadLinesEditorDialog = new RoadLinesEditorDialog(&roadLines.type);
                    roadLinesEditorDialog->exec();
                    if (roadLines.type.empty()) {
                        return;
                    }

                    VectorMapSingleton::getInstance()->update(roadLines);

                    break;
                }
            }
        }

        // 删除高亮节点
        if (outline != nullptr) {
            outline->removeChildren(0, outline->getNumChildren());
            outline->getParent(0)->removeChild(outline);
            outline = nullptr;
        }
    }
}

