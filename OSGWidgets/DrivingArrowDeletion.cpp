//
// Created by zhihui on 4/22/19.
//

#include <iostream>

#include <osg/Geode>
#include <osg/Switch>
#include <osgViewer/View>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgText/Text>

#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VectorMapSingleton.h"
#include "DrivingArrowDeletion.h"

DrivingArrowDeletion::DrivingArrowDeletion(osg::Switch *rootNode) : rootNode(rootNode), drivingArrowItemNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, drivingArrowItemNodeName))),
                                                                    childNode(nullptr), x(0), y(0) {}

bool DrivingArrowDeletion::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {

        case (osgGA::GUIEventAdapter::MOVE): {
            x = ea.getX();
            y = ea.getY();

            return false;
        }

        case (osgGA::GUIEventAdapter::RELEASE): {
            if (x == ea.getX() && y == ea.getY()) {
                pick(ea, view);
            }
            return true;
        }

        default:
            return false;
    }
}

void DrivingArrowDeletion::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    osg::ref_ptr<osg::Geode> geodeNode;

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
                if (childNode->getName().compare(0, 13, "DrivingArrow_") == 0) {
                    size_t deleteDrivingArrowID = std::stoi(childNode->getName().substr(13));

                    mdc::DrivingArrow drivingArrow = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::DrivingArrow>(deleteDrivingArrowID));
                    mdc::Point centerPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(drivingArrow.centerPoint));

                    VectorMapSingleton::getInstance()->remove(drivingArrow);

                    osg::ref_ptr<osg::Switch> itemDrivingArrowNode = dynamic_cast<osg::Switch *>(childNode->getParent(
                            0));
                    std::cout << "itemDrivingArrowNode name: " << itemDrivingArrowNode->getName() << std::endl;

                    if (itemDrivingArrowNode != nullptr) {
                        itemDrivingArrowNode->removeChild(childNode);
                    }
                    childNode = nullptr;

                    bool deleteCenterPoint = false;
                    if (centerPoint.fromPointLineID.empty() && centerPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(centerPoint);
                        deleteCenterPoint = true;
                    }

                    // 清理vectorItemNode上面挂载的Sphere节点（可能是line先被删除，只剩下drivingArrow节点）
                    osg::ref_ptr<osg::Switch> vectorItemNode = dynamic_cast<osg::Switch *> (NodeTreeSearch::findNodeWithName(
                            rootNode, vectorItemNodeName));
                    int numVectorItem = vectorItemNode->getNumChildren();
                    for (int i = 0; i < numVectorItem; ++i) {
                        auto child = dynamic_cast<osg::Switch *>(vectorItemNode->getChild(i));
                        int numItemGeode = child->getNumChildren();
                        for (int j = 0; j < numItemGeode; ++j) {
                            auto deleteSphere = dynamic_cast<osg::Geode *>(child->getChild(j));
                            if (deleteSphere->getName() == "Sphere") {
                                int sphereID = 0;
                                deleteSphere->getUserValue("ID", sphereID);
                                if (deleteCenterPoint) {
                                    if (sphereID == centerPoint.pID) {
                                        child->removeChild(deleteSphere);
                                    }
                                }
                                if (numItemGeode != child->getNumChildren()) {
                                    numItemGeode = child->getNumChildren();
                                    j = -1;
                                }
                            }
                        }
                    }

                    osg::ref_ptr<osg::Switch> pointTextNode = dynamic_cast<osg::Switch *> (NodeTreeSearch::findNodeWithName(
                            rootNode, pointTextNodeName));
                    int numPointGeode = pointTextNode->getNumChildren();
                    for (int i = 0; i < numPointGeode; ++i) {
                        auto child = dynamic_cast<osg::Geode *>(pointTextNode->getChild(i));
                        if (deleteCenterPoint) {
                            if (child->getName() == std::string("TSphere_" + std::to_string(centerPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                            if (numPointGeode != pointTextNode->getNumChildren()) {
                                numPointGeode = pointTextNode->getNumChildren();
                                i = -1;
                            }
                        }
                    }

                    osg::ref_ptr<osg::Switch> drivingArrowTextNode = dynamic_cast<osg::Switch *> (NodeTreeSearch::findNodeWithName(
                            rootNode, drivingArrowTextNodeName));
                    osg::ref_ptr<osg::Geode> drivingArrowTextGeode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(
                            rootNode, std::string("TDrivingArrow_" + std::to_string(drivingArrow.daID)).c_str()));
                    drivingArrowTextGeode->removeDrawables(0, drivingArrowTextGeode->getNumDrawables());
                    drivingArrowTextNode->removeChild(drivingArrowTextGeode);

                    break;
                }
            }
        }
    }
}