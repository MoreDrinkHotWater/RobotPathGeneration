//
// Created by zhihui on 4/23/19.
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
#include "TrafficLightsDeletion.h"

TrafficLightsDeletion::TrafficLightsDeletion(osg::Switch *rootNode) : rootNode(rootNode), trafficLightsItemNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, trafficLightsItemNodeName))),
                                                                      childNode(nullptr), x(0), y(0) {}

bool TrafficLightsDeletion::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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

void TrafficLightsDeletion::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
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
                if (childNode->getName().compare(0, 14, "TrafficLights_") == 0) {
                    size_t deleteTrafficLightsID = std::stoi(childNode->getName().substr(14));

                    mdc::TrafficLights trafficLights = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::TrafficLights>(deleteTrafficLightsID));
                    mdc::Point lightLocationPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(trafficLights.lightLocationPoint));

                    VectorMapSingleton::getInstance()->remove(trafficLights);

                    osg::ref_ptr<osg::Switch> itemTrafficLightsNode = dynamic_cast<osg::Switch *>(childNode->getParent(
                            0));
                    std::cout << "itemTrafficLightsNode name: " << itemTrafficLightsNode->getName() << std::endl;

                    if (itemTrafficLightsNode != nullptr) {
                        itemTrafficLightsNode->removeChild(childNode);
                    }
                    childNode = nullptr;

                    bool deleteLightLocationPoint = false;
                    if (lightLocationPoint.fromPointLineID.empty() && lightLocationPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(lightLocationPoint);
                        deleteLightLocationPoint = true;
                    }

                    // 清理vectorItemNode上面挂载的Sphere节点（可能是line先被删除，只剩下trafficLights节点）
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
                                if (deleteLightLocationPoint) {
                                    if (sphereID == lightLocationPoint.pID) {
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
                        if (deleteLightLocationPoint) {
                            if (child->getName() == std::string("TSphere_" + std::to_string(lightLocationPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                            if (numPointGeode != pointTextNode->getNumChildren()) {
                                numPointGeode = pointTextNode->getNumChildren();
                                i = -1;
                            }
                        }
                    }

                    osg::ref_ptr<osg::Switch> trafficLightsTextNode = dynamic_cast<osg::Switch *> (NodeTreeSearch::findNodeWithName(
                            rootNode, trafficLightsTextNodeName));
                    osg::ref_ptr<osg::Geode> trafficLightsTextGeode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(
                            rootNode, std::string("TTrafficLights_" + std::to_string(trafficLights.tlID)).c_str()));
                    trafficLightsTextGeode->removeDrawables(0, trafficLightsTextGeode->getNumDrawables());
                    trafficLightsTextNode->removeChild(trafficLightsTextGeode);

                    break;
                }
            }
        }
    }
}