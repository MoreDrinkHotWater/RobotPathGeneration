//
// Created by zhihui on 4/9/19.
//

#include <iostream>

#include <osg/Geode>
#include <osg/Switch>
#include <osgViewer/View>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osgFX/Outline>

#include "LineDeletion.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VectorMapSingleton.h"

LineDeletion::LineDeletion(osg::Switch *rootNode) : rootNode(rootNode), vectorItemNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, vectorItemNodeName))),
                                                    selectedLine(nullptr), outline(nullptr), childNode(nullptr), x(0),
                                                    y(0) {}

bool LineDeletion::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case (osgGA::GUIEventAdapter::KEYDOWN): {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape) {
                cleanup();
                return true;
            }
            return false;
        }
        case (osgGA::GUIEventAdapter::MOVE): {
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
                    if (childNode->getName().compare(0, 5, "Line_") == 0) {
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
        default:
            return false;
    }
}

void LineDeletion::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    osg::ref_ptr<osg::Switch> vectorNode(nullptr);
    osg::ref_ptr<osg::Geode> geodeNode;

    osg::ref_ptr<osg::Switch> lineTextNode;
    osg::ref_ptr<osg::Switch> pointTextNode;

    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        // 删除高亮节点
        if (outline != nullptr) {
            outline->removeChildren(0, outline->getNumChildren());
            outline->getParent(0)->removeChild(outline);
            outline = nullptr;
        }

        if (picker->containsIntersections()) {
            auto intersections = picker->getIntersections();
            for (const auto &intersection : intersections) {
                childNode = dynamic_cast<osg::Geode *>(intersection.nodePath.back());
                if (childNode == nullptr) {
                    break;
                }
                if (childNode->getName().compare(0, 5, "Line_") == 0) {
                    size_t deleteLineID = std::stoi(childNode->getName().substr(5));

                    mdc::Line deleteLine = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Line>(deleteLineID));

                    mdc::Point sPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(deleteLine.sPID));
                    mdc::Point ePoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(deleteLine.ePID));

                    auto iter = std::find(sPoint.fromPointLineID.begin(), sPoint.fromPointLineID.end(), deleteLine.lID);
                    sPoint.fromPointLineID.erase(iter);
                    VectorMapSingleton::getInstance()->update(sPoint);

                    iter = std::find(ePoint.toPointLineID.begin(), ePoint.toPointLineID.end(), deleteLineID);
                    ePoint.toPointLineID.erase(iter);
                    VectorMapSingleton::getInstance()->update(ePoint);

                    for (const auto &lineID : sPoint.toPointLineID) {
                        mdc::Line line = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Line>(lineID));
                        auto iter = std::find(line.fromThisLineID.begin(), line.fromThisLineID.end(), deleteLineID);
                        line.fromThisLineID.erase(iter);
                        VectorMapSingleton::getInstance()->update(line);
                    }

                    for (const auto &lineID : ePoint.fromPointLineID) {
                        mdc::Line line = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Line>(lineID));
                        auto iter = std::find(line.toThisLineID.begin(), line.toThisLineID.end(), deleteLineID);
                        line.toThisLineID.erase(iter);
                        VectorMapSingleton::getInstance()->update(line);
                    }

                    VectorMapSingleton::getInstance()->remove(deleteLine);

                    bool deleteSPoint, deleteEPoint;
                    deleteSPoint = deleteEPoint = false;
                    if (sPoint.fromPointLineID.empty() && sPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(sPoint);
                        deleteSPoint = true;
                    }
                    if (ePoint.fromPointLineID.empty() && ePoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(ePoint);
                        deleteEPoint = true;
                    }

                    vectorNode = dynamic_cast<osg::Switch *>(childNode->getParent(0));
                    std::cout << "vectorNode name: " << vectorNode->getName() << std::endl;
                    for (int i = 0; i < vectorNode->getNumChildren(); ++i) {
                        auto child = vectorNode->getChild(i);
                        std::cout << "vectorNode child name: " << child->getName() << std::endl;
                    }
                    std::cout << std::endl;
                    if (vectorNode != nullptr) {
                        vectorNode->removeChild(childNode);
                    }
                    childNode = nullptr;

                    if (deleteSPoint) {
                        int numItemVector = vectorItemNode->getNumChildren();
                        for (int i = 0; i < numItemVector; ++i) {
                            auto child = dynamic_cast<osg::Switch *>(vectorItemNode->getChild(i));
                            int numItemGeode = child->getNumChildren();
                            for (int j = 0; j < numItemGeode; ++j) {
                                std::cout << "child's num: " << child->getNumChildren() << std::endl;
                                auto deleteSphere = dynamic_cast<osg::Geode *>(child->getChild(j));
                                if (deleteSphere->getName() == "Sphere") {
                                    int sphereID = 0;
                                    deleteSphere->getUserValue("ID", sphereID);
                                    if (sphereID == sPoint.pID) {
                                        child->removeChild(deleteSphere);
                                    }
                                    if (numItemGeode != child->getNumChildren()) {
                                        numItemGeode = child->getNumChildren();
                                        j = 0;
                                    }
                                }
                            }
                        }
                    }

                    if (deleteEPoint) {
                        int numItemVector = vectorItemNode->getNumChildren();
                        for (int i = 0; i < numItemVector; ++i) {
                            auto child = dynamic_cast<osg::Switch *>(vectorItemNode->getChild(i));
                            int numItemGeode = child->getNumChildren();
                            for (int j = 0; j < numItemGeode; ++j) {
                                std::cout << "child's num: " << child->getNumChildren() << std::endl;
                                auto deleteSphere = dynamic_cast<osg::Geode *>(child->getChild(j));
                                if (deleteSphere->getName() == "Sphere") {
                                    int sphereID = 0;
                                    deleteSphere->getUserValue("ID", sphereID);
                                    if (sphereID == ePoint.pID) {
                                        child->removeChild(deleteSphere);
                                    }
                                    if (numItemGeode != child->getNumChildren()) {
                                        numItemGeode = child->getNumChildren();
                                        j = 0;
                                    }
                                }
                            }
                        }
                    }

                    lineTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                lineTextNodeName));
                    osg::ref_ptr<osg::Geode> lineTextGeode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(
                            rootNode, std::string("TLine_" + std::to_string(deleteLineID)).c_str()));
                    lineTextGeode->removeDrawables(0, lineTextGeode->getNumDrawables());
                    lineTextNode->removeChild(lineTextGeode);

                    pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                 pointTextNodeName));

                    int numPointGeode = pointTextNode->getNumChildren();
                    for (int i = 0; i < numPointGeode; ++i) {
                        auto child = dynamic_cast<osg::Geode *>(pointTextNode->getChild(i));
                        if (deleteSPoint) {
                            if (child->getName() == std::string("TSphere_" + std::to_string(sPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                        }
                        if (deleteEPoint) {
                            if (child->getName() == std::string("TSphere_" + std::to_string(ePoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                        }
                        if (numPointGeode != pointTextNode->getNumChildren()) {
                            numPointGeode = pointTextNode->getNumChildren();
                            i = -1;
                        }
                    }

                    break;
                }
            }
        }
    }
}


void LineDeletion::cleanup() {}