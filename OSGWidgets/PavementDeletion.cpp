//
// Created by zhihui on 4/17/19.
//

#include <iostream>

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
#include "PavementDeletion.h"

PavementDeletion::PavementDeletion(osg::Switch *rootNode) : rootNode(rootNode), pavementItemNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pavementItemNodeName))),
                                                            outline(nullptr), childNode(nullptr), x(0), y(0) {}

bool PavementDeletion::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case (osgGA::GUIEventAdapter::KEYDOWN) : {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape) {
                return true;
            }
            return false;
        }
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
                    if (childNode->getName().compare(0, 9, "Pavement_") == 0) {
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

void PavementDeletion::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    osg::ref_ptr<osg::Switch> vectorNode(nullptr);
    osg::ref_ptr<osg::Geode> geodeNode;


    osg::ref_ptr<osg::Switch> pointTextNode;
    osg::ref_ptr<osg::Switch> pavementTextNode;

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
                if (childNode->getName().compare(0, 9, "Pavement_") == 0) {
                    size_t deletePavementID = std::stoi(childNode->getName().substr(9));

                    mdc::Pavement pavement = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Pavement>(deletePavementID));
                    mdc::Point upperLeftCornerPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(pavement.upperLeftCorner));
                    mdc::Point lowerLeftCornerPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(pavement.lowerLeftCorner));
                    mdc::Point upperRightCornerPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(pavement.upperRightCorner));
                    mdc::Point lowerRightCornerPoint = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(pavement.lowerRightCorner));

                    VectorMapSingleton::getInstance()->remove(pavement);

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

                    bool deleteUpperLeftCornerPoint, deleteLowerLeftCornerPoint, deleteUpperRightCornerPoint, deleteLowerRightCornerPoint;
                    deleteUpperLeftCornerPoint = deleteLowerLeftCornerPoint =
                    deleteUpperRightCornerPoint = deleteLowerRightCornerPoint = false;
                    if (upperLeftCornerPoint.fromPointLineID.empty() && upperLeftCornerPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(upperLeftCornerPoint);
                        deleteUpperLeftCornerPoint = true;
                    }
                    if (lowerLeftCornerPoint.fromPointLineID.empty() && lowerLeftCornerPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(lowerLeftCornerPoint);
                        deleteLowerLeftCornerPoint = true;
                    }

                    if (upperRightCornerPoint.fromPointLineID.empty() && upperRightCornerPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(upperRightCornerPoint);
                        deleteUpperRightCornerPoint = true;
                    }

                    if (lowerRightCornerPoint.fromPointLineID.empty() && lowerRightCornerPoint.toPointLineID.empty()) {
                        VectorMapSingleton::getInstance()->remove(lowerRightCornerPoint);
                        deleteLowerRightCornerPoint = true;
                    }

                    int numItemPavement = pavementItemNode->getNumChildren();
                    std::cout << "name: " << pavementItemNode->getName() << ", num: " << numItemPavement << std::endl;
                    for (int i = 0; i < numItemPavement; ++i) {
                        auto child = dynamic_cast<osg::Switch *>(pavementItemNode->getChild(i));
                        int numItemGeode = child->getNumChildren();
                        for (int j = 0; j < numItemGeode; ++j) {
                            auto deleteSphere = dynamic_cast<osg::Geode *> (child->getChild(j));
                            if (deleteSphere->getName() == "Sphere") {
                                int sphereID = 0;
                                deleteSphere->getUserValue("ID", sphereID);
                                if (sphereID == upperLeftCornerPoint.pID || sphereID == lowerLeftCornerPoint.pID ||
                                    sphereID == upperRightCornerPoint.pID || sphereID == lowerRightCornerPoint.pID) {
                                    child->removeChild(deleteSphere);
                                }
                                if (numItemGeode != child->getNumChildren()) {
                                    numItemGeode = child->getNumChildren();
                                    j = -1;
                                }
                            }
                        }
                    }

                    // 清理vectorItemNode上面挂载的Sphere节点

                    osg::ref_ptr<osg::Switch> vectorItemNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
                            rootNode, vectorItemNodeName));
                    int numVectorItem = vectorItemNode->getNumChildren();
                    for (int i = 0; i < numVectorItem; ++i) {
                        auto child = dynamic_cast<osg::Switch *>(vectorItemNode->getChild(i));
                        int numItemGeode = child->getNumChildren();
                        for (int j = 0; j < numItemGeode; ++j) {
                            auto deleteSphere = dynamic_cast<osg::Geode *> (child->getChild(j));
                            if (deleteSphere->getName() == "Sphere") {
                                int sphereID = 0;
                                deleteSphere->getUserValue("ID", sphereID);
                                if (deleteUpperLeftCornerPoint) {
                                    if (sphereID == upperLeftCornerPoint.pID) {
                                        child->removeChild(deleteSphere);
                                    }
                                }
                                if (deleteLowerLeftCornerPoint) {
                                    if (sphereID == lowerLeftCornerPoint.pID) {
                                        child->removeChild(deleteSphere);
                                    }
                                }
                                if (deleteUpperRightCornerPoint) {
                                    if (sphereID == upperRightCornerPoint.pID) {
                                        child->removeChild(deleteSphere);
                                    }
                                }
                                if (deleteLowerRightCornerPoint) {
                                    if (sphereID == lowerRightCornerPoint.pID) {
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

                    pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                 pointTextNodeName));

                    int numPointGeode = pointTextNode->getNumChildren();
                    for (int i = 0; i < numPointGeode; ++i) {
                        auto child = dynamic_cast<osg::Geode *>(pointTextNode->getChild(i));
                        if (deleteUpperLeftCornerPoint) {
                            if (child->getName() ==
                                std::string("TSphere_" + std::to_string(upperLeftCornerPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                        }
                        if (deleteLowerLeftCornerPoint) {
                            if (child->getName() ==
                                std::string("TSphere_" + std::to_string(lowerLeftCornerPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                        }
                        if (deleteUpperRightCornerPoint) {
                            if (child->getName() ==
                                std::string("TSphere_" + std::to_string(upperRightCornerPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                        }
                        if (deleteLowerLeftCornerPoint) {
                            if (child->getName() ==
                                std::string("TSphere_" + std::to_string(lowerRightCornerPoint.pID))) {
                                child->removeDrawables(0, child->getNumDrawables());
                                pointTextNode->removeChild(child);
                            }
                        }
                        if (numPointGeode != pointTextNode->getNumChildren()) {
                            numPointGeode = pointTextNode->getNumChildren();
                            i = -1;
                        }
                    }

                    pavementTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                    pavementTextNodeName));
                    osg::ref_ptr<osg::Geode> pavementTextGeode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(
                            rootNode, std::string("TPavement_" + std::to_string(pavement.pID)).c_str()));
                    pavementTextGeode->removeDrawables(0, pavementTextGeode->getNumDrawables());
                    pavementTextNode->removeChild(pavementTextGeode);

                    break;
                }
            }
        }
    }
}

