//
// Created by zhihui on 4/24/19.
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
#include "RoadLinesDeletion.h"

RoadLinesDeletion::RoadLinesDeletion(osg::Switch *rootNode)
        : rootNode(rootNode),
          roadLinesItemNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                         roadLinesItemNodeName))),
          outline(nullptr),
          childNode(nullptr),
          x(0),
          y(0) {}

bool RoadLinesDeletion::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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

void RoadLinesDeletion::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    osg::ref_ptr<osg::Switch> vectorNode(nullptr);
    osg::ref_ptr<osg::Geode> geodeNode;

    osg::ref_ptr<osg::Switch> pointTextNode;
    osg::ref_ptr<osg::Switch> roadLinesTextNode;

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
                if (childNode->getName().compare(0, 10, "RoadLines_") == 0) {
                    size_t deleteRoadLinesID = std::stoi(childNode->getName().substr(10));

                    mdc::RoadLines roadLines =
                            VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::RoadLines>(deleteRoadLinesID));
                    std::vector<std::pair<bool, mdc::Point>> allPoints;

                    for (const auto &id : roadLines.allPointsID) {
                        mdc::Point point;
                        point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(id));
                        allPoints.emplace_back(std::make_pair(false, point));
                    }

                    VectorMapSingleton::getInstance()->remove(roadLines);

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

                    for (auto &point : allPoints) {
                        if (point.second.fromPointLineID.empty() && point.second.toPointLineID.empty()) {
                            VectorMapSingleton::getInstance()->remove(point.second);
                            point.first = true;
                        }
                    }

                    int numItemPavement = roadLinesItemNode->getNumChildren();
                    std::cout << "name: " << roadLinesItemNode->getName() << ", num: " << numItemPavement << std::endl;
                    for (int i = 0; i < numItemPavement; ++i) {
                        auto child = dynamic_cast<osg::Switch *>(roadLinesItemNode->getChild(i));
                        int numItemGeode = child->getNumChildren();
                        for (int j = 0; j < numItemGeode; ++j) {
                            auto deleteSphere = dynamic_cast<osg::Geode *> (child->getChild(j));
                            if (deleteSphere == nullptr) {
                                // 可能有修改时添加的osgFX节点
                                continue;
                            }
                            if (deleteSphere->getName() == "Sphere") {
                                int sphereID = 0;
                                deleteSphere->getUserValue("ID", sphereID);
                                if (std::find_if(allPoints.begin(),
                                                 allPoints.end(),
                                                 [sphereID](const std::pair<bool, mdc::Point> &pair) {
                                                   return pair.second.pID == sphereID;
                                                 }) != allPoints.end()) {
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

                    osg::ref_ptr<osg::Switch>
                            vectorItemNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
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
                                auto it = std::find_if(allPoints.begin(),
                                                       allPoints.end(),
                                                       [sphereID](const std::pair<bool, mdc::Point> &pair) {
                                                         return pair.second.pID == sphereID;
                                                       });
                                if (it != allPoints.end() && it->first) {
                                    child->removeChild(deleteSphere);
                                }
                                if (numItemGeode != child->getNumChildren()) {
                                    numItemGeode = child->getNumChildren();
                                    j = -1;
                                }
                            }
                        }
                    }

                    pointTextNode =
                            dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointTextNodeName));

                    int numPointGeode = pointTextNode->getNumChildren();
                    for (int i = 0; i < numPointGeode; ++i) {
                        auto child = dynamic_cast<osg::Geode *>(pointTextNode->getChild(i));
                        std::string nodeName = child->getName();
                        auto it = std::find_if(allPoints.begin(),
                                               allPoints.end(),
                                               [&nodeName](const std::pair<bool, mdc::Point> &pair) {
                                                 return nodeName
                                                         == std::string("TSphere_" + std::to_string(pair.second.pID));
                                               });
                        if(it != allPoints.end() && it->first) {
                            child->removeDrawables(0, child->getNumDrawables());
                            pointTextNode->removeChild(child);
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
