//
// Created by zhihui on 4/8/19.
//

#include <iostream>
#include <iomanip>

#include <osg/Geode>
#include <osg/Switch>
#include <osgViewer/View>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgText/Text>

#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VectorMapSingleton.h"
#include "LineModification.h"

LineModification::LineModification(osg::Switch *rootNode) : rootNode(rootNode), vectorItemNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, vectorItemNodeName))), pointCloudNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointCloudNodeName))),
                                                            tempModLineGeode(nullptr),
                                                            selectedPoint(std::make_pair(0, osg::Vec3d())),
                                                            involvedPoints(), redrawLines(), count(1), deletedPoint(),
                                                            x(0), y(0) {}

LineModification::~LineModification() {
    cleanup();
}

bool LineModification::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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
        case (osgGA::GUIEventAdapter::MOVE) : {

            x = ea.getX();
            y = ea.getY();

            if (selectedPoint.first == 0) {
                return false;
            }

            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);

            // 只能移动到点云上面非以画点的点上
            rootNode->setSingleChildOn(rootNode->getChildIndex(pointCloudNode));
            view->getCamera()->accept(iv);
            rootNode->setAllChildrenOn();

            if (picker->containsIntersections()) {
                osg::Vec3d curPoint;
                bool isIntersect = false;

                auto intersections = picker->getIntersections();
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        curPoint = intersection.localIntersectionPoint;
                        isIntersect = true;
                        break;
                    }
                }

                if (!isIntersect) {
                    return false;
                }

                if (tempModLineGeode == nullptr) {
                    tempModLineGeode = new osg::Geode;
                    tempModLineGeode->setName("tempModLineGeode");
                    vectorItemNode->addChild(tempModLineGeode);
                } else {
                    tempModLineGeode->removeDrawables(0, tempModLineGeode->getNumDrawables());
                }

                for (const auto &point : involvedPoints) {
                    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                    vertices->push_back(std::get<1>(point));
                    vertices->push_back(curPoint);

                    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                    geom->setName("tempModLineGeom");
                    tempModLineGeode->addDrawable(geom);

                    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertices->size()));
                    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                    colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

                    geom->setColorArray(colors.get());
                    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                    geom->setVertexArray(vertices.get());
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

void LineModification::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {

    osg::ref_ptr<osg::Switch> vectorNode;
    osg::ref_ptr<osg::Geode> geodeNode;

    osg::ref_ptr<osg::Switch> lineTextNode;
    osg::ref_ptr<osg::Switch> pointTextNode;

    if (selectedPoint.first == 0) {
        count = 1;
        involvedPoints.clear();
    }
    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        if (count == 1) {
            // 第一次点击是选需要修改的点
            redrawLines.clear();
            deletedPoint.clear();
            if (picker->containsIntersections()) {
                osg::Vec3d localPoint;
                int localPointIndex = 0;

                auto intersections = picker->getIntersections();
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "Sphere") {
                        osg::ref_ptr<osg::Switch> first = dynamic_cast<osg::Switch *>(childNode->getParent(0));
                        osg::ref_ptr<osg::Geode> second = dynamic_cast<osg::Geode *> (childNode);
                        deletedPoint.insert(std::make_pair(first, second));
                    }
                }
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "Sphere") {
                        childNode->getUserValue("pos", localPoint);
                        childNode->getUserValue("ID", localPointIndex);
                        selectedPoint = std::make_pair(localPointIndex, localPoint);
                        mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                                mdc::Key<mdc::Point>(localPointIndex));
                        mdc::Line line;
                        for (const auto &ID : point.fromPointLineID) {
                            redrawLines.push_back(ID);
                            line = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Line>(ID));
                            mdc::Point ePoint = VectorMapSingleton::getInstance()->findByID(
                                    mdc::Key<mdc::Point>(line.ePID));
                            involvedPoints.emplace_back(
                                    std::make_pair(ePoint.pID, osg::Vec3d(ePoint.ly, ePoint.bx, ePoint.h)));
                        }
                        for (const auto &ID : point.toPointLineID) {
                            redrawLines.push_back(ID);
                            line = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Line>(ID));
                            mdc::Point sPoint = VectorMapSingleton::getInstance()->findByID(
                                    mdc::Key<mdc::Point>(line.sPID));
                            involvedPoints.emplace_back(
                                    std::make_pair(sPoint.pID, osg::Vec3d(sPoint.ly, sPoint.bx, sPoint.h)));
                        }
                        count++;
                        break;
                    }
                }
            }
        } else if (count == 2) {
            // 第二次点击是选新的点
            if (picker->containsIntersections()) {
                osg::Vec3d localPoint;
                bool isIntersect = false;

                auto intersections = picker->getIntersections();
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        localPoint = intersection.localIntersectionPoint;
                        mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                                mdc::Key<mdc::Point>(selectedPoint.first));
                        point.ly = localPoint.x();
                        point.bx = localPoint.y();
                        point.h = localPoint.z();
                        VectorMapSingleton::getInstance()->update(point);
                        isIntersect = true;
                        break;
                    }
                }

                if (isIntersect) {
                    std::string itemVectorName;
                    for (const auto &ID : redrawLines) {
                        geodeNode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(rootNode, std::string(
                                "Line_" + std::to_string(ID)).c_str()));
                        std::cout << "geodeNode: " << geodeNode->getName() << std::endl;
                        vectorNode = dynamic_cast<osg::Switch *>(geodeNode->getParent(0));
                        vectorNode->removeChild(geodeNode);

                        auto line = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Line>(ID));
                        auto sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(line.sPID));
                        auto ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(line.ePID));

                        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                        vertices->push_back(osg::Vec3d(sPoint.ly, sPoint.bx, sPoint.h));
                        vertices->push_back(osg::Vec3d(ePoint.ly, ePoint.bx, ePoint.h));

                        osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                        colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

                        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                        geom->setVertexArray(vertices.get());
                        geom->setColorArray(colors.get());
                        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

                        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                        geode->setName("Line_" + std::to_string(ID));
                        geode->addDrawable(geom);

                        vectorNode->addChild(geode);

                        lineTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                    lineTextNodeName));
                        geodeNode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(rootNode, std::string(
                                "TLine_" + std::to_string(ID)).c_str()));

                        lineTextNode->removeChild(geodeNode);
                        osg::Vec3d pos1(sPoint.ly, sPoint.bx, sPoint.h);
                        osg::Vec3d pos2(ePoint.ly, ePoint.bx, ePoint.h);
                        osg::Vec3d pos = (pos1 + pos2) / 2;
                        std::string name = "TLine_" + std::to_string(line.lID);
                        osg::Vec4 color(1.0, 0.0, 0.0, 0.5);

                        osg::ref_ptr<osg::Geode> lineTextGeode = new osg::Geode;
                        lineTextGeode->setName(name);

                        osg::ref_ptr<osgText::Text> text = new osgText::Text;
                        text->setCharacterSize(0.5);
                        text->setAxisAlignment(osgText::TextBase::XY_PLANE);
                        text->setPosition(pos);
                        text->setText(std::to_string(line.lID));
                        text->setColor(color);

                        lineTextGeode->addDrawable(text);

                        lineTextNode->addChild(lineTextGeode);

                        if (itemVectorName != vectorNode->getName()) {
                            mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                                    mdc::Key<mdc::Point>(selectedPoint.first));
                            osg::Vec3d newPoint(osg::Vec3d(point.ly, point.bx, point.h));
                            osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
                            pointGeode->setName("Sphere");
                            pointGeode->setUserValue("ID", static_cast<int>(point.pID));
                            pointGeode->setUserValue("pos", newPoint);

                            osg::ref_ptr<osg::ShapeDrawable> pointSphere = new osg::ShapeDrawable(
                                    new osg::Sphere(newPoint, 0.15f));
                            pointGeode->addDrawable(pointSphere);
                            vectorNode->addChild(pointGeode);

                            pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                         pointTextNodeName));
                            geodeNode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                    std::string(
                                                                                                            "TSphere_" +
                                                                                                            std::to_string(
                                                                                                                    point.pID)).c_str()));
                            pointTextNode->removeChild(geodeNode);

                            name = "TSphere_" + std::to_string(point.pID);
                            color = osg::Vec4(0.0, 1.0, 0.0, 0.5);

                            osg::ref_ptr<osg::Geode> pointTextGeode = new osg::Geode;
                            pointTextGeode->setName(name);

                            text = new osgText::Text;
                            text->setCharacterSize(0.5);
                            text->setAxisAlignment(osgText::TextBase::XY_PLANE);
                            text->setPosition(newPoint);
                            text->setText(std::to_string(point.pID));
                            text->setColor(color);

                            pointTextGeode->addDrawable(text);

                            pointTextNode->addChild(pointTextGeode);

                        }
                        itemVectorName = vectorNode->getName();
                    }
                    for (const auto & point : deletedPoint) {
                        point.first->removeChild(point.second);
                    }
                    selectedPoint.first = 0;
                    selectedPoint.second = osg::Vec3d();
                    involvedPoints.clear();
                    if (tempModLineGeode != nullptr) {
                        tempModLineGeode->removeDrawables(0, tempModLineGeode->getNumDrawables());
                    }
                    vectorItemNode->removeChild(tempModLineGeode);
                    tempModLineGeode = nullptr;
                }
            }
        }
    }
}

void LineModification::cleanup() {
    if (tempModLineGeode != nullptr) {
        tempModLineGeode->removeDrawables(0, tempModLineGeode->getNumDrawables());
    }
    vectorItemNode->removeChild(tempModLineGeode);
    tempModLineGeode = nullptr;
    selectedPoint.first = 0;
    selectedPoint.second = osg::Vec3d();
    involvedPoints.clear();
}