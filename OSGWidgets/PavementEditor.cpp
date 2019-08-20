//
// Created by zhihui on 4/16/19.
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
#include "PavementEditor.h"
#include "Color.h"

PavementEditor::PavementEditor(osg::Switch *rootNode) : rootNode(rootNode), tempNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, tempNodeName))), tempLineGeode(nullptr),
                                                        selectedPoints(), curPointIndex(0), x(0), y(0) {}

PavementEditor::~PavementEditor() {
    cleanup();
}

bool PavementEditor::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case (osgGA::GUIEventAdapter::KEYDOWN) : {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape) {
                rollback();
                return true;
            }
            return false;
        }
        case (osgGA::GUIEventAdapter::MOVE): {
            x = ea.getX();
            y = ea.getY();

            if (selectedPoints.empty()) {
                return false;
            }

            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);

            if (picker->containsIntersections()) {
                osg::Vec3d curPoint;

                bool isIntersect = false;
                auto intersections = picker->getIntersections();
                for (const auto &intersection: intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        curPoint = intersection.localIntersectionPoint;
                        isIntersect = true;
                        break;
                    }
                    if (childNode->getName() == "Sphere") {
                        childNode->getUserValue("pos", curPoint);
                        isIntersect = true;
                        break;
                    }
                }

                if (!isIntersect) {
                    return false;
                }

                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(std::get<1>(selectedPoints.back()));
                vertices->push_back(curPoint);

                if (tempLineGeode == nullptr) {
                    tempLineGeode = new osg::Geode;
                    tempLineGeode->setName("tempLineGeode");
                    tempNode->addChild(tempLineGeode);
                } else {
                    tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
                }

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setName("tempLineGeom");
                tempLineGeode->addDrawable(geom);

                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertices->size()));
                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->setVertexArray(vertices.get());
            }
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

void PavementEditor::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    if (selectedPoints.empty()) {
        updateIndex();
        cleanup();
    }
    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

        double w = 1.5, h = 1.5;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
        osgUtil::IntersectionVisitor iv(picker);
        view->getCamera()->accept(iv);

        if (picker->containsIntersections()) {
            osg::Vec3d localPoint;
            int localPointIndex = 0;

            auto intersections = picker->getIntersections();
            for (const auto &intersection : intersections) {
                auto childNode = intersection.nodePath.back();
                if (childNode->getName() == "CloudPoints") {
                    localPoint = intersection.localIntersectionPoint;
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }
                    localPointIndex = curPointIndex++;

                    osg::ref_ptr<osg::Geode> nodeGeode = new osg::Geode;
                    nodeGeode->setName("Sphere");
                    nodeGeode->setUserValue("pos", localPoint);
                    nodeGeode->setUserValue("ID", localPointIndex);

                    osg::ref_ptr<osg::ShapeDrawable> nodeSphere = new osg::ShapeDrawable(
                            new osg::Sphere(localPoint, 0.1f));
                    nodeSphere->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
                    nodeGeode->addDrawable(nodeSphere.get());
                    tempNode->addChild(nodeGeode.get());

                    break;
                }
                if (childNode->getName() == "Sphere") {
                    childNode->getUserValue("pos", localPoint);
                    childNode->getUserValue("ID", localPointIndex);
                    if (!selectedPoints.empty()) {
                        if (std::get<1>(selectedPoints.back()) == localPoint) {
                            return;
                        }
                    }
                    // 方便回退
                    osg::ref_ptr<osg::Geode> nodeGeode = new osg::Geode;
                    nodeGeode->setName("Sphere");
                    nodeGeode->setUserValue("pos", localPoint);
                    nodeGeode->setUserValue("ID", localPointIndex);
                    osg::ref_ptr<osg::ShapeDrawable> nodeSphere = new osg::ShapeDrawable(
                            new osg::Sphere(localPoint, 0.1f));
                    nodeSphere->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
                    nodeGeode->addDrawable(nodeSphere.get());
                    tempNode->addChild(nodeGeode.get());
                    break;
                }
            }
            // 所选点可能在临时的线上，localPoint是默认的0值
            if (localPointIndex == 0) {
                return;
            }
            std::cout << "------------Debug-----------------" << std::endl;
            std::cout << "localPointIndex: " << localPointIndex << " --- localPoint: " << localPoint << std::endl;
            selectedPoints.emplace_back(std::make_pair(localPointIndex, localPoint));
            if (selectedPoints.size() >= 2) {
                osg::Vec3d penultimatePoint = std::get<1>(*(selectedPoints.end() - 2));
                osg::Vec3d lastPoint = std::get<1>(selectedPoints.back());

                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(penultimatePoint);
                vertices->push_back(lastPoint);

                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 1.0, 1.0));

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setName("solidLineGeom");
                geom->setVertexArray(vertices.get());
                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->setName("solidLineGeode");
                geode->addDrawable(geom);

                tempNode->addChild(geode);
            }
        }
    }

    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && selectedPoints.size() >= 2) {

        // 人行道的线不维护其关系
        std::vector<mdc::Point> points;
        int numPoints = selectedPoints.size();

        size_t curPavementIndex = VectorMapSingleton::getInstance()->getMaxPavementIndex() + 1;

        for (int i = 0; i < numPoints; ++i) {
            size_t index = selectedPoints[i].first;
            osg::Vec3d pos = selectedPoints[i].second;
            mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                    mdc::Key<mdc::Point>(selectedPoints[i].first));
            point.pID = index;
            point.bx = pos.y();
            point.ly = pos.x();
            point.h = pos.z();
            points.push_back(point);
            VectorMapSingleton::getInstance()->update(point);
        }
        std::cout << "------------Debug-----------------" << std::endl;
        std::cout << "update point: " << std::endl;
        VectorMapSingleton::getInstance()->printAllPoints();

        mdc::Pavement pavement;
        pavement.pID = curPavementIndex++;
        pavement.upperLeftCorner = points[0].pID;
        pavement.lowerLeftCorner = points[1].pID;
        pavement.upperRightCorner = points[2].pID;
        pavement.lowerRightCorner = points[3].pID;

        VectorMapSingleton::getInstance()->update(pavement);
        VectorMapSingleton::getInstance()->printAllPavements();
        //
        osg::ref_ptr<osg::Switch> pavementItemNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
                rootNode, pavementItemNodeName));
        int pavementIndex = pavementItemNode->getNumChildren();
        std::string type = "pavement";

        osg::ref_ptr<osg::Switch> pavementNode = new osg::Switch;
        pavementNode->setName(pavementItemName + std::to_string(pavementIndex));
        pavementNode->setUserValue("itemType", type);
        pavementItemNode->addChild(pavementNode);

        // 人行道的线是逆时针链接
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        for (const auto &point : points) {
            vertices->push_back(osg::Vec3d(point.ly, point.bx, point.h));
        }

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(pavementColor);

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setVertexArray(vertices.get());
        geom->setColorArray(colors.get());
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setName("Pavement_" + std::to_string(pavement.pID));
        geode->addDrawable(geom);

        pavementNode->addChild(geode);

        // 画点
        for (const auto &point : points) {
            int localPointIndex = point.pID;
            osg::Vec3d localPoint(point.ly, point.bx, point.h);

            osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
            pointGeode->setName("Sphere");
            pointGeode->setUserValue("ID", localPointIndex);
            pointGeode->setUserValue("pos", localPoint);

            osg::ref_ptr<osg::ShapeDrawable> pointSphere = new osg::ShapeDrawable(new osg::Sphere(localPoint, 0.15f));
            pointSphere->setColor(pavementColor);
            pointGeode->addDrawable(pointSphere);
            pavementNode->addChild(pointGeode);
        }

        // 显示点ID
        osg::ref_ptr<osg::Switch> pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                               pointTextNodeName));
        for (const auto &point : points) {
            osg::Vec3d pos(point.ly, point.bx, point.h);
            std::string name = "TSphere_" + std::to_string(point.pID);
            osg::Vec4 color(0.0, 1.0, 0.0, 0.5);

            osg::ref_ptr<osg::Geode> pointTextGeode = new osg::Geode;
            pointTextGeode->setName(name);

            osg::ref_ptr<osgText::Text> text = new osgText::Text;
            text->setCharacterSize(0.5);
            text->setAxisAlignment(osgText::TextBase::XY_PLANE);
            text->setPosition(pos);
            text->setText(std::to_string(point.pID));
            text->setColor(color);

            pointTextGeode->addDrawable(text);

            pointTextNode->addChild(pointTextGeode);
        }

        // 显示人行道ID
        osg::ref_ptr<osg::Switch> pavementTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
                rootNode, pavementTextNodeName));
        osg::Vec3d pos1(points[0].ly, points[0].bx, points[0].h);
        osg::Vec3d pos2(points[1].ly, points[1].bx, points[1].h);
        osg::Vec3d pos3(points[2].ly, points[2].bx, points[2].h);
        osg::Vec3d pos4(points[3].ly, points[3].bx, points[3].h);
        osg::Vec3d pos = ((pos1 + pos2) / 2 + (pos3 + pos4) / 2) / 2;
        std::string name = "TPavement_" + std::to_string(pavement.pID);

        osg::ref_ptr<osg::Geode> pavementTextGeode = new osg::Geode;
        pavementTextGeode->setName(name);

        osg::ref_ptr<osgText::Text> text = new osgText::Text;
        text->setCharacterSize(0.8);
        text->setAxisAlignment(osgText::TextBase::XY_PLANE);
        text->setPosition(pos);
        text->setText(std::to_string(pavement.pID));
        text->setColor(pavementColor);

        pavementTextGeode->addDrawable(text);

        pavementTextNode->addChild(pavementTextGeode);

        cleanup();
    }
}

void PavementEditor::updateIndex() {
    curPointIndex = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;
}

void PavementEditor::rollback() {
    if (!selectedPoints.empty()) {
        selectedPoints.pop_back();
    }
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
    }
    // 删除已画的点和线
    tempNode->removeChildren(tempNode->getNumChildren() - 2, 2);
}

void PavementEditor::cleanup() {
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
        tempLineGeode = nullptr;
    }
    tempNode->removeChildren(0, tempNode->getNumChildren());

    selectedPoints.clear();
}