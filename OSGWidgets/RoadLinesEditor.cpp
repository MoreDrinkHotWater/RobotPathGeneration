//
// Created by zhihui on 4/23/19.
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
#include "RoadLinesEditor.h"
#include "RoadLinesEditorDialog.h"
#include "Color.h"

RoadLinesEditor::RoadLinesEditor(osg::Switch *rootNode) : rootNode(rootNode), tempNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, tempNodeName))), tempLineGeode(nullptr),
                                                          selectedPoints(), curPointIndex(0), x(0), y(0) {}

RoadLinesEditor::~RoadLinesEditor() {
    cleanup();
}

bool RoadLinesEditor::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    auto *view = dynamic_cast<osgViewer::View *>(&aa);
    if (view == nullptr) {
        return false;
    }
    switch (ea.getEventType()) {
        case (osgGA::GUIEventAdapter::KEYDOWN): {
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

void RoadLinesEditor::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
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

        // 道路标记的线不维护其关系
        std::vector<mdc::Point> points;
        int numPoints = selectedPoints.size();

        size_t curRoadLinesIndex = VectorMapSingleton::getInstance()->getMaxRoadLinesIndex() + 1;

        mdc::RoadLines roadLines;
        roadLines.rlID = curRoadLinesIndex++;
        auto *roadLinesEditorDialog = new RoadLinesEditorDialog(&roadLines.type);
        roadLinesEditorDialog->exec();
        if (roadLines.type.empty()) {
            cleanup();
            return;
        }
        for (int i = 0; i < numPoints; ++i) {
            roadLines.allPointsID.push_back(selectedPoints[i].first);
        }

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

        VectorMapSingleton::getInstance()->update(roadLines);
        VectorMapSingleton::getInstance()->printAllRoadLines();

        //
        osg::ref_ptr<osg::Switch> roadLinesItemNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
                rootNode, roadLinesItemNodeName));
        int roadLinesIndex = roadLinesItemNode->getNumChildren();
        std::string type = "roadLines";

        osg::ref_ptr<osg::Switch> roadLinesNode = new osg::Switch;
        roadLinesNode->setName(roadLinesItemName + std::to_string(roadLinesIndex));
        roadLinesNode->setUserValue("itemType", type);
        roadLinesItemNode->addChild(roadLinesNode);

        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        for (const auto &point : points) {
            vertices->push_back(osg::Vec3d(point.ly, point.bx, point.h));
        }

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(roadLinesColor);

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setVertexArray(vertices.get());
        geom->setColorArray(colors.get());
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setName("RoadLines_" + std::to_string(roadLines.rlID));
        geode->addDrawable(geom);

        roadLinesNode->addChild(geode);

        // 画点
        for (const auto &point : points) {
            int localPointIndex = point.pID;
            osg::Vec3d localPoint(point.ly, point.bx, point.h);

            osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
            pointGeode->setName("Sphere");
            pointGeode->setUserValue("ID", localPointIndex);
            pointGeode->setUserValue("pos", localPoint);

            osg::ref_ptr<osg::ShapeDrawable> pointSphere = new osg::ShapeDrawable(new osg::Sphere(localPoint, 0.15f));
            pointSphere->setColor(roadLinesColor);
            pointGeode->addDrawable(pointSphere);
            roadLinesNode->addChild(pointGeode);
        }

        // 显示点ID
        osg::ref_ptr<osg::Switch> pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                               pointTextNodeName));
        for (const auto &point : points) {
            osg::Vec3d pos(point.ly, point.bx, point.h);
            std::string name = "TSphere_" + std::to_string(point.pID);

            osg::ref_ptr<osg::Geode> pointTextGeode = new osg::Geode;
            pointTextGeode->setName(name);

            osg::ref_ptr<osgText::Text> text = new osgText::Text;
            text->setCharacterSize(0.5);
            text->setAxisAlignment(osgText::TextBase::XY_PLANE);
            text->setPosition(pos);
            text->setText(std::to_string(point.pID));
            text->setColor(roadLinesColor);

            pointTextGeode->addDrawable(text);

            pointTextNode->addChild(pointTextGeode);
        }

        cleanup();
    }
}

void RoadLinesEditor::updateIndex() {
    curPointIndex = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;
}

void RoadLinesEditor::rollback() {
    if (!selectedPoints.empty()) {
        selectedPoints.pop_back();
    }
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
    }
    // 删除已画的点和线
    tempNode->removeChildren(tempNode->getNumChildren() - 2, 2);
}

void RoadLinesEditor::cleanup() {
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
        tempLineGeode = nullptr;
    }
    tempNode->removeChildren(0, tempNode->getNumChildren());

    selectedPoints.clear();
}