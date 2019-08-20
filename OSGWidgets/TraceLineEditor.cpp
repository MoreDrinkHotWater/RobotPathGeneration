//
// Created by zhihui on 4/28/19.
//
#include <iostream>
#include <iomanip>

#include <osg/Geode>
#include <osg/Switch>
#include <osgViewer/View>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osgText/Text>

#include "TraceLineEditor.h"
#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "Color.h"
#include "VectorMapSingleton.h"

TraceLineEditor::TraceLineEditor(osg::Switch *rootNode)
        : rootNode(rootNode),
          tempNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, tempNodeName))),
          virtualPlaneNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                        virtualPlaneNodeName))),
          tempLineGeode(nullptr),
          curPointIndex(0),
          x(0),
          y(0) {}

TraceLineEditor::~TraceLineEditor() {
    cleanup();
}

bool TraceLineEditor::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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
            // 先判断当前选点是否是已有的点或者点云上的点
            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);
            bool hasAlready = false;
            osg::Vec3d curPoint;
            if (picker->containsIntersections()) {

                auto intersections = picker->getIntersections();
                for (const auto &intersection: intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        curPoint = intersection.localIntersectionPoint;
                        hasAlready = true;
                        break;
                    }
                    if (childNode->getName() == "Sphere") {
                        childNode->getUserValue("pos", curPoint);
                        hasAlready = true;
                        break;
                    }
                }
            }

            if (!hasAlready) {
                // 生成虚拟平面用于选点
                double vpW = 80, vpH = 80;
                osg::ref_ptr<osgUtil::PolytopeIntersector> vpPicker =
                        new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                                         x - vpW,
                                                         y - vpH,
                                                         x + vpW,
                                                         y + vpH);
                osgUtil::IntersectionVisitor vpIv(vpPicker);
                view->getCamera()->accept(vpIv);

                osg::ref_ptr<osg::Switch> geom = new osg::Switch;
                // 平面四边形顶点
                osg::ref_ptr<osg::Vec3Array> quVertices = new osg::Vec3Array;
                osg::ref_ptr<osg::Geode> quGeode = new osg::Geode;
                osg::ref_ptr<osg::Geometry> quGeom = new osg::Geometry;
                osg::ref_ptr<osg::Vec4Array> quColors = new osg::Vec4Array;

                if (vpPicker->containsIntersections()) {
                    std::vector<osg::Vec3d> points;
                    double approximateZ;
                    auto intersections = vpPicker->getIntersections();
                    for (const auto &intersection : intersections) {
                        auto childNode = intersection.nodePath.back();
                        if (childNode->getName() == "CloudPoints") {
                            points.push_back(intersection.localIntersectionPoint);
                        }
                    }
                    // 一个点不好画平面
                    if (points.size() < 2) {
                        return false;
                    }
                    std::cout << "------------Debug-----------------" << std::endl;
                    std::cout << "points'size: " << points.size() << std::endl;
                    approximateZ = calculateApproximateZ(points);
                    std::cout << "approximateZ: " << approximateZ << std::endl;
//                    for (const auto point : points) {
//                        std::cout << "x: " << point.x() << ", y: " << point.y() << ", z: " << point.z()
//                                  << std::endl;
//                    }
                    std::cout << "------------Debug-----------------" << std::endl;

                    // 原始点X-Y坐标值
                    std::vector<std::pair<double, double>> vertices;
                    for (const auto &point : points) {
                        vertices.emplace_back(std::make_pair(point.x(), point.y()));
                    }
//                    for (const std::pair<double, double> pair : vertices) {
//                        std::cout << "(" << pair.first << ", " << pair.second << ")" << std::endl;
//                    }

                    double minX, maxX;
                    double minY, maxY;
                    minX = maxX = vertices[0].first;
                    minY = maxY = vertices[0].second;

                    for (const std::pair<double, double> pair : vertices) {
                        if (pair.first < minX) {
                            minX = pair.first;
                        }
                        if (pair.first > maxX) {
                            maxX = pair.first;
                        }
                        if (pair.second < minY) {
                            minY = pair.second;
                        }
                        if (pair.second > maxY) {
                            maxY = pair.second;
                        }
                    }

                    std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY
                              << std::endl;

                    quVertices->push_back(osg::Vec3(minX, minY, approximateZ));
                    quVertices->push_back(osg::Vec3(minX, maxY, approximateZ));
                    quVertices->push_back(osg::Vec3(maxX, maxY, approximateZ));
                    quVertices->push_back(osg::Vec3(maxX, minY, approximateZ));

                    quColors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 0.1f));

                    quGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
                    quGeom->setVertexArray(quVertices);
                    quGeom->setColorArray(quColors);
                    quGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
                    quGeode->addDrawable(quGeom);

                    // 设置透明度
                    osg::ref_ptr<osg::StateSet> stateset = quGeode->getOrCreateStateSet();
                    stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
                    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                    geom->addChild(quGeode);
                    virtualPlaneNode->addChild(geom);

                    // 当前鼠标所选的点
                    double w = 1.5, h = 1.5;
                    osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                            osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
                    osgUtil::IntersectionVisitor iv(picker);
                    view->getCamera()->accept(iv);
                    if (picker->containsIntersections()) {
                        auto intersections = picker->getIntersections();
                        std::cout << "intersections: " << intersections.size() << std::endl;
                        for (const auto &intersection: intersections) {
                            auto childNode = intersection.nodePath.back();
                            curPoint = intersection.localIntersectionPoint;
                            std::cout << "------------Debug-----------------" << std::endl;
                            std::cout << "curPoint: " << curPoint << std::endl;
                            std::cout << "------------Debug-----------------" << std::endl;
                            break;
                        }
                        curPoint.z() = approximateZ;
                    } else {
                        virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
                        return false;
                    }
                } else {
                    return false;
                }
            }
//                std::cout << "------------Debug-----------------" << std::endl;
//                auto iter = picker->getIntersections().begin();
//                for (; iter != picker->getIntersections().end(); ++iter) {
//                    std::cout << "----nodePath----" << std::endl;
//                    for (const auto &node:iter->nodePath) {
//                        std::cout << node->className() << ", " << node->libraryName() << ", " << node->getName()
//                                  << std::endl;
//                    }
//                }
//                std::cout << "------------Debug-----------------" << std::endl;

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

            virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());

            return true;
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

void TraceLineEditor::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
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
        bool hasAlready = false;
        osg::Vec3d localPoint;
        int localPointIndex = 0;
        if (picker->containsIntersections()) {

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
                    hasAlready = true;
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
                    hasAlready = true;
                    break;
                }
            }
        }
        if (!hasAlready) {
            // 生成虚拟平面用于选点
            double vpW = 80, vpH = 80;
            osg::ref_ptr<osgUtil::PolytopeIntersector> vpPicker =
                    new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                                     x - vpW,
                                                     y - vpH,
                                                     x + vpW,
                                                     y + vpH);
            osgUtil::IntersectionVisitor vpIv(vpPicker);
            view->getCamera()->accept(vpIv);

            osg::ref_ptr<osg::Switch> geom = new osg::Switch;
            // 平面四边形顶点
            osg::ref_ptr<osg::Vec3Array> quVertices = new osg::Vec3Array;
            osg::ref_ptr<osg::Geode> quGeode = new osg::Geode;
            osg::ref_ptr<osg::Geometry> quGeom = new osg::Geometry;
            osg::ref_ptr<osg::Vec4Array> quColors = new osg::Vec4Array;

            if (vpPicker->containsIntersections()) {
                std::vector<osg::Vec3d> points;
                double approximateZ;
                auto intersections = vpPicker->getIntersections();
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "CloudPoints") {
                        points.push_back(intersection.localIntersectionPoint);
                    }
                }
                // 一个点不好画平面
                if (points.size() < 2) {
                    return;
                }
                std::cout << "------------Debug-----------------" << std::endl;
                std::cout << "points'size: " << points.size() << std::endl;
                approximateZ = calculateApproximateZ(points);
                std::cout << "approximateZ: " << approximateZ << std::endl;
//                    for (const auto point : points) {
//                        std::cout << "x: " << point.x() << ", y: " << point.y() << ", z: " << point.z()
//                                  << std::endl;
//                    }
                std::cout << "------------Debug-----------------" << std::endl;

                // 原始点X-Y坐标值
                std::vector<std::pair<double, double>> vertices;
                for (const auto &point : points) {
                    vertices.emplace_back(std::make_pair(point.x(), point.y()));
                }
//                    for (const std::pair<double, double> pair : vertices) {
//                        std::cout << "(" << pair.first << ", " << pair.second << ")" << std::endl;
//                    }

                double minX, maxX;
                double minY, maxY;
                minX = maxX = vertices[0].first;
                minY = maxY = vertices[0].second;

                for (const std::pair<double, double> pair : vertices) {
                    if (pair.first < minX) {
                        minX = pair.first;
                    }
                    if (pair.first > maxX) {
                        maxX = pair.first;
                    }
                    if (pair.second < minY) {
                        minY = pair.second;
                    }
                    if (pair.second > maxY) {
                        maxY = pair.second;
                    }
                }

                std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY
                          << std::endl;

                quVertices->push_back(osg::Vec3(minX, minY, approximateZ));
                quVertices->push_back(osg::Vec3(minX, maxY, approximateZ));
                quVertices->push_back(osg::Vec3(maxX, maxY, approximateZ));
                quVertices->push_back(osg::Vec3(maxX, minY, approximateZ));

                quColors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 0.1f));

                quGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
                quGeom->setVertexArray(quVertices);
                quGeom->setColorArray(quColors);
                quGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
                quGeode->addDrawable(quGeom);

                // 设置透明度
                osg::ref_ptr<osg::StateSet> stateset = quGeode->getOrCreateStateSet();
                stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
                stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                geom->addChild(quGeode);
                virtualPlaneNode->addChild(geom);

                // 当前鼠标所选的点
                double w = 1.5, h = 1.5;
                osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                        osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
                osgUtil::IntersectionVisitor iv(picker);
                view->getCamera()->accept(iv);
                if (picker->containsIntersections()) {
                    auto intersections = picker->getIntersections();
                    std::cout << "intersections: " << intersections.size() << std::endl;
                    for (const auto &intersection: intersections) {
                        auto childNode = intersection.nodePath.back();
                        localPoint = intersection.localIntersectionPoint;
                        std::cout << "------------Debug-----------------" << std::endl;
                        std::cout << "curPoint: " << localPoint << std::endl;
                        std::cout << "------------Debug-----------------" << std::endl;
                        break;
                    }
                    localPoint.z() = approximateZ;
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
                } else {
                    virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
                    return;
                }
            } else {
                return;
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
        virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
    }

    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON && selectedPoints.size() >= 2) {

        // 更新新画线的基本字段
        std::vector<mdc::Point> points;
        std::vector<mdc::TraceLine> traceLines;
        std::set<size_t> newTraceLineID;
        std::set<size_t>::iterator itNewTraceLineID;
        size_t curTraceLineIndex = VectorMapSingleton::getInstance()->getMaxTraceLinesIndex() + 1;
        int numLines = selectedPoints.size() - 1;
        int numPoints = selectedPoints.size();
        for (int i = 0; i < numLines; ++i) {
            mdc::TraceLine traceLine;
            traceLine.tlID = curTraceLineIndex++;
            traceLine.sPID = std::get<0>(selectedPoints[i]);
            traceLine.ePID = std::get<0>(selectedPoints[i + 1]);
            traceLines.push_back(traceLine);
            newTraceLineID.insert(traceLine.tlID);
        }

        // 更新所涉及点的全部字段
        for (int i = 0; i < numPoints; ++i) {
            size_t index = selectedPoints[i].first;
            osg::Vec3d pos = selectedPoints[i].second;
            mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                    mdc::Key<mdc::Point>(selectedPoints[i].first));
            point.pID = index;
            point.bx = pos.y();
            point.ly = pos.x();
            point.h = pos.z();
            if (i == 0) {
                point.fromPointLineID.push_back(traceLines[i].tlID);
            } else if (i == numPoints - 1) {
                point.toPointLineID.push_back(traceLines[i - 1].tlID);
            } else {
                point.fromPointLineID.push_back(traceLines[i].tlID);
                point.toPointLineID.push_back(traceLines[i - 1].tlID);
            }
            points.push_back(point);
            VectorMapSingleton::getInstance()->update(point);
        }
        std::cout << "------------Debug-----------------" << std::endl;
        std::cout << "update point: " << std::endl;
        VectorMapSingleton::getInstance()->printAllPoints();

        // 更新新画线的其他字段
        for (int i = 0; i < numLines; ++i) {
            mdc::Point point;
            point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLines[i].sPID));
            for (auto lineID : point.toPointLineID) {
                traceLines[i].toThisLineID.push_back(lineID);
            }
            point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLines[i].ePID));
            for (auto lineID : point.fromPointLineID) {
                traceLines[i].fromThisLineID.push_back(lineID);
            }
        }
        VectorMapSingleton::getInstance()->update(traceLines);
        std::cout << "------------Debug-----------------" << std::endl;
        std::cout << "update traceLines: " << std::endl;
        VectorMapSingleton::getInstance()->printAllTraceLines();

        // 更新所涉及线（非新画线）的全部字段
        std::set<size_t> tempSet;
        std::set<size_t>::iterator it;
        for (int i = 0; i < numPoints; ++i) {
            mdc::TraceLine traceLine;
            mdc::Point point;
            if (i == 0) {
                point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(points[i].pID));
                for (auto lineID : point.toPointLineID) {
                    traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(lineID));
                    itNewTraceLineID = newTraceLineID.find(traceLine.tlID);
                    if (itNewTraceLineID != newTraceLineID.end()) {
                        continue;
                    }
                    traceLine.fromThisLineID.push_back(traceLines[i].tlID);
                    VectorMapSingleton::getInstance()->update(traceLine);
                }
            } else if (i == numPoints - 1) {
                it = tempSet.find(points[i].pID);
                if (it == tempSet.end()) {
                    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(points[i].pID));
                    for (auto lineID : point.fromPointLineID) {
                        traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(lineID));
                        itNewTraceLineID = newTraceLineID.find(traceLine.tlID);
                        if (itNewTraceLineID != newTraceLineID.end()) {
                            continue;
                        }
                        traceLine.toThisLineID.push_back(traceLines[i - 1].tlID);
                        VectorMapSingleton::getInstance()->update(traceLine);
                    }
                }
            } else {
                it = tempSet.find(points[i].pID);
                if (it == tempSet.end()) {
                    point = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(points[i].pID));
                    for (auto lineID : point.toPointLineID) {
                        traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(lineID));
                        itNewTraceLineID = newTraceLineID.find(traceLine.tlID);
                        if (itNewTraceLineID != newTraceLineID.end()) {
                            continue;
                        }
                        traceLine.fromThisLineID.push_back(traceLines[i].tlID);
                        VectorMapSingleton::getInstance()->update(traceLine);
                    }
                    for (auto lineID : point.fromPointLineID) {
                        traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(lineID));
                        itNewTraceLineID = newTraceLineID.find(traceLine.tlID);
                        if (itNewTraceLineID != newTraceLineID.end()) {
                            continue;
                        }
                        traceLine.toThisLineID.push_back(traceLines[i - 1].tlID);
                        VectorMapSingleton::getInstance()->update(traceLine);
                    }
                }
            }
            tempSet.insert(points[i].pID);
        }
        std::cout << "------------Debug-----------------" << std::endl;
        VectorMapSingleton::getInstance()->printAllTraceLines();

        //
        osg::ref_ptr<osg::Switch> traceItemNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(
                rootNode, traceItemNodeName));
        int vectorIndex = traceItemNode->getNumChildren();
        int startTraceLineID = traceLines[0].tlID;
        int endTraceLineID = traceLines.back().tlID;
        std::string type = "TraceLine";

        osg::ref_ptr<osg::Switch> traceNode = new osg::Switch;
        traceNode->setName(traceLineItemName + std::to_string(vectorIndex));
        traceNode->setUserValue("startID", startTraceLineID);
        traceNode->setUserValue("endID", endTraceLineID);
        traceNode->setUserValue("itemType", type);
        traceItemNode->addChild(traceNode);

        // 画线（逐条画线，方便删除）
        for (const auto &traceLine : traceLines) {
            auto sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.sPID));
            auto ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.ePID));

            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
            vertices->push_back(osg::Vec3d(sPoint.ly, sPoint.bx, sPoint.h));
            vertices->push_back(osg::Vec3d(ePoint.ly, ePoint.bx, ePoint.h));

            osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
            colors->push_back(traceLinesColor);

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setVertexArray(vertices.get());
            geom->setColorArray(colors.get());
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->setName("TraceLine_" + std::to_string(traceLine.tlID));
            geode->addDrawable(geom);

            traceNode->addChild(geode);
        }
//        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
//        for (const auto &point : points) {
//            vertices->push_back(osg::Vec3d(point.ly, point.bx, point.h));
//        }
//
//        osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
//        colors->push_back(osg::Vec3(1.0, 1.0, 1.0));
//
//        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
//        geom->setVertexArray(vertices.get());
//        geom->setColorArray(colors.get());
//        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
//        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
//
//        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
//        geode->setName("Line");
//        geode->addDrawable(geom);
//
//        traceNode->addChild(geode);

        // 画点
        for (const auto &point : points) {
            int localPointIndex = point.pID;
            osg::Vec3d localPoint(point.ly, point.bx, point.h);

            osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
            pointGeode->setName("Sphere");
            pointGeode->setUserValue("ID", localPointIndex);
            pointGeode->setUserValue("pos", localPoint);

            osg::ref_ptr<osg::ShapeDrawable>
                    pointSphere = new osg::ShapeDrawable(new osg::Sphere(localPoint, 0.15f));
            pointSphere->setColor(traceLinesColor);
            pointGeode->addDrawable(pointSphere);
            traceNode->addChild(pointGeode);
        }

        // 显示点ID
        osg::ref_ptr<osg::Switch> pointTextNode =
                dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointTextNodeName));
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
            text->setColor(traceLinesColor);

            pointTextGeode->addDrawable(text);

            pointTextNode->addChild(pointTextGeode);
        }

        // 显示线ID
        osg::ref_ptr<osg::Switch> traceLineTextNode =
                dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, traceLineTextNodeName));
        for (const auto &traceLine : traceLines) {
            auto sPoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.sPID));
            auto ePoint = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::Point>(traceLine.ePID));

            osg::Vec3d pos1(sPoint.ly, sPoint.bx, sPoint.h);
            osg::Vec3d pos2(ePoint.ly, ePoint.bx, ePoint.h);
            osg::Vec3d pos = (pos1 + pos2) / 2;
            std::string name = "TTraceLine_" + std::to_string(traceLine.tlID);

            osg::ref_ptr<osg::Geode> lineTextGeode = new osg::Geode;
            lineTextGeode->setName(name);

            osg::ref_ptr<osgText::Text> text = new osgText::Text;
            text->setCharacterSize(0.5);
            text->setAxisAlignment(osgText::TextBase::XY_PLANE);
            text->setPosition(pos);
            text->setText(std::to_string(traceLine.tlID));
            text->setColor(traceLinesColor);

            lineTextGeode->addDrawable(text);

            traceLineTextNode->addChild(lineTextGeode);
        }

        cleanup();
    }
}

void TraceLineEditor::updateIndex() {
    curPointIndex = VectorMapSingleton::getInstance()->getMaxPointIndex() + 1;
}

void TraceLineEditor::rollback() {
    if (!selectedPoints.empty()) {
        selectedPoints.pop_back();
    }
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
    }
    // 删除已画的点和线
    tempNode->removeChildren(tempNode->getNumChildren() - 2, 2);
}

void TraceLineEditor::cleanup() {
    if (tempLineGeode != nullptr) {
        tempLineGeode->removeDrawables(0, tempLineGeode->getNumDrawables());
        tempLineGeode = nullptr;
    }
    tempNode->removeChildren(0, tempNode->getNumChildren());

    selectedPoints.clear();
}

double TraceLineEditor::calculateApproximateZ(const std::vector<osg::Vec3d> &points) const {
    double minZ, maxZ;
    double delta;
    int maxN = 0;
    double res = 0;
    // 个数-Z坐标
    std::vector<std::pair<int, std::vector<double>>> numZ(10);

    minZ = maxZ = points[0].z();
    for (const auto &point : points) {
        if (point.z() > maxZ) {
            maxZ = point.z();
        }
        if (point.z() < minZ) {
            minZ = point.z();
        }
    }
    delta = (maxZ - minZ) / 10;

    for (const auto &point : points) {
        int i = static_cast<int>((point.z() - minZ) / delta);
        if (i < 0) {
            i = 0;
        }
        if (i >= 10) {
            i = 9;
        }
        numZ[i].first++;
        numZ[i].second.push_back(point.z());
    }

    for (const auto &num : numZ) {
        if (num.first > maxN) {
            maxN = num.first;
            double sum = 0;
            for (const auto &n : num.second) {
                sum += n;
            }
            res = sum / num.first;
        }
    }

    return res;
}

