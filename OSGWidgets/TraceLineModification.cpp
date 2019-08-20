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
#include <Color.h>

#include "NodeTreeSearch.h"
#include "NodeNames.h"
#include "VectorMapSingleton.h"
#include "TraceLineModification.h"

TraceLineModification::TraceLineModification(osg::Switch *rootNode)
        : rootNode(rootNode),
          traceItemNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, traceItemNodeName))),
          pointCloudNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, pointCloudNodeName))),
          virtualPlaneNode(dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                        virtualPlaneNodeName))),
          tempModTraceLineGeode(nullptr),
          selectedPoint(std::make_pair(0, osg::Vec3d())),
          involvedPoints(),
          redrawTraceLines(),
          count(1),
          deletedPoint(),
          x(0),
          y(0) {}

TraceLineModification::~TraceLineModification() {
    cleanup();
}

bool TraceLineModification::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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
            view->getCamera()->accept(iv);
            bool hasAlready = false;
            osg::Vec3d curPoint;
            if (picker->containsIntersections()) {

                auto intersections = picker->getIntersections();
                for (const auto &intersection : intersections) {
                    auto childNode = intersection.nodePath.back();
                    if (childNode->getName() == "Sphere") {
                        return false;
                    }
                    if (childNode->getName() == "CloudPoints") {
                        curPoint = intersection.localIntersectionPoint;
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

            if (tempModTraceLineGeode == nullptr) {
                tempModTraceLineGeode = new osg::Geode;
                tempModTraceLineGeode->setName("tempModTraceLineGeode");
                traceItemNode->addChild(tempModTraceLineGeode);
            } else {
                tempModTraceLineGeode->removeDrawables(0, tempModTraceLineGeode->getNumDrawables());
            }

            for (const auto &point : involvedPoints) {
                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                vertices->push_back(std::get<1>(point));
                vertices->push_back(curPoint);

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setName("tempModTraceLineGeom");
                tempModTraceLineGeode->addDrawable(geom);

                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, vertices->size()));
                osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;
                colors->push_back(osg::Vec3(1.0, 0.0, 0.0));

                geom->setColorArray(colors.get());
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->setVertexArray(vertices.get());
            }

            virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
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

void TraceLineModification::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
    osg::ref_ptr<osg::Switch> traceNode;
    osg::ref_ptr<osg::Geode> geodeNode;

    osg::ref_ptr<osg::Switch> traceLineTextNode;
    osg::ref_ptr<osg::Switch> pointTextNode;

    if (selectedPoint.first == 0) {
        count = 1;
        involvedPoints.clear();
    }
    if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

        if (count == 1) {

            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);

            // 第一次点击是选需要修改的点
            redrawTraceLines.clear();
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
                        mdc::TraceLine traceLine;
                        for (const auto &ID : point.fromPointLineID) {
                            redrawTraceLines.push_back(ID);
                            traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(ID));
                            mdc::Point ePoint = VectorMapSingleton::getInstance()->findByID(
                                    mdc::Key<mdc::Point>(traceLine.ePID));
                            involvedPoints.emplace_back(
                                    std::make_pair(ePoint.pID, osg::Vec3d(ePoint.ly, ePoint.bx, ePoint.h)));
                        }
                        for (const auto &ID : point.toPointLineID) {
                            redrawTraceLines.push_back(ID);
                            traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(ID));
                            mdc::Point sPoint = VectorMapSingleton::getInstance()->findByID(
                                    mdc::Key<mdc::Point>(traceLine.sPID));
                            involvedPoints.emplace_back(
                                    std::make_pair(sPoint.pID, osg::Vec3d(sPoint.ly, sPoint.bx, sPoint.h)));
                        }
                        count++;
                        break;
                    }
                }
            }
        } else if (count == 2) {
            double w = 1.5, h = 1.5;
            osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
            osgUtil::IntersectionVisitor iv(picker);
            view->getCamera()->accept(iv);
            bool hasAlready = false;
            osg::Vec3d localPoint;
            // 第二次点击是选新的点
            if (picker->containsIntersections()) {

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
                            break;
                        }
                        localPoint.z() = approximateZ;
                        mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                                mdc::Key<mdc::Point>(selectedPoint.first));
                        point.ly = localPoint.x();
                        point.bx = localPoint.y();
                        point.h = localPoint.z();
                        VectorMapSingleton::getInstance()->update(point);
                    } else {
                        virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
                        return;
                    }
                } else {
                    return;
                }
            }

            std::string itemTraceName;
            for (const auto &ID : redrawTraceLines) {
                geodeNode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(rootNode, std::string(
                        "TraceLine_" + std::to_string(ID)).c_str()));
                std::cout << "geodeNode: " << geodeNode->getName() << std::endl;
                traceNode = dynamic_cast<osg::Switch *>(geodeNode->getParent(0));
                traceNode->removeChild(geodeNode);

                auto traceLine = VectorMapSingleton::getInstance()->findByID(mdc::Key<mdc::TraceLine>(ID));
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
                geode->setName("TraceLine_" + std::to_string(ID));
                geode->addDrawable(geom);

                traceNode->addChild(geode);

                traceLineTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                 traceLineTextNodeName));
                geodeNode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(rootNode, std::string(
                        "TTraceLine_" + std::to_string(ID)).c_str()));

                traceLineTextNode->removeChild(geodeNode);
                osg::Vec3d pos1(sPoint.ly, sPoint.bx, sPoint.h);
                osg::Vec3d pos2(ePoint.ly, ePoint.bx, ePoint.h);
                osg::Vec3d pos = (pos1 + pos2) / 2;
                std::string name = "TTraceLine_" + std::to_string(traceLine.tlID);

                osg::ref_ptr<osg::Geode> traceLineTextGeode = new osg::Geode;
                traceLineTextGeode->setName(name);

                osg::ref_ptr<osgText::Text> text = new osgText::Text;
                text->setCharacterSize(0.5);
                text->setAxisAlignment(osgText::TextBase::XY_PLANE);
                text->setPosition(pos);
                text->setText(std::to_string(traceLine.tlID));
                text->setColor(traceLinesColor);

                traceLineTextGeode->addDrawable(text);

                traceLineTextNode->addChild(traceLineTextGeode);

                if (itemTraceName != traceNode->getName()) {
                    mdc::Point point = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::Point>(selectedPoint.first));
                    osg::Vec3d newPoint(osg::Vec3d(point.ly, point.bx, point.h));
                    osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
                    pointGeode->setName("Sphere");
                    pointGeode->setUserValue("ID", static_cast<int>(point.pID));
                    pointGeode->setUserValue("pos", newPoint);

                    osg::ref_ptr<osg::ShapeDrawable> pointSphere = new osg::ShapeDrawable(
                            new osg::Sphere(newPoint, 0.15f));
                    pointSphere->setColor(traceLinesColor);
                    pointGeode->addDrawable(pointSphere);
                    traceNode->addChild(pointGeode);

                    pointTextNode = dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                                 pointTextNodeName));
                    geodeNode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(rootNode,
                                                                                            std::string(
                                                                                                    "TSphere_"
                                                                                                            + std::to_string(
                                                                                                                    point.pID)).c_str()));
                    pointTextNode->removeChild(geodeNode);

                    name = "TSphere_" + std::to_string(point.pID);

                    osg::ref_ptr<osg::Geode> pointTextGeode = new osg::Geode;
                    pointTextGeode->setName(name);

                    text = new osgText::Text;
                    text->setCharacterSize(0.5);
                    text->setAxisAlignment(osgText::TextBase::XY_PLANE);
                    text->setPosition(newPoint);
                    text->setText(std::to_string(point.pID));
                    text->setColor(traceLinesColor);

                    pointTextGeode->addDrawable(text);

                    pointTextNode->addChild(pointTextGeode);

                }
                itemTraceName = traceNode->getName();
            }
            for (const auto &point : deletedPoint) {
                point.first->removeChild(point.second);
            }
            selectedPoint.first = 0;
            selectedPoint.second = osg::Vec3d();
            involvedPoints.clear();
            if (tempModTraceLineGeode != nullptr) {
                tempModTraceLineGeode->removeDrawables(0, tempModTraceLineGeode->getNumDrawables());
            }
            traceItemNode->removeChild(tempModTraceLineGeode);
            tempModTraceLineGeode = nullptr;
            virtualPlaneNode->removeChildren(0, virtualPlaneNode->getNumChildren());
        }
    }
}

void TraceLineModification::cleanup() {
    if (tempModTraceLineGeode != nullptr) {
        tempModTraceLineGeode->removeDrawables(0, tempModTraceLineGeode->getNumDrawables());
    }
    traceItemNode->removeChild(tempModTraceLineGeode);
    tempModTraceLineGeode = nullptr;
    selectedPoint.first = 0;
    selectedPoint.second = osg::Vec3d();
    involvedPoints.clear();
}

double TraceLineModification::calculateApproximateZ(const std::vector<osg::Vec3d> &points) const {
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