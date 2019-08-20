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
#include "DrivingArrowEditorDialog.h"
#include "DrivingArrowModification.h"

DrivingArrowModification::DrivingArrowModification(osg::Switch *rootNode) : rootNode(rootNode), drivingArrowItemNode(
        dynamic_cast<osg::Switch *>(NodeTreeSearch::findNodeWithName(rootNode, drivingArrowItemNodeName))),
                                                                            childNode(nullptr), x(0), y(0) {}

bool DrivingArrowModification::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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

void DrivingArrowModification::pick(const osgGA::GUIEventAdapter &ea, osgViewer::View *view) {
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
                    size_t modifyDrivingArrowID = std::stoi(childNode->getName().substr(13));

                    mdc::DrivingArrow drivingArrow = VectorMapSingleton::getInstance()->findByID(
                            mdc::Key<mdc::DrivingArrow>(modifyDrivingArrowID));

                    auto *drivingArrowEditorDialog = new DrivingArrowEditorDialog(&drivingArrow.type,
                                                                                  &drivingArrow.typeIndex);
                    drivingArrowEditorDialog->exec();

                    VectorMapSingleton::getInstance()->update(drivingArrow);

                    osg::ref_ptr<osg::Switch> drivingArrowTextNode = dynamic_cast<osg::Switch *> (NodeTreeSearch::findNodeWithName(
                            rootNode, drivingArrowTextNodeName));
                    osg::ref_ptr<osg::Geode> drivingArrowTextGeode = dynamic_cast<osg::Geode *>(NodeTreeSearch::findNodeWithName(
                            rootNode, std::string("TDrivingArrow_" + std::to_string(drivingArrow.daID)).c_str()));

                    for (int i = 0; i < drivingArrowTextGeode->getNumDrawables(); ++i) {
                        if (drivingArrowTextGeode->getDrawable(i)->getName() == "drivingArrowText") {
                            auto childTextNode = dynamic_cast<osgText::Text *>(drivingArrowTextGeode->getDrawable(i));
                            childTextNode->setText(drivingArrow.type);
                            break;
                        }
                    }

                    break;
                }
            }
        }
    }
}