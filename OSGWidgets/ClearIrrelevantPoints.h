//
// Created by zhihui on 8/5/19.
//
#ifndef DEMO_TWO_CLEARIRRELEVANTPOINTS_H
#define DEMO_TWO_CLEARIRRELEVANTPOINTS_H

#include <QObject>
#include <osg/ref_ptr>
#include <osgViewer/View>
#include <osg/Switch>
#include "ClearPointsEvents.h"
#include "PolygonClearPointsEvents.h"

class ClearIrrelevantPoints : public QObject {
    Q_OBJECT
public:
    explicit ClearIrrelevantPoints() = default;

    ~ClearIrrelevantPoints() override;

    void removeEvent();

    void removePolygonEvent();

private:

    osg::ref_ptr<osgViewer::View> mainView;
    osg::ref_ptr<osg::Switch> rootNode;

    osg::ref_ptr<ClearPointsEvents> clearPointsEvents;

    osg::ref_ptr<PolygonClearPointsEvents> polygonClearPointsEvents;
public Q_SLOTS:

    void clearIrrelevantPointsSlot(osg::ref_ptr<osg::Switch> rootnode,osg::ref_ptr<osgViewer::View> mainview, bool isactive);

    void PolygonclearIrrelevantPointsSlot(osg::ref_ptr<osg::Switch> rootnode,osg::ref_ptr<osgViewer::View> mainview, bool isactive);
};

#endif //DEMO_TWO_CLEARIRRELEVANTPOINTS_H
