//
// Created by zhihui on 3/28/19.
//

#ifndef POINTCLOUDAPPLICATION_TEXTCONTROLLER_H
#define POINTCLOUDAPPLICATION_TEXTCONTROLLER_H

#include <osgGA/GUIEventHandler>

class TextController : public osgGA::GUIEventHandler {
public:
    explicit TextController(osg::Switch *node);

    ~TextController() override = default;

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

private:
    void initChildrenMask(osg::Switch *node);

    void reverseNodeMask(osg::Switch *node);

private:
    osg::ref_ptr<osg::Switch> rootNode;
    osg::ref_ptr<osg::Switch> textNode;
    osg::ref_ptr<osg::Switch> pointTextNode;
    osg::ref_ptr<osg::Switch> lineTextNode;
};


#endif //POINTCLOUDAPPLICATION_TEXTCONTROLLER_H
