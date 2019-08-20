//
// Created by zhihui on 4/11/19.
//

#ifndef HDMAPS_COLOR_H
#define HDMAPS_COLOR_H

const std::vector<osg::Vec3> ColorZList = {osg::Vec3(0.0, 0.0, 0.0),

                                           osg::Vec3(0.0, 0.0, 0.1), osg::Vec3(0.0, 0.0, 0.2), osg::Vec3(0.0, 0.0, 0.3),
                                           osg::Vec3(0.0, 0.0, 0.4), osg::Vec3(0.0, 0.0, 0.5), osg::Vec3(0.0, 0.0, 0.6),
                                           osg::Vec3(0.0, 0.0, 0.7), osg::Vec3(0.0, 0.0, 0.8), osg::Vec3(0.0, 0.0, 0.9),

                                           osg::Vec3(0.0, 0.0, 1.0),

                                           osg::Vec3(0.0, 0.1, 1.0), osg::Vec3(0.0, 0.2, 1.0), osg::Vec3(0.0, 0.3, 1.0),
                                           osg::Vec3(0.0, 0.4, 1.0), osg::Vec3(0.0, 0.5, 1.0), osg::Vec3(0.0, 0.6, 1.0),
                                           osg::Vec3(0.0, 0.7, 1.0), osg::Vec3(0.0, 0.8, 1.0), osg::Vec3(0.0, 0.9, 1.0),

                                           osg::Vec3(0.0, 1.0, 1.0),

                                           osg::Vec3(0.0, 1.0, 0.9), osg::Vec3(0.0, 1.0, 0.8), osg::Vec3(0.0, 1.0, 0.7),
                                           osg::Vec3(0.0, 1.0, 0.6), osg::Vec3(0.0, 1.0, 0.5), osg::Vec3(0.0, 1.0, 0.4),
                                           osg::Vec3(0.0, 1.0, 0.3), osg::Vec3(0.0, 1.0, 0.2), osg::Vec3(0.0, 1.0, 0.1),

                                           osg::Vec3(0.0, 1.0, 0.0),

                                           osg::Vec3(0.1, 1.0, 0.0), osg::Vec3(0.2, 1.0, 0.0), osg::Vec3(0.3, 1.0, 0.0),
                                           osg::Vec3(0.4, 1.0, 0.0), osg::Vec3(0.5, 1.0, 0.0), osg::Vec3(0.6, 1.0, 0.0),
                                           osg::Vec3(0.7, 1.0, 0.0), osg::Vec3(0.8, 1.0, 0.0), osg::Vec3(0.9, 1.0, 0.0),

                                           osg::Vec3(1.0, 1.0, 0.0),

                                           osg::Vec3(1.0, 0.9, 0.0), osg::Vec3(1.0, 0.8, 0.0), osg::Vec3(1.0, 0.7, 0.0),
                                           osg::Vec3(1.0, 0.6, 0.0), osg::Vec3(1.0, 0.5, 0.0), osg::Vec3(1.0, 0.4, 0.0),
                                           osg::Vec3(1.0, 0.3, 0.0), osg::Vec3(1.0, 0.2, 0.0), osg::Vec3(1.0, 0.1, 0.0),

                                           osg::Vec3(1.0, 0.0, 0.0),

};

const osg::Vec4 pavementColor = osg::Vec4(0.3, 0.6, 0.9, 1.0);

const osg::Vec4 drivingArrowColor = osg::Vec4(0.9, 0.3, 0.6, 1.0);

const osg::Vec4 trafficLightsColor = osg::Vec4(0.6, 0.9, 0.3, 1.0);

const osg::Vec4 roadLinesColor = osg::Vec4(0.3, 0.9, 0.6, 1.0);

const osg::Vec4 traceLinesColor = osg::Vec4(0.6, 0.3, 0.9, 1.0);

const osg::Vec4 measurePointsColor = osg::Vec4(1.0, 0.8, 0.6, 1.0);

#endif //HDMAPS_COLOR_H
