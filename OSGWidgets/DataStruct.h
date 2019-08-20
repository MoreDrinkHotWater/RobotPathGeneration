#ifndef LAS_DATA_STRUCT
#define LAS_DATA_STRUCT
#include <vector>
#include <list>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
using namespace std;

namespace pmProcessUrban {
struct HEADERBLOCK {
  char FileSignature[4];
  unsigned long Reserved;
  unsigned long ProjectIDGUIDdata1;
  unsigned short ProjectIDGUIDdata2;
  unsigned short ProjectIDGUIDdata3;
  unsigned char ProjectIDGUIDdata4[8];
  unsigned char VersionMajor;//
  unsigned char VersionMinor;//
  char SystemIdentifier[32];
  char GeneratingSoftware[32];
  unsigned short FileCreationDayofYear;
  unsigned short FileCreationYear;
  unsigned short HeaderSize;//
  unsigned long OffSetToPointData;//
  unsigned long NumberOfVariableLengthRecords;//
  unsigned char PointDataFormatID;//
  unsigned short PointDataRecordLength;//
  unsigned long NumberOfPointRecords;//This field contains the total number of point records within the file.
  unsigned long Numberofpointsbyreturn[5];/* This field contains an array of the total point records per return.
												The first unsigned long value will be the total number of records from the first return, and the
												second contains the total number for return two, and so forth up to five returns. */

  double XScaleFactor;//
  double YScaleFactor;//
  double ZScaleFactor;//
  double XOffSet;
  double YOffSet;
  double ZOffSet;
  double MaxX;
  double MinX;
  double MaxY;
  double MinY;
  double MaxZ;
  double MinZ;
};

struct VARISBLELENGTHRECORDS {
  unsigned short RecordSignature;
  char UserID[16];
  unsigned short RecordID;
  unsigned short RecordLengthAfterHeader;//
  char Description[32];
};

struct POINTDATARECORDFORMAT0 {
  /*The X, Y, and Z values are stored as long integers.  The X, Y, and Z values are
  used in conjunction with the scale values and the offset values to determine the
  coordinate for each point.
  Xcoordinate = (Xrecord * Xscale) + Xoffset
  Ycoordinate = (Yrecord * Yscale) + Yoffset
  Zcoordinate = (Zrecord * Zscale) + Zoffset */
  long X;//
  long Y;//
  long Z;//
  unsigned short Intensity;//
  unsigned short ReturnNumber;//
  unsigned short NumberofReturns;//
  unsigned short ScanDirectionFlag;/*A bit value of 1 is a positive scan direction, and a
										  bit value of 0 is a negative scan direction (where positive scan direction is a scan moving
										  from the left side of the in-track direction to the right side and negative the opposite). */
  unsigned short EdgeofFlightLine;/*The Edge of Flight Line data bit has a value of 1 only when the point is at
										 the end of a scan.  It is the last point on a given scan line before it changes direction. */
  unsigned char Classification;//
  char ScanAngleRank;/*The Scan Angle Rank is a signed one-byte number with a valid range from -90
						   to +90.  The Scan Angle Rank is the angle (rounded to the nearest integer in the absolute value
						   sense) at which the laser point was output from the laser system including the roll of the aircraft.
						   The scan angle is within 1 degree of accuracy from +90 to �C90 degrees.  The scan angle is an
						   angle based on 0 degrees being nadir, and �C90 degrees to the left side of the aircraft in the
						   direction of flight.*/
  unsigned char UserData;
  unsigned short PointSourceID;

};
/*���¼�ĸ�ʽ1*/
struct POINTDATARECORDFORMAT1 {
  long X;
  long Y;
  long Z;
  unsigned short Intensity;
  unsigned short ReturnNumber;
  unsigned short NumberofReturns;
  unsigned short ScanDirectionFlag;
  unsigned short EdgeofFlightLine;
  unsigned char Classification;
  char ScanAngleRank;
  unsigned char UserData;
  unsigned short PointSourceID;
  double GPSTime;//gps
};
struct POINTDATARECORDFORMAT2 {
  long X;
  long Y;
  long Z;
  unsigned short Intensity;
  unsigned short ReturnNumber;
  unsigned short NumberofReturns;
  unsigned short ScanDirectionFlag;
  unsigned short EdgeofFlightLine;
  unsigned char Classification;
  char ScanAngleRank;
  unsigned char UserData;
  unsigned short PointSourceID;
  unsigned short R;
  unsigned short G;
  unsigned short B;
};
struct POINTDATARECORDFORMAT3 {
  long X;
  long Y;
  long Z;
  unsigned short Intensity;
  unsigned short ReturnNumber;
  unsigned short NumberofReturns;
  unsigned short ScanDirectionFlag;
  unsigned short EdgeofFlightLine;
  unsigned char Classification;
  char ScanAngleRank;
  unsigned char UserData;
  unsigned short PointSourceID;
  unsigned short R;
  unsigned short G;
  unsigned short B;
  double GPSTime;//gpsʱ��
};

struct LAS_POINT {
  double x;
  double y;
  float z;
  uint8_t R;
  uint8_t G;
  uint8_t B;
  unsigned short Intensity;
};

struct LAS_POINT_PROPERTY {
  float normal_x;
  float normal_y;
  float normal_z;
  double d;
  float Principal_x;
  float Principal_y;
  float Principal_z;
  int Dimension;
  int SegmentID;
  int sort_id;
  bool is_edge;
  int obj_id;
  int obj_sort;
};

enum POINTCLASS {
  NONE_PT = -1,
  UNKNOWN_PT = 0,
  VERTICAL_POLE,
  HORIZENTAL_POLE,
  OTHER_POLE,
  VERTICAL_PLANE,
  HORIZENTAL_PLANE,
  OTHER_PLANE,
  SPHERALITY
};

struct Grid {
  bool is_empty;
  Grid() {
      is_empty = true;
  }
};
struct Voxel {
  vector<int> point_id;
  float min_z;
  float max_z;
  float dertaz;
  float min_z_x;
  float min_z_y;
  float NeighborMin_z;
  int PointsNumber;
  float mean_z;

  Voxel() {
      min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
      PointsNumber = 1;
      dertaz = 0.0;
  }
};

struct SimplifiedVoxel {
  vector<int> point_id;
  float max_curvature;
  int max_curvature_point_id;
  bool has_keypoint;
  SimplifiedVoxel() {
      has_keypoint = false;
  }
};

struct Voxel_z {
  vector<float> z;
};

struct LAS_POINT_PROPERTY_Simple {
  bool sort_id;
};

struct ExtremumCoordinate {
  float Centerx;
  float Centery;
  float Centerz;
  float Highx;
  float Highy;
  float Highz;
  float Lowx;
  float Lowy;
  float Lowz;
  float midz;
  int HighType;
};

struct WFSegPnts {
  float x;
  float y;
  float z;
  int PntSub;
  int Seglabel;
};

struct WFSegPntsType {
  float Max_z;
  float Mid_z;
  float Min_z;
  float PntNum;
  int SegLabel;
};

struct SegmentFeature {
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float ymax_x;
  float ymin_x;
  float width;
  float height;
  float Zmin;
  float Zmax;
  float x_zmin;
  float y_zmin;
  float height_differece_to_ground;
  double center_x;
  double center_y;
  double center_z;

  float normal_x;
  float normal_y;
  float normal_z;
  float d;

  float Principal_x;
  float Principal_y;
  float Principal_z;

  double average_distance_between_points;
  int r;
  int g;
  int b;
  float std;//
};

struct Segment {
  int SegmentID;

  int NumOfPoint;
  vector<int> point_id;
  vector<int> edge_point_id;
  SegmentFeature SegmentProperty;
  vector<int> NeighbourhoodSegmentID;
  int ObjectID;
  int segment_sort;
  vector<int> EdgePointsIDOfOneSegment;
  float ground_z;

};

struct Multi_Obj {
  int ObjectID;
  int ObjectSort;
  vector<int> point_id;
};
struct XYLabel {
  double x;
  double y;
  double z;
  int label;

};

struct PoleLikeObject_v {
  int ObjectID;
  int ObjectSort;
  float Zmin;
  float Zmax;
  float height;
  double centerx;
  double centery;
};

struct LINE {
  double x1;
  double y1;
  double z1;
  double x2;
  double y2;
  double z2;
  float a;
  float b;
  float c;
  int nunOfPoint;
  bool islow;
};

struct PlaneLikeObject_v {
  int ObjectID;
  int ObjectSort;
  float Zmin;
  float Zmax;
  float height;
  vector<LINE> lines;
};

struct Object {
  vector<int> SegmentID;
  vector<int> NeighbourhoodDistance;
  int ObjectID;
  int ObjectSort;
  float width;
  float height;
  float lenth;
  float Zmin;
  float Zmax;
  float HeightDifferenceBeteewnGeometricalCenterAndBarycenter;
  int NumOfPoint;
  double deviation;
  double percentage_of_ball_points;
  double percentage_of_vertical_plane_points;
  double percentage_of_vertical_pole_points;
  float height_differece_to_ground;
  int r;
  int g;
  int b;
  vector<int> point_id;
  float centerx_vertical_pole;
  float centery_vertical_pole;
  float ground_z;

  struct PoleLikeObject_v {
    int ObjectID;
    int ObjectSort;
    float Zmin;
    float Zmax;
    float height;
    double centerx;
    double centery;
  };

  struct PlaneLikeObject_v {
    int ObjectID;
    int ObjectSort;
    float Zmin;
    float Zmax;
    float height;
    vector<LINE> lines;
  };
};

struct PlaneFeature {
  double width;
  double height;
  int NumOfPoint;
  int SegLabel;
  float Plane_zmin;
  float Object_zmin;
  float a1;
  float b1;
  float c1;
  float d1;
  float xmax;
  float xmin;
  float ymax;
  float ymax_x;
  float ymin;
  float ymin_x;
};
struct dis_objectid {
  float dis;
  int object_id;
};

struct POINT_XYZ {
  double x;
  double y;
  double z;
};

struct VECTOR {
  int object_id;
  uint8_t entity_category;
  bool is_lowest;
  uint8_t object_category;
  uint8_t pts_num;
  vector<POINT_XYZ> line_point;
};

struct PARAMETER {

  float min_average_distance_between_points;
  int NumOfNeiboringPointsInCalculateMeanDistance;    //The number of neighbors to analyze for each point

  float cylinder_radius;
  float max_height_difference;

  double small_radius;
  double large_radius;
  float disT;

  float RadiusInRegionGrowning;
  float CosNormalT;
  float PlaneDistanceT;
  float CosPrincipalT;
  float PoleDistanceT;
  int point_num_in_min_seg;

  float min_scale_in_level_pole;
  float min_scale_in_Vertical_pole;
  float min_scale_in_other_pole;//2
  float min_scale_in_level_plane;
  float min_scale_in_Vertical_plane;
  float min_scale_in_other_plane;//2

  float radius_in_topological_relation;
  float SaliencySegScaleT;

  float max_height_difference_between_fasade_and_Ground;

};

struct Bounds {
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
  Bounds() {
      min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
  }
};

struct CenterPoint {
  double x;
  double y;
  double z;
  CenterPoint(double x = 0, double y = 0, double z = 0) :
          x(x), y(y), z(z) {
      z = 0.0;
      x = y = 0.0;
  }

};

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudXYZI_Ptr;
typedef pcl::PointCloud<pcl::PointXYZI> CloudXYZI;
}

#endif