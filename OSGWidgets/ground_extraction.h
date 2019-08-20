#ifndef GROUNDEXTRACTION
#define GROUNDEXTRACTION

#include "DataStruct.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace pmProcessUrban {
class CGroundExtraction {
 public:

  CGroundExtraction();
  ~CGroundExtraction();
  /*�������ȡ,�ú�������ʶ������ͷǵ����;*/
  void ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud);

  void SetMaxHeightDifference(float max_height_difference) { max_height_difference_ = max_height_difference; }
  void SetMinPointNumInGrid(int min_pt_num_in_grid) { min_pt_num_in_grid_ = min_pt_num_in_grid; }
  void SetGridResolution(float grid_resolution) { grid_resolution_ = grid_resolution; }

 protected:

 private:

  void getCloudBound(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Bounds &bound);

  /*��������е���͵�߳�;*/
  void CalculateGridAndMinZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            double max_x, double max_y, double min_x, double min_y,
                            int row, int list, int num_voxel, Voxel *grid);

  /*���ݵ��Grid����͵�ĸ̲߳�жϵ����ͷǵ����;*/
  void JudgeGroundAndNonGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
                                     Voxel *grid, int num_voxel);

  /*�Ե�����һ���жϣ�����õ���������Χ�����ܶ࣬���ж�Ϊ�ǵ����;*/
  void FilterNotGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud);

  float grid_resolution_;//�����ķֱ���;
  int min_pt_num_in_grid_;//��������С�ĵ������С�ڸõ�������Ϊ�ø���Ϊ������;
  float max_height_difference_;//�����ڵĵ����������͵�ĸ̲߳�,���ڸ�ֵ����Ϊ�Ƿǵ����;


};
}
#endif