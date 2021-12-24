/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Eigen>

namespace licalib {

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef pcl::PointXYZRGB colorPointT;
typedef pcl::PointCloud<colorPointT> colorPointCloudT;

struct PointXYZIT {
  PCL_ADD_POINT4D
  float intensity;
  double timestamp;
  uint16_t  ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

    struct PointXYZIR {
        PCL_ADD_POINT4D
        float intensity;
        uint16_t  ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

    inline void downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                                float in_leaf_size) {

        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(in_cloud);
        sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
        sor.filter(*out_cloud);
    }

};

POINT_CLOUD_REGISTER_POINT_STRUCT(licalib::PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (std::uint16_t, ring, ring))


POINT_CLOUD_REGISTER_POINT_STRUCT(licalib::PointXYZIR,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring))

//typedef licalib::PointXYZIR VPoint;
//typedef pcl::PointCloud<VPoint> VPointCloud;

typedef licalib::PointXYZIT TPoint;
typedef pcl::PointCloud<TPoint> TPointCloud;

inline void TPointCloud2VPointCloud(TPointCloud::Ptr input_pc,
                                    licalib::VPointCloud::Ptr output_pc) {

    output_pc->header = input_pc->header;
    output_pc->height = input_pc->height;
    output_pc->width = input_pc->width;
    output_pc->is_dense = input_pc->is_dense;
    output_pc->resize(output_pc->width * output_pc->height);
    if(input_pc->height == 1){
        for(int i = 0; i < input_pc->size(); i++){
            licalib::VPoint point;
            point.x = input_pc->at(i).x;
            point.y = input_pc->at(i).y;
            point.z = input_pc->at(i).z;
            point.intensity = input_pc->at(i).ring; //intensity 保存 ring 值
            output_pc->at(i) = point;
        }
    }
    else{
        for(int h = 0; h < input_pc->height; h++) {
            for(int w = 0; w < input_pc->width; w++) {
                licalib::VPoint point;
                point.x = input_pc->at(w,h).x;
                point.y = input_pc->at(w,h).y;
                point.z = input_pc->at(w,h).z;
                point.intensity = input_pc->at(w,h).ring;
                output_pc->at(w,h) = point;
            }
        }
    }
}

/*
inline void TPointCloud2VPointCloud(TPointCloud::Ptr input_pc,
                                    VPointCloud::Ptr output_pc) {

  output_pc->header = input_pc->header;
  output_pc->height = input_pc->height;
  output_pc->width = input_pc->width;
  output_pc->is_dense = input_pc->is_dense;
  output_pc->resize(output_pc->width * output_pc->height);
  if(input_pc->height == 1){
      for(int i = 0; i < input_pc->size(); i++){
          VPoint point;
          point.x = input_pc->at(i).x;
          point.y = input_pc->at(i).y;
          point.z = input_pc->at(i).z;
          point.intensity = input_pc->at(i).intensity;
          point.ring = input_pc->at(i).ring;
          output_pc->at(i) = point;
      }
  }
  else{
      for(int h = 0; h < input_pc->height; h++) {
          for(int w = 0; w < input_pc->width; w++) {
              VPoint point;
              point.x = input_pc->at(w,h).x;
              point.y = input_pc->at(w,h).y;
              point.z = input_pc->at(w,h).z;
              point.intensity = input_pc->at(w,h).intensity;
              point.ring = input_pc->at(w,h).ring;
              output_pc->at(w,h) = point;
          }
      }
  }
}

inline void VPointCloud2PCLXYZI(VPointCloud::Ptr input_pc,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc) {

    output_pc->header = input_pc->header;
    output_pc->height = input_pc->height;
    output_pc->width = input_pc->width;
    output_pc->is_dense = input_pc->is_dense;
    output_pc->resize(output_pc->width * output_pc->height);
    if(input_pc->height == 1){
        for(int i = 0; i < input_pc->size(); i++){
            pcl::PointXYZI point;
            point.x = input_pc->at(i).x;
            point.y = input_pc->at(i).y;
            point.z = input_pc->at(i).z;
            point.intensity = input_pc->at(i).intensity;
            output_pc->at(i) = point;
        }
    }
    else{
        for(int h = 0; h < input_pc->height; h++) {
            for(int w = 0; w < input_pc->width; w++) {
                pcl::PointXYZI point;
                point.x = input_pc->at(w,h).x;
                point.y = input_pc->at(w,h).y;
                point.z = input_pc->at(w,h).z;
                point.intensity = input_pc->at(w,h).intensity;
                output_pc->at(w,h) = point;
            }
        }
    }
}


inline void downsampleCloud(VPointCloud::Ptr in_cloud,
                            VPointCloud::Ptr out_cloud,
                            float in_leaf_size) {

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_new;
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_new;
    VPointCloud2PCLXYZI(in_cloud,in_cloud_new);

    sor.setInputCloud(in_cloud_new);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_new);
}*/

#endif
