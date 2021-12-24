#ifndef PNADAR_COMMON_HPP
#define PNADAR_COMMON_HPP

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>

#include <utils/pcl_utils.h>

template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}


namespace licalib {

    class PandarConverter {
    public:
        typedef std::shared_ptr<PandarConverter> Ptr;

        enum ModelType {
            PandarXT,
        };

        PandarConverter(ModelType modelType = PandarXT) : m_modelType_(modelType) {}

        /**
         * @brief 点云格式转换函数，此函数会改变lidarMsg与outPointCloud
         *
         * @param lidarMsg ROS类型的点云消息（原本时间戳在是点云中最后一个点，现在改为点云中第一个点的了）
         * @param outPointCloud PointXYZIT类型的点云
         */
        void unpack_scan(sensor_msgs::PointCloud2::Ptr &lidarMsg, TPointCloud &outPointCloud) const {
//            pcl::PointCloud<licalib::PointXYZIT> cloud;
            pcl::fromROSMsg(*lidarMsg, outPointCloud);
            for (int i = 0; i < outPointCloud.size(); i++)
            {
                outPointCloud.points[i].intensity = outPointCloud.points[i].ring; //intensity存ring值
            }

            /*outPointCloud.is_dense = false;
            int plsize = cloud.points.size();
            cloud.resize(plsize);
            for (int i = 0; i < plsize; i++)
            {
                TPoint added_pt;
                added_pt.x = cloud.points[i].x;
                added_pt.y = cloud.points[i].y;
                added_pt.z = cloud.points[i].z;
                added_pt.intensity = cloud.points[i].intensity;
                added_pt.timestamp = cloud.points[i].timestamp; //s
                cloud.push_back(added_pt);
            }*/
        }

    private:
        ModelType m_modelType_;
    };
}
#endif