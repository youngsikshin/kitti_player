#ifndef DATATYPES_H
#define DATATYPES_H

#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>

typedef double NumType;
typedef Eigen::Matrix<NumType, 3, 4> Matrix3x4;

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

#endif // DATATYPES_H
