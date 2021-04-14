#ifndef KITTIDATA_H
#define KITTIDATA_H
#include <QString>
#include <QStringList>
#include <QFileInfoList>
#include <QVector>
#include <QtOpenGL>
#include <QMatrix4x4>
#include <vector>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <kitti_player/Datatypes.h>

enum VelodyneLayer {
    Layer16,
    Layer64
};

class KittiData
{
public:
    KittiData();
    KittiData(QString path);
    ~KittiData();

    void path(QString path) { path_ = path; }
    QString path() { return path_; }
    void set_sequence(QString str_seq);

    QString seq_path() { return path_+"sequences/"+str_seq_+"/"; }
    QString gt_path() { return path_+"poses/"; }
    QString gt_fname() { return gt_path()+str_seq_+".txt"; }
    QString calib_fname() { return seq_path()+"calib.txt"; }
    QString ros_camera_calib_fname() { return ros_camera_calib_fname_; }
    QString ros_color_camera_calib_fname() { return ros_color_camera_calib_fname_; }

    int velodyne_layer() { return velodyne_layer_; }
    void velodyne_layer(VelodyneLayer layer) { velodyne_layer_ = layer; }

    cv::Mat& left_image() { return left_image_; }
    cv::Mat& right_image() { return right_image_; }
    cv::Mat& left_color_image() { return left_color_image_; }
    cv::Mat& right_color_image() { return right_color_image_; }

    PointCloud& velodyne_data() { return velodyne_data_; }

    void set_left_image(int i) { left_image_ = cv::imread(get_abs_path(flist_left_image_, i).toStdString(),CV_LOAD_IMAGE_GRAYSCALE); }
    void set_right_image(int i) { right_image_ = cv::imread(get_abs_path(flist_right_image_, i).toStdString(),CV_LOAD_IMAGE_GRAYSCALE); }
    void set_left_color_image(int i) { left_color_image_ = cv::imread(get_abs_path(flist_left_color_image_, i).toStdString()); }
    void set_right_color_image(int i) { right_color_image_ = cv::imread(get_abs_path(flist_right_color_image_, i).toStdString()); }

    void set_velodyne(int i) { read_velodyne(get_abs_path(flist_velodyne_, i)); }

    double get_time_diff(int i) { return times_[i+1]-times_[i]; }
    int data_length() { return times_.size(); }

    Matrix3x4& P0() { return P0_; }
    Matrix3x4& P1() { return P1_; }
    Matrix3x4& P2() { return P2_; }
    Matrix3x4& P3() { return P3_; }
    Matrix3x4& Tr() { return Tr_; }

    void read_velodyne(QString fname);
    void set_write_bin(bool is_write_bin) { is_write_bin_ = is_write_bin; }

    static QImage cv_mat_to_qimage(cv::Mat &src);

    cv::Mat& spherical_velodyne() { return spherical_velodyne_; }

private:
    QString path_;
    QString str_seq_;

    QString left_image_path_;
    QString right_image_path_;
    QString left_color_image_path_;
    QString right_color_image_path_;

    PointCloud velodyne_data_;

    QString velodyne_path_;
    QString gt_fname_;
    QString calib_fname_;
    QString ros_camera_calib_fname_;
    QString ros_color_camera_calib_fname_;

    QFileInfoList flist_left_image_;
    QFileInfoList flist_right_image_;
    QFileInfoList flist_left_color_image_;
    QFileInfoList flist_right_color_image_;
    QFileInfoList flist_velodyne_;

    VelodyneLayer velodyne_layer_;

    QVector<double> times_;
    QVector<QMatrix4x4> poses_;

    Matrix3x4 P0_, P1_, P2_, P3_;
    Matrix3x4 Tr_;

    bool is_write_bin_;

    Eigen::Vector3d translation_;
    Eigen::Quaterniond quat_;


    // Current Data
    cv::Mat left_image_;
    cv::Mat right_image_;
    cv::Mat left_color_image_;
    cv::Mat right_color_image_;

    QString get_abs_path (QFileInfoList& flist, int i) { return flist.at(i).absoluteFilePath(); }

    QFileInfoList get_filelist(const QString path, const QString name_filter);
    void print_filelist(const QFileInfoList flist);

    // velodyne grandslam
    cv::Mat spherical_velodyne_;

};

#endif // KITTIDATA_H
