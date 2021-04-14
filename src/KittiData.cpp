#include <iostream>
#include <QDir>
#include <QTextStream>
#include <QDataStream>

#include <QDebug>

#include <kitti_player/KittiData.h>

using namespace std;

KittiData::KittiData()
  :  spherical_velodyne_(50, 900, CV_32FC1)
{
    str_seq_ = "00";
    is_write_bin_ = false;
}

KittiData::KittiData(QString path)
{
//    cerr << "[KittiData]\t Called KittiData(QString path)" << endl;
    path_ = path;
    str_seq_ = "00";
    is_write_bin_ = false;
}

KittiData::~KittiData()
{

}

void KittiData::set_sequence(QString str_seq)
{
    times_.clear();
    poses_.clear();

    str_seq_ = str_seq;

    left_image_path_ = seq_path() + "image_0";
    right_image_path_ = seq_path() + "image_1";
    left_color_image_path_ = seq_path() + "image_2";
    right_color_image_path_ = seq_path() + "image_3";

    velodyne_path_ = seq_path() + "velodyne";
    gt_fname_ = gt_fname();
    calib_fname_ = calib_fname();

    flist_left_image_ = get_filelist(left_image_path_,"*.png");
    flist_right_image_ = get_filelist(right_image_path_,"*.png");
    flist_left_color_image_ = get_filelist(left_color_image_path_,"*.png");
    flist_right_color_image_ = get_filelist(right_color_image_path_,"*.png");
    flist_velodyne_ = get_filelist(velodyne_path_, "*.bin");

    // Read times.txt
    QFile ftimes(seq_path()+"times.txt");
    if (!ftimes.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream times_stream(&ftimes);
    while (!times_stream.atEnd()) {
        QString line = times_stream.readLine();
        times_.push_back(line.toDouble());
    }

    // Read ground truth poses
    if(str_seq.toInt() < 10) {
        QFile gt_poses(gt_fname_);
        if (!gt_poses.open(QIODevice::ReadOnly | QIODevice::Text))
            return;

        QTextStream poses_stream(&gt_poses);
        while (!poses_stream.atEnd()) {
            QString line = poses_stream.readLine();
            QStringList list = line.split(" ");
            QMatrix4x4 Tgt(list[0].toDouble(), list[1].toDouble(), list[2].toDouble(), list[3].toDouble(),
                           list[4].toDouble(), list[5].toDouble(), list[6].toDouble(), list[7].toDouble(),
                           list[8].toDouble(), list[9].toDouble(), list[10].toDouble(), list[11].toDouble(),
                           0,    0,    0,    1);
            poses_.push_back(Tgt);
        }
    }

    // Read Calibration Parameter
    QFile fcalib(calib_fname_);
    if (!fcalib.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream fcalib_stream(&fcalib);
    vector<Matrix3x4> vec_calib_matrix;
    int calib_mat_idx=0;

    while (!fcalib_stream.atEnd()) {
        QString line = fcalib_stream.readLine();
        QStringList list = line.split(" ");

        Matrix3x4 calib_mat;
        calib_mat << list[1].toDouble(), list[2].toDouble(), list[3].toDouble(), list[4].toDouble(),
                     list[5].toDouble(), list[6].toDouble(), list[7].toDouble(), list[8].toDouble(),
                     list[9].toDouble(), list[10].toDouble(), list[11].toDouble(), list[12].toDouble();

        vec_calib_matrix.push_back(calib_mat);
    }

    P0_ = vec_calib_matrix[0];
    P1_ = vec_calib_matrix[1];
    P2_ = vec_calib_matrix[2];
    P3_ = vec_calib_matrix[3];
    Tr_ = vec_calib_matrix[4];

    std::cout << Tr_.matrix() << std::endl;
    std::cout <<Tr_.block<3,3>(0,0) << std::endl;

    Eigen::Quaterniond quat(Tr_.block<3,3>(0,0));
    quat_=quat;
    translation_ << Tr_(0,3) , Tr_(1,3) , Tr_(2,3);

    std::cout << translation_.matrix() << std::endl;
    std::cout << quat_.x() << std::endl;
    std::cout << quat_.y() << std::endl;
    std::cout << quat_.z() << std::endl;
    std::cout << quat_.w() << std::endl;

    if(!str_seq_.compare("00") || !str_seq_.compare("01") || !str_seq_.compare("02")) {
        ros_camera_calib_fname_ = "KITTI00-02.yaml";
        ros_color_camera_calib_fname_ = "KITTI00-02_color.yaml";
    }
    else if(!str_seq_.compare("03")) {
        ros_camera_calib_fname_ = "KITTI03.yaml";
        ros_color_camera_calib_fname_ = "KITTI03_color.yaml";
    }
    else if(!str_seq_.compare("04") || !str_seq_.compare("05") || !str_seq_.compare("06") || !str_seq_.compare("07") || !str_seq_.compare("08") || !str_seq_.compare("09") || !str_seq_.compare("10") || !str_seq_.compare("11") || !str_seq_.compare("12")) {
        ros_camera_calib_fname_ = "KITTI04-12.yaml";
        ros_color_camera_calib_fname_ = "KITTI04-12_color.yaml";
    }
    else {
        ros_camera_calib_fname_ = "KITTI00-02.yaml";
        ros_color_camera_calib_fname_ = "KITTI00-02_color.yaml";
    }

    qDebug() << ros_camera_calib_fname_;

}

QImage KittiData::cv_mat_to_qimage(cv::Mat &src) {
  QImage dst;

  switch (src.type()) {
  case CV_8UC1:
    dst = QImage(src.data, src.cols, src.rows, static_cast<int> (src.step), QImage::Format_Grayscale8);
    break;
  case CV_8UC3:
    dst = QImage(src.data, src.cols, src.rows, static_cast<int> (src.step), QImage::Format_RGB888);
    dst = dst.rgbSwapped();
  default:
    break;
  }

//  cv::namedWindow("test");
//  cv::imshow("test", src);
//  cv::waitKey(1);

  return dst;
}


QFileInfoList KittiData::get_filelist(const QString path, const QString name_filter)
{
    QDir dir(path);
    QStringList name_filters;
    name_filters << name_filter;

    QFileInfoList filelist = dir.entryInfoList(name_filters, QDir::NoDotAndDotDot|QDir::AllDirs|QDir::Files);

    return filelist;
}

void KittiData::print_filelist(const QFileInfoList flist)
{
    std::cout << "[KittiData]\t" << flist.size() << std::endl;
    for (int i = 0; i < flist.size(); i++)
    {
        QFileInfo fi = flist.at(i);
        if (fi.isFile()) {
//            std::cout << fi.fileName().toStdString() << std::endl;
        }
    }
}

void KittiData::read_velodyne(QString fname)
{
    QFile velodyne_file(fname);

    QStringList split_fname = fname.split('/');
    QString out_fname(split_fname.last());
    QFile out_file(out_fname);
    QDataStream out;

    if(is_write_bin_) {
        qDebug() << out_fname;
        out_file.open(QIODevice::WriteOnly);
        out.setDevice(&out_file);
        out.setByteOrder(QDataStream::LittleEndian);
        out.setFloatingPointPrecision(QDataStream::SinglePrecision);
    }

    if (!velodyne_file.open(QIODevice::ReadOnly))
        return;

    QDataStream in(&velodyne_file);
    velodyne_data_.clear();

    int cnt = 0;

    bool init_azimuth = false;
    double prev_azimuth;

    PointCloud single_layer;

    spherical_velodyne_ = cv::Mat::zeros(64, 900, CV_32FC1);

    while(!in.atEnd()) {
        in.setByteOrder(QDataStream::LittleEndian);
        in.setFloatingPointPrecision(QDataStream::SinglePrecision);
        double x, y, z, r;

        in >> x >> y >> z >> r;

        if (velodyne_layer() == Layer16) {
            double azimuth = atan2(static_cast<double> (y), static_cast<double>(x));
            azimuth = (azimuth > 0 ? azimuth : (2*M_PI + azimuth));

            if (!init_azimuth) {
                prev_azimuth = azimuth;
                init_azimuth = true;
            }
            else {
                if((azimuth - prev_azimuth) < -0.2) {
                    if(cnt%4 == 0) {
                        velodyne_data_  += single_layer;

                        if(is_write_bin_) {
                            for(auto point:single_layer) {
                                out << point.x << point.y << point.z << point.intensity;
                            }
                        }
                    }
                    cnt++;
                    single_layer.clear();
                }
                prev_azimuth = azimuth;
            }

            Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = r;

            single_layer.push_back(point);
        }
        else if (velodyne_layer() == Layer64) {
            Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = r;

            float u = 0.5* (1.0-atan2(point.y, point.x)/M_PI)*900.0;

            float rr = sqrt(x*x+y*y+z*z);
//            float v = (1.0-(asin(z/r)+3.0*M_PI/180.0)/(28.0*M_PI/180.0))*64.0;
            float v = fabs(1 - (asin(z/rr)*180.0/M_PI+25.0)/29.0)*50.0;

            int iu = static_cast<int>(u);
            int iv = static_cast<int>(v);

            if (iv>=0 && iv<50 && iu >=0 && iv < 900)
            {
                spherical_velodyne_.at<float>(iv, iu) = r;
            }
            else
            {
                cout << iu << ", " << iv << ", " << asin(z/r)*180.0/M_PI << endl;
            }


            velodyne_data_.push_back(point);
        }
    }

    if (velodyne_layer() == Layer16 && cnt%4 == 0) {
        velodyne_data_  += single_layer;
    }


    if(is_write_bin_) {
        out_file.close();
    }
    single_layer.clear();

}

//void KittiData::insert_QVector(QVector<GLfloat>& dst, const QVector<GLfloat>& src)
//{
//    for(int i=0; i<src.size(); i++) {
//        dst.push_back(src[i]);;
//    }
//}
