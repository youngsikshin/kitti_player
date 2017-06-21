#include <iostream>
#include <QDir>
#include <QTextStream>
#include <QDataStream>

#include <QDebug>

#include <kitti_player/KittiData.h>

using namespace std;

KittiData::KittiData()
{
    str_seq_ = "00";
}

KittiData::KittiData(QString path)
{
//    cerr << "[KittiData]\t Called KittiData(QString path)" << endl;
    path_ = path;
    str_seq_ = "00";

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
    if (!velodyne_file.open(QIODevice::ReadOnly))
        return;

    QDataStream in(&velodyne_file);
    velodyne_data_.clear();

    int cnt = 0;

    bool init_azimuth = false;
    double prev_azimuth;

    PointCloud single_layer;

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

            velodyne_data_.push_back(point);
        }
    }

    if (velodyne_layer() == Layer16 && cnt%4 == 0) {
        velodyne_data_  += single_layer;
    }

    single_layer.clear();

}

//void KittiData::insert_QVector(QVector<GLfloat>& dst, const QVector<GLfloat>& src)
//{
//    for(int i=0; i<src.size(); i++) {
//        dst.push_back(src[i]);;
//    }
//}
