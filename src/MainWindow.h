#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <QTimer>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <kitti_player/kitti_playerConfig.h>

#include <kitti_player/KittiData.h>
#include <kitti_player/Datatypes.h>

using namespace std;

namespace Ui {
class MainWindow;
}

class UniqueId
{
public:
  UniqueId():unique_id(0)
  {

  }
  int index() { return unique_id; }
  void init() { unique_id = 0; }
  void inc() { ++unique_id; }
private:
  int unique_id;
};

static UniqueId index_manager;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void ros_init(ros::NodeHandle node, ros::NodeHandle private_nh);

    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event);

private slots:
    void onTimer();
    void on_comboBox_currentIndexChanged(int index);
    void on_layerSelector16_clicked();
    void on_layerSelector64_clicked();
    void on_startButton_clicked();
    void on_stepButton_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *timer_;

    QString data_path_;
    QString str_seq_;
    KittiData kitti_data_;

    int delay_ms_;
    double speed_;

    ///Pointer to dynamic reconfigure service srv_
    std::thread spin_thread_;
    void spinner() { ros::spin(); }

    dynamic_reconfigure::Server<kitti_player::kitti_playerConfig> server_;
    dynamic_reconfigure::Server<kitti_player::kitti_playerConfig>::CallbackType f_;
    kitti_player::kitti_playerConfig config_;

    void dynamic_parameter_callback(kitti_player::kitti_playerConfig &config, uint32_t level);


    void initialize();
    void reset_sequence();
    void load_data();

//    inline const QString& sequence_path() { return data_path_+"sequences/"+str_seq_+"/"; }
//    inline const QString& gt_fname() { return data_path_+"poses/"+str_seq_+".txt"; }
//    camlidar::CamLidarCalib::Ptr camlidar_calib_;
//    std::thread camlidar_thread_;

    QPixmap image_;

    std::string str_path_;
    std::string str_left_topic_;
    std::string str_right_topic_;
    std::string str_left_color_topic_;
    std::string str_right_color_topic_;
    std::string str_velodyne_topic_;
    std::string str_depth_map_topic_;

    bool is_left_image_pub_;
    bool is_right_image_pub_;
    bool is_left_color_image_pub_;
    bool is_right_color_image_pub_;
    bool is_velodyne_pub_;
    bool is_depth_map_pub_;

    ros::NodeHandle nh_;
    ros::Publisher pc_pub_;

    image_transport::ImageTransport *it_;
    image_transport::Publisher left_img_pub_;
    image_transport::Publisher right_img_pub_;
    image_transport::Publisher left_color_img_pub_;
    image_transport::Publisher right_color_img_pub_;
    image_transport::Publisher depth_map_pub_;

    void publish_image(image_transport::Publisher& img_pub, cv::Mat& img);
    void publish_velodyne(ros::Publisher& pc_pub, PointCloud& pc);

    ros::Time sync_time_;


private slots:
    void set_pixmap();
};

#endif // MAINWINDOW_H
