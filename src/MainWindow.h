#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>
#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <QTimer>

#include <ros/ros.h>
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

    void initialize();
    void reset_sequence();
    void load_data();
//    inline const QString& sequence_path() { return data_path_+"sequences/"+str_seq_+"/"; }
//    inline const QString& gt_fname() { return data_path_+"poses/"+str_seq_+".txt"; }
//    camlidar::CamLidarCalib::Ptr camlidar_calib_;
//    std::thread camlidar_thread_;

    QPixmap image_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;

private slots:
    void set_pixmap();
};

#endif // MAINWINDOW_H
