#include <string>
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <ros/package.h>
//#include "odomproblem.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), kitti_data_()
{
    ui->setupUi(this);

    // QTimer initialization
    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(onTimer()));
    timer_->stop();

    speed_ = 1.0;

}

void MainWindow::initialize()
{
    kitti_data_.path(data_path_);

    ui->pathLabel->setText(data_path_);

    // set initial velodyne layer
    ui->layerSelector64->setChecked(true);
    if(ui->layerSelector64->isChecked()) kitti_data_.velodyne_layer(Layer64);
    if(ui->layerSelector16->isChecked()) kitti_data_.velodyne_layer(Layer16);

    // set initial sequence to 00
    reset_sequence();
}

void MainWindow::ros_init(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    private_nh.param("data_path", str_path_, std::string("/var/data/kitti/dataset/"));

    private_nh.param("left_topic", str_left_topic_, std::string("/kitti/left_image"));
    private_nh.param("right_topic", str_right_topic_, std::string("/kitti/right_image"));
    private_nh.param("left_color_topic", str_left_color_topic_, std::string("/kitti/left_color_image"));
    private_nh.param("right_color_topic", str_right_color_topic_, std::string("/kitti/right_color_image"));
    private_nh.param("velodyne_topic", str_velodyne_topic_, std::string("/kitti/velodyne_points"));

    private_nh.param("left_image_pub", is_left_image_pub_, true);
    private_nh.param("right_image_pub", is_right_image_pub_, false);
    private_nh.param("left_color_image_pub", is_left_color_image_pub_, false);
    private_nh.param("right_color_image_pub", is_right_color_image_pub_, false);

    private_nh.param("velodyne_pub", is_velodyne_pub_, true);

//    cout << "left_color: " << is_left_color_image_pub_ << endl;

    data_path_ = QString::fromStdString(str_path_);
    std::string pkg_path = ros::package::getPath("kitti_player");

    initialize();

    this->nh_ = node;

    it_ = new image_transport::ImageTransport(nh_);

    if(is_left_image_pub_)
      left_img_pub_ = it_->advertise(str_left_topic_+"/image_rect", 10);

    if(is_right_image_pub_)
      right_img_pub_ = it_->advertise(str_right_topic_+"/image_rect", 10);

    if(is_left_color_image_pub_)
      left_color_img_pub_ = it_->advertise(str_left_color_topic_+"/image_rect", 10);

    if(is_right_color_image_pub_)
      right_color_img_pub_ = it_->advertise(str_right_color_topic_+"/image_rect", 10);

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(str_velodyne_topic_, 10);

    spin_thread_ = std::thread(&MainWindow::spinner,this);

}

//void MainWindow::dynamic_parameter_callback(kitti_player::kitti_playerConfig &config, uint32_t level)
//{
//    speed_ = config.speed;

//    ui->startButton->setText("play");
//}

MainWindow::~MainWindow()
{
    ros::shutdown();
    spin_thread_.join();
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void MainWindow::reset_sequence()
{
    ui->lineEdit->setText("Sequence " + ui->comboBox->currentText());

    str_seq_ = ui->comboBox->currentText();
    kitti_data_.set_sequence(str_seq_);

    index_manager.init();

    // For slider bar
    ui->dataProgress->setMaximum(kitti_data_.data_length());
}

void MainWindow::load_data()
{
    sync_time_ = ros::Time::now();

    if(index_manager.index() >= kitti_data_.data_length()) return;

    if(ui->layerSelector64->isChecked()) kitti_data_.velodyne_layer(Layer64);
    if(ui->layerSelector16->isChecked()) kitti_data_.velodyne_layer(Layer16);

    // Setting data and Publish
    if(is_left_image_pub_) {
        kitti_data_.set_left_image(index_manager.index());
//        publish_image(left_img_pub_, kitti_data_.left_image());
        publish_image(left_img_pub_,kitti_data_.left_image(), str_left_topic_);
    }

    if(is_right_image_pub_) {
        kitti_data_.set_right_image(index_manager.index());
        publish_image(right_img_pub_, kitti_data_.right_image());
    }

    if(is_left_color_image_pub_) {
        kitti_data_.set_left_color_image(index_manager.index());
        publish_image(left_color_img_pub_, kitti_data_.left_color_image());
    }
    if(is_right_color_image_pub_) {
        kitti_data_.set_right_color_image(index_manager.index());
        publish_image(right_color_img_pub_, kitti_data_.right_color_image());
    }

    if(is_velodyne_pub_) {
        kitti_data_.set_velodyne(index_manager.index());
        publish_velodyne(pc_pub_, kitti_data_.velodyne_data());
    }

    // progress slider
    ui->dataProgress->setValue(index_manager.index());

    // For visualization
    QPixmap vis_image;


    QImage left_qimage;

    if(is_left_color_image_pub_)
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.left_color_image());
    else if(is_right_color_image_pub_)
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.right_color_image());
    else if(is_left_image_pub_)
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.left_image());
    else
        left_qimage = KittiData::cv_mat_to_qimage(kitti_data_.right_image());

    vis_image.convertFromImage(left_qimage);

    ui->imageLabel->setPixmap(vis_image.scaledToWidth(ui->imageLabel->width()));

    // increase index
    index_manager.inc();

}

void MainWindow::publish_image(image_transport::Publisher& img_pub, cv::Mat& img)
{
    cv_bridge::CvImage cv_image;
    cv_image.header.seq = index_manager.index();
//    cv_image.header.stamp = ros::Time::now();
    cv_image.header.stamp = sync_time_;
//    cv_image.header.frame_id = "/sensor/camera/grayscale/left";

    if(img.type() == CV_8UC1)
        cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    else if(img.type() == CV_8UC3)
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    else if(img.type() == CV_32F)
        cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    cv_image.image = img;

    img_pub.publish(cv_image.toImageMsg());
}

void MainWindow::publish_image(image_transport::Publisher& img_pub, cv::Mat& img, std::string frame_id)
{
  cv_bridge::CvImage cv_image;
  cv_image.header.seq = index_manager.index();
//    cv_image.header.stamp = ros::Time::now();
  cv_image.header.stamp = sync_time_;
  cv_image.header.frame_id = frame_id;

  if(img.type() == CV_8UC1)
      cv_image.encoding = sensor_msgs::image_encodings::MONO8;
  else if(img.type() == CV_8UC3)
      cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  else if(img.type() == CV_32F)
      cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  cv_image.image = img;

  img_pub.publish(cv_image.toImageMsg());

}

void MainWindow::publish_velodyne(ros::Publisher& pc_pub, PointCloud& pc)
{
    sensor_msgs::PointCloud2 out_pc;
    pcl::toROSMsg(pc, out_pc);

    out_pc.header.seq = index_manager.index();
//    out_pc.header.stamp = ros::Time::now();
    out_pc.header.stamp = sync_time_;
    out_pc.header.frame_id = "/velodyne";
    pc_pub_.publish(out_pc);
}

void MainWindow::set_pixmap()
{
////    cerr << "[MainWindow]\t Called set_pixmap()" << endl;
//    cv::Mat image = camlidar_calib_->frame();
//    QImage qimage;

//    switch (image.type()) {
//    case CV_8UC1:
//      qimage = QImage(image.data, image.cols, image.rows, static_cast<int> (image.step), QImage::Format_Grayscale8);
//      break;
//    case CV_8UC3:
//      qimage = QImage(image.data, image.cols, image.rows, static_cast<int> (image.step), QImage::Format_RGB888);
//      qimage = qimage.rgbSwapped();
//    default:
//      break;
//    }

//    image_ = QPixmap::fromImage(qimage);
//    ui->imageLabel->setPixmap(image_.scaledToWidth(ui->imageLabel->width()));
}

void MainWindow::onTimer()
{
    load_data();

    if(!ui->startButton->text().compare("play"))  timer_->stop();
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    reset_sequence();
}

void MainWindow::on_layerSelector64_clicked()
{
    kitti_data_.velodyne_layer(Layer64);
}

void MainWindow::on_layerSelector16_clicked()
{
    kitti_data_.velodyne_layer(Layer16);
}

void MainWindow::on_startButton_clicked()
{

    if(!ui->startButton->text().compare("play")) {
        ui->startButton->setText("stop");

        delay_ms_ = static_cast<int> (kitti_data_.get_time_diff(index_manager.index())*1000);
        int scaled_time = static_cast<int> (static_cast<double>(delay_ms_ ) / speed_);

        timer_->start(scaled_time);

//        load_data();
    }
    else if(!ui->startButton->text().compare("stop")) {
        ui->startButton->setText("play");
//        timer_->stop();
    }

}

void MainWindow::on_stepButton_clicked()
{
    load_data();
}

void MainWindow::on_checkBoxBinary_clicked()
{
    if(ui->checkBoxBinary->checkState())
        kitti_data_.set_write_bin(true);
    else
        kitti_data_.set_write_bin(false);
}
