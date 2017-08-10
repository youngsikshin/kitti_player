#include <string>
#include "MainWindow.h"
#include "ui_MainWindow.h"
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

    speed_ =1.0;

    is_left_color_image_pub_ = false;
    is_right_color_image_pub_ = false;
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
    private_nh.param("right_image_pub", is_right_image_pub_, true);
    private_nh.param("left_color_image_pub", is_left_color_image_pub_, false);
    private_nh.param("right_color_image_pub", is_right_color_image_pub_, false);
    private_nh.param("velodyne_pub", is_velodyne_pub_, true);

//    cout << "left_color: " << is_left_color_image_pub_ << endl;

    data_path_ = QString::fromStdString(str_path_);

    initialize();

    this->nh_ = node;

    it_ = new image_transport::ImageTransport(nh_);
    left_img_pub_ = it_->advertise(str_left_topic_, 10);
    right_img_pub_ = it_->advertise(str_right_topic_, 10);
    left_color_img_pub_ = it_->advertise(str_left_color_topic_, 10);
    right_color_img_pub_ = it_->advertise(str_right_color_topic_, 10);

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(str_velodyne_topic_, 10);

    f_ = boost::bind(&MainWindow::dynamic_parameter_callback, this, _1, _2);
    server_.setCallback(f_);

    spin_thread_ = std::thread(&MainWindow::spinner,this);

//    camlidar_calib_.reset(new camlidar::CamLidarCalib(node, private_nh));
//    connect(camlidar_calib_.get(), SIGNAL(image_signal()), this, SLOT(set_pixmap()));
//    camlidar_thread_ = std::thread(&camlidar::CamLidarCalib::run, camlidar_calib_);
}

void MainWindow::dynamic_parameter_callback(kitti_player::kitti_playerConfig &config, uint32_t level)
{
    speed_ = config.speed;

    ui->startButton->setText("play");
}

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
        publish_image(left_img_pub_, kitti_data_.left_image());
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
//    // Make depthmap
//    Matrix3x4 P0 = kitti_data_.P0();
//    Matrix3x4 P1 = kitti_data_.P1();
//    Matrix3x4 P2 = kitti_data_.P2();
//    Matrix3x4 P3 = kitti_data_.P3();
//    Matrix3x4 Tr = kitti_data_.Tr();

//    cv::Mat left_cvimg = kitti_data_.left_color_image();
//    cv::Mat resized_img;
//    cv::Mat show_img;

//    double scale = 0.45;//0.2;
//    cv::resize(left_cvimg, show_img, cv::Size(), scale, scale);
//    cv::resize(left_cvimg, resized_img, cv::Size(), scale, scale);

//    cv::Mat depth_map = cv::Mat(show_img.size(), CV_32F, cv::Scalar(0));

//    for (auto iter = kitti_data_.velodyne_data().begin(); iter != kitti_data_.velodyne_data().end(); ++iter) {

//        Eigen::Vector4d XYZ_vel (iter->x, iter->y, iter->z, 1.0);
//        Eigen::Vector3d XYZ_cam = Tr*XYZ_vel;
//        Eigen::Vector4d XYZ(XYZ_cam(0), XYZ_cam(1), XYZ_cam(2), 1.0);

//        Eigen::Vector3d xyz = P2 * XYZ;

//        Eigen::Vector2d uv(xyz(0)/xyz(2), xyz(1)/xyz(2));
//        uv.noalias() = uv * scale;

//        int u = static_cast<int> (round(uv(0)));
//        int v = static_cast<int> (round(uv(1)));

//        if (u > 0 && u < show_img.cols && v > 0 && v < show_img.rows && XYZ(2) > 0) {
//            depth_map.at<float> (v, u) = XYZ(2);
//            cv::circle(show_img, cv::Point(u, v), 0.1, cv::Scalar(0, 0, 255), -1);
//        }

//    }


//    cv::namedWindow("test", cv::WINDOW_NORMAL);
//    cv::imshow("test", show_img);
//    cv::waitKey(1);

//    depth_map.convertTo(depth_map, CV_16UC1, 1000.0);

//    QString depthmap_fname = "d_" + QString::number(index_manager.index())+".png";
//    QString resized_fname = QString::number(index_manager.index())+".png";

////    cv::Rect rect(0, 27, resized_img.cols, resized_img.rows-27);

//    cv::imwrite(depthmap_fname.toStdString(), depth_map);
//    cv::imwrite(resized_fname.toStdString(), resized_img);

}

void MainWindow::publish_image(image_transport::Publisher& img_pub, cv::Mat& img)
{
    cv_bridge::CvImage cv_image;
    cv_image.header.seq = index_manager.index();
//    cv_image.header.stamp = ros::Time::now();
    cv_image.header.stamp = sync_time_;
    cv_image.header.frame_id = "kitti";
    if(img.type() == CV_8UC1)
        cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    else if(img.type() == CV_8UC3)
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;

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
    out_pc.header.frame_id = "velodyne";
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
