#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <ocam/camConfig.h>
// #include <dynamic_reconfigure/server.h>

#include "withrobot_camera.hpp"

class Camera
{
    Withrobot::Camera *camera;
    Withrobot::camera_format camFormat;

private:
    int width_;
    int height_;
    std::string devPath_;

public:
    Camera(int resolution, double frame_rate) : camera(NULL)
    {

        enum_dev_list();

        camera = new Withrobot::Camera(devPath_.c_str());

        if (resolution == 0)
        {
            width_ = 1280;
            height_ = 960;
        }
        if (resolution == 1)
        {
            width_ = 1280;
            height_ = 720;
        }
        if (resolution == 2)
        {
            width_ = 640;
            height_ = 480;
        }
        if (resolution == 3)
        {
            width_ = 320;
            height_ = 240;
        }

        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('G', 'R', 'B', 'G'), 1, (unsigned int)frame_rate);

        /*
         * get current camera format (image size and frame rate)
         */
        camera->get_current_format(camFormat);

        camFormat.print();

        /* Withrobot camera start */
        camera->start();
    }

    ~Camera()
    {
        camera->stop();
        delete camera;
    }

    void enum_dev_list()
    {
        /* enumerate device(UVC compatible devices) list */
        std::vector<Withrobot::usb_device_info> dev_list;
        int dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1)
        {
            dev_list.clear();

            return;
        }

        for (unsigned int i = 0; i < dev_list.size(); i++)
        {
            if (dev_list[i].product == "oCam-1CGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1CGN-U-T")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U-T")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
        }
    }

    void uvc_control(int exposure, int gain, int blue, int red, bool ae)
    {
        /* Exposure Setting */
        camera->set_control("Exposure (Absolute)", exposure);

        /* Gain Setting */
        camera->set_control("Gain", gain);

        /* White Balance Setting */
        camera->set_control("White Balance Blue Component", blue);
        camera->set_control("White Balance Red Component", red);

        /* Auto Exposure Setting */
        if (ae)
            camera->set_control("Exposure, Auto", 0x3);
        else
            camera->set_control("Exposure, Auto", 0x1);
    }

    bool getImages(cv::Mat &image)
    {

        cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
        cv::Mat dstImg;

        if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1)
        {
            cvtColor(srcImg, dstImg, cv::COLOR_BayerGR2RGB);
            image = dstImg;

            return true;
        }
        else
        {
            return false;
        }
    }
};

/**
 * @brief       the camera ros warpper class
 */

class oCamROS : public rclcpp::Node
{
private:
    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;
    bool show_image_;
    bool config_changed_; // 이거 쓰이는데가 없는데..?

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    void publishCamInfo(sensor_msgs::msg::CameraInfo &cam_info_msg, rclcpp::Time now)
    {
        cam_info_msg.header.stamp = now;
        camera_info_pub_->publish(cam_info_msg);
    }

    image_transport::Publisher camera_image_pub_;
    void publishImage(cv::Mat img, std::string img_frame_id, rclcpp::Time t)
    {
        cv_bridge::CvImage cv_image;
        cv_image.image = img;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.frame_id = img_frame_id;
        cv_image.header.stamp = t;
        camera_image_pub_.publish(cv_image.toImageMsg());
    }

    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
    std::string camera_frame_id_;
    std::shared_ptr<Camera> ocam;

    void device_poll()
    {
        // Setup camera info
        sensor_msgs::msg::CameraInfo camera_info = info_manager_->getCameraInfo();
        camera_info.header.frame_id = camera_frame_id_;

        cv::Mat camera_image;
        rclcpp::Rate rate(frame_rate_);

        while (rclcpp::ok())
        {
            auto now = this->now();

            if (!ocam->getImages(camera_image))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            else
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Success, found camera");
            }

            if (camera_image_pub_.getNumSubscribers() > 0)
            {
                publishImage(camera_image, "camera_frame", now);
            }

            if (camera_info_pub_->get_subscription_count() > 0)
            {
                publishCamInfo(camera_info, now);
            }

            if (show_image_)
            {
                cv::imshow("image", camera_image);
                cv::waitKey(10);
            }

            rate.sleep();
        }
    }

public:
    oCamROS() : Node("ocam_ros")
    {
        /* default parameters */
        resolution_ = 2;
        frame_rate_ = 30.0;
        exposure_ = 100;
        gain_ = 150;
        wb_blue_ = 200;
        wb_red_ = 160;
        autoexposure_ = false;
        camera_frame_id_ = "camera";
        show_image_ = true;

        /* get parameters */
        this->declare_parameter("resolution", resolution_);
        this->declare_parameter("frame_rate", frame_rate_);
        this->declare_parameter("exposure", exposure_);
        this->declare_parameter("gain", gain_);
        this->declare_parameter("wb_blue", wb_blue_);
        this->declare_parameter("wb_red", wb_red_);
        this->declare_parameter("camera_frame_id", camera_frame_id_);
        this->declare_parameter("show_image", show_image_);
        this->declare_parameter("auto_exposure", autoexposure_);

        this->get_parameter("resolution", resolution_);
        this->get_parameter("frame_rate", frame_rate_);
        this->get_parameter("exposure", exposure_);
        this->get_parameter("gain", gain_);
        this->get_parameter("wb_blue", wb_blue_);
        this->get_parameter("wb_red", wb_red_);
        this->get_parameter("camera_frame_id", camera_frame_id_);
        this->get_parameter("show_image", show_image_);
        this->get_parameter("auto_exposure", autoexposure_);

        /* initialize the camera */
        ocam = std::make_shared<Camera>(resolution_, frame_rate_);
        ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        RCLCPP_INFO(this->get_logger(), "Initialized the camera");

        // Initialize image transport
        image_transport::ImageTransport it(shared_from_this());
        camera_image_pub_ = it.advertise("camera/image_raw", 1);

        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);
        info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(shared_from_this(), "camera", "package://ocam/config/camera.yaml");

        // Start device poll in a separate thread
        std::thread(&oCamROS::device_poll, this).detach();
    }

    ~oCamROS()
    {
        // Clean up resources
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<oCamROS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
