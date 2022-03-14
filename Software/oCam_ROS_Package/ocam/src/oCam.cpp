#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <ocam/camConfig.h>
#include <boost/thread.hpp>

#include "withrobot_camera.hpp"

class Camera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;

private:
    int width_;
    int height_;
    int dev_num;
    bool flag0144 = false;
    std::vector<Withrobot::usb_device_info> dev_list;
    std::string devPath_;

public:

    Camera(int resolution, double frame_rate): camera(NULL) {
        
        enum_dev_list(dev_list);

        camera = new Withrobot::Camera(devPath_.c_str());

        for (int i=0; i < dev_num; i++) {
            if (flag0144 = true)
            {
                if (resolution == 0) { width_ = 1280; height_ = 800;}
                if (resolution == 1) { width_ = 1280; height_ = 720;}
                if (resolution == 2) { width_ = 640; height_  = 480;}
                if (resolution == 3) { width_ = 640; height_  = 400;}
                if (resolution == 4) { width_ = 320; height_  = 240;}
            }
            else{
                if (resolution == 0) { width_ = 1280; height_ = 960;}
                if (resolution == 1) { width_ = 1280; height_ = 720;}
                if (resolution == 2) { width_ = 640; height_  = 480;}
                if (resolution == 3) { width_ = 320; height_  = 240;}
            }
            
        }
        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','R','B','G'), 1, (unsigned int)frame_rate);

        /*
         * get current camera format (image size and frame rate)
         */
        camera->get_current_format(camFormat);

        camFormat.print();

        /* Withrobot camera start */
        camera->start();
	}

    ~Camera() {
        camera->stop();
        delete camera;

	}

    void enum_dev_list(std::vector<Withrobot::usb_device_info> dev_list)
    {
        /* enumerate device(UVC compatible devices) list */
        // std::vector<Withrobot::usb_device_info> dev_list;
        dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1) {
            dev_list.clear();

            return;
        }

        for (unsigned int i=0; i < dev_list.size(); i++) {
	    
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
            else if (dev_list[i].product == "oCam-1CGN-U-T2")
            {
                devPath_ = dev_list[i].dev_node;
                flag0144 = true;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U-T2")
            {
                devPath_ = dev_list[i].dev_node;
                flag0144 = true;
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

    bool getImages(cv::Mat &image) {

        cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
        cv::Mat dstImg;

        if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1)
        {
            cvtColor(srcImg, dstImg, cv::COLOR_BayerGR2RGB);
            image = dstImg;

            return true;
        } else {
            return false;
        }
	}
};

/**
 * @brief       the camera ros warpper class
 */
class oCamROS {

private:
    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;
    bool show_image_;
    bool config_changed_;

    ros::NodeHandle nh;
    std::string camera_frame_id_;
    Camera* ocam;
    /**
     * @brief      { publish camera info }
     *
     * @param[in]  pub_cam_info  The pub camera information
     * @param[in]  cam_info_msg  The camera information message
     * @param[in]  now           The now
     */
    void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now) {
        cam_info_msg.header.stamp = now;
        pub_cam_info.publish(cam_info_msg);
    }

    /**
     * @brief      { publish image }
     *
     * @param[in]  img           The image
     * @param      img_pub       The image pub
     * @param[in]  img_frame_id  The image frame identifier
     * @param[in]  t             { parameter_description }
     */
    void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t) {
        cv_bridge::CvImage cv_image;
        cv_image.image = img;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.frame_id = img_frame_id;
        cv_image.header.stamp = t;
        img_pub.publish(cv_image.toImageMsg());
    }

    void device_poll() {
        //Reconfigure confidence
        dynamic_reconfigure::Server<ocam::camConfig> server;
        dynamic_reconfigure::Server<ocam::camConfig>::CallbackType f;
        f = boost::bind(&oCamROS::callback, this ,_1, _2);
        server.setCallback(f);

        // setup publisher stuff
        image_transport::ImageTransport it(nh);
        image_transport::Publisher camera_image_pub = it.advertise("camera/image_raw", 1);

        ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);

        sensor_msgs::CameraInfo camera_info;

        ROS_INFO("Loading from ROS calibration files");


        // get config from the left, right.yaml in config
        camera_info_manager::CameraInfoManager info_manager(nh);

        info_manager.setCameraName("camera");
        info_manager.loadCameraInfo( "package://ocam/config/camera.yaml");
        camera_info = info_manager.getCameraInfo();

        camera_info.header.frame_id = camera_frame_id_;

        ROS_INFO("Got camera calibration files");

        // loop to publish images;
        cv::Mat camera_image;

        ros::Rate r(frame_rate_);

        while (ros::ok())
        {
            ros::Time now = ros::Time::now();

            if (!ocam->getImages(camera_image)) {
                usleep(1000);
                continue;
            } else {
                ROS_INFO_ONCE("Success, found camera");
            }

            if (camera_image_pub.getNumSubscribers() > 0) {
                publishImage(camera_image, camera_image_pub, "camera_frame", now);
            }

            if (camera_info_pub.getNumSubscribers() > 0) {
                publishCamInfo(camera_info_pub, camera_info, now);
            }


            if (show_image_) {
                cv::imshow("image", camera_image);
                cv::waitKey(10);
            }

            r.sleep();
        }
    }

    void callback(ocam::camConfig &config, uint32_t level) {
        ocam->uvc_control(config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
    }


public:
    /**
	 * @brief      { function_description }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
     */
    oCamROS() {
        ros::NodeHandle priv_nh("~");

        /* default parameters */
        resolution_ = 0;
        frame_rate_ = 60.0;
        exposure_ = 100;
        gain_ = 150;
        wb_blue_ = 200;
        wb_red_ = 160;
        autoexposure_= false;
        camera_frame_id_ = "camera";
        show_image_ = true;

        /* get parameters */
        priv_nh.getParam("resolution", resolution_);
        priv_nh.getParam("frame_rate", frame_rate_);
        priv_nh.getParam("exposure", exposure_);
        priv_nh.getParam("gain", gain_);
        priv_nh.getParam("wb_blue", wb_blue_);
        priv_nh.getParam("wb_red", wb_red_);
        priv_nh.getParam("camera_frame_id", camera_frame_id_);
        priv_nh.getParam("show_image", show_image_);
        priv_nh.getParam("auto_exposure", autoexposure_);

        /* initialize the camera */
        ocam = new Camera(resolution_, frame_rate_);
        ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        ROS_INFO("Initialized the camera");

        // thread
        boost::shared_ptr<boost::thread> device_poll_thread;
        device_poll_thread = boost::shared_ptr<boost::thread>(new boost::thread(&oCamROS::device_poll, this));
	}

    ~oCamROS() {
        delete ocam;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ocam");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    oCamROS ocam_ros;

    ros::spin();

    return 0;
}

