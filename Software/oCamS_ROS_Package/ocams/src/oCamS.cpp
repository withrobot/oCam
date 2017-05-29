#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include "withrobot_camera.hpp"

namespace Withrobot {


class StereoCamera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;

public:

	/**
	 * @brief      { stereo camera driver }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
	 */
    StereoCamera(int resolution, double frame_rate): camera(NULL) {

        enum_dev_list();

        camera = new Withrobot::Camera(devPath_.c_str());

        if (resolution == 0) { width_ = 1280; height_ = 960;}
        if (resolution == 1) { width_ = 1280; height_ = 720;}
        if (resolution == 2) { width_ = 640; height_  = 480;}

        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, (unsigned int)frame_rate);

        /*
         * get current camera format (image size and frame rate)
         */
        camera->get_current_format(camFormat);

        camFormat.print();

        /* Withrobot camera start */
        camera->start();
	}

	~StereoCamera() {
        camera->stop();
        delete camera;

	}

    void enum_dev_list()
    {
        /* enumerate device(UVC compatible devices) list */
        std::vector<Withrobot::usb_device_info> dev_list;
        int dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1) {
            dev_list.clear();

            return;
        }

        for (unsigned int i=0; i < dev_list.size(); i++) {
            if (dev_list[i].product == "oCamS-1CGN-U")
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
        camera->set_control("Exposure, Auto", !ae);

    }

	/**
	 * @brief      Gets the images.
	 *
	 * @param      left_image   The left image
	 * @param      right_image  The right image
	 *
	 * @return     The images.
	 */
	bool getImages(cv::Mat& left_image, cv::Mat& right_image) {

        cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC2);
        cv::Mat dstImg[2];

        if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1) {
            cv::split(srcImg, dstImg);

            right_image = dstImg[0];
            left_image = dstImg[1];

            return true;
        } else {
            return false;
        }
	}

private:
    int width_;
    int height_;
    std::string devPath_;

};

/**
 * @brief       the camera ros warpper class
 */
class oCamStereoROS {
public:

	/**
	 * @brief      { function_description }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
	 */
    oCamStereoROS(ros::NodeHandle nh, ros::NodeHandle priv_nh) {

        /* default parameters */
        resolution_ = 1;
        frame_rate_ = 60.0;
        exposure_ = 100;
        gain_ = 50;
        wb_blue_ = 200;
        wb_red_ = 160;
        autoexposure_= false;
        left_frame_id_ = "left_camera";
        right_frame_id_ = "right_camera";
        show_image_ = true;

        /* get parameters */
        priv_nh.getParam("resolution", resolution_);
        priv_nh.getParam("frame_rate", frame_rate_);
        priv_nh.getParam("exposure", exposure_);
        priv_nh.getParam("gain", gain_);
        priv_nh.getParam("wb_blue", wb_blue_);
        priv_nh.getParam("wb_red", wb_red_);
        priv_nh.getParam("left_frame_id", left_frame_id_);
        priv_nh.getParam("right_frame_id", right_frame_id_);
        priv_nh.getParam("show_image", show_image_);
        priv_nh.getParam("auto_exposure", autoexposure_);

        /* initialize the camera */
        ocams = new StereoCamera(resolution_, frame_rate_);
        ocams->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        ROS_INFO("Initialized the camera");

        // setup publisher stuff
        image_transport::ImageTransport it(nh);
        image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
        image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

        ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
        ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

        sensor_msgs::CameraInfo left_info, right_info;

        ROS_INFO("Loading from ROS calibration files");

        // get config from the left, right.yaml in config
        camera_info_manager::CameraInfoManager info_manager(nh);

        info_manager.setCameraName("left");
        info_manager.loadCameraInfo( "package://ocams/config/left.yaml");
        left_info = info_manager.getCameraInfo();

        info_manager.setCameraName("right");
        info_manager.loadCameraInfo( "package://ocams/config/right.yaml");
        right_info = info_manager.getCameraInfo();

        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;

        std::cout << left_info << std::endl;
        std::cout << right_info << std::endl;

        ROS_INFO("Got camera calibration files");

        // loop to publish images;
        cv::Mat left_image, right_image;
        ros::Rate r(frame_rate_);

        while (ros::ok()) {     // dave
            ros::Time now = ros::Time::now();
            if (!ocams->getImages(left_image, right_image)) {
                usleep(1000);
                continue;
            } else {
                ROS_INFO_ONCE("Success, found camera");
            }

			if (show_image_) {
                cv::imshow("left", left_image);
                cv::imshow("right", right_image);
                cv::waitKey(10);
			}

			if (left_image_pub.getNumSubscribers() > 0) {
				publishImage(left_image, left_image_pub, "left_frame", now);
			}
			if (right_image_pub.getNumSubscribers() > 0) {
				publishImage(right_image, right_image_pub, "right_frame", now);
			}
			if (left_cam_info_pub.getNumSubscribers() > 0) {
				publishCamInfo(left_cam_info_pub, left_info, now);
			}
			if (right_cam_info_pub.getNumSubscribers() > 0) {
				publishCamInfo(right_cam_info_pub, right_info, now);
			}

            r.sleep();
			// since the frame rate was set inside the camera, no need to do a ros sleep
        }
	}

    ~oCamStereoROS() {
        delete ocams;
    }

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
		cv_image.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
		cv_image.header.frame_id = img_frame_id;
		cv_image.header.stamp = t;
		img_pub.publish(cv_image.toImageMsg());
	}

private:
	int resolution_;
	double frame_rate_;
	int exposure_, gain_, wb_blue_, wb_red_;
	bool autoexposure_;
	bool show_image_;
	bool config_changed_;

	std::string left_frame_id_, right_frame_id_;

	StereoCamera* ocams;

};

}

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "ocams");

        ros::NodeHandle nh;
        ros::NodeHandle priv_nh("~");

        Withrobot::oCamStereoROS ocams_ros(nh, priv_nh);

        ros::spin();

        return 0;
    }

    catch(std::runtime_error& e) {
        ros::shutdown();
        return 1;
    }

}
