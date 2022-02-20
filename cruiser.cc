#include <iostream>
#include "paddle_api.h"
#include <arm_neon.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "math.h"
#include <sys/time.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <limits>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <map>
#include <fstream>
using namespace cv;
using namespace std;
using namespace ros;
float myMax(int var1, int var2){

	if (var1 > var2){
		return var1;
	}else {
		return var2;
	}
}

float myMin(int var1, int var2){

	if (var1 > var2){
		return var2;
	}else {
		return var1;
	}
}
std_msgs::Int32 lane_vel;   
std_msgs::Int32 speed;
std_msgs::Int32 gear;         
const std::vector<int64_t> INPUT_SHAPE = {1, 3, 128, 128};
const std::vector<float> INPUT_MEAN = {0.5f, 0.5f, 0.5f};
const std::vector<float> INPUT_STD = {1.0f, 1.0f, 1.0f};
bool isSidewalk=false;
inline int64_t get_current_us() {
    struct timeval time;
    gettimeofday(&time, NULL);
    return 1000000LL * (int64_t)time.tv_sec + (int64_t)time.tv_usec;
}
void gameStage_callback(const std_msgs::Int32::ConstPtr &msg){
    
        if(msg->data == 4||msg->data==8||msg->data==9||msg->data==5||msg->data==15){
            isSidewalk = true;
            cout<<"isSidewalk=ture"<<endl;
        }

        else{
            isSidewalk = false;
            cout<<"isSidewalk=False"<<endl;
        }

}
void preprocess(cv::Mat &input_image, const std::vector<float> &input_mean,
                const std::vector<float> &input_std, int input_width,
                int input_height, float *input_data) {
  cv::Mat resize_image;
  cv::resize(input_image, resize_image, cv::Size(input_width, input_height), 0, 0);
  cv::Mat norm_image;
  resize_image.convertTo(norm_image, CV_32FC3, 1/255.f);
  // NHWC->NCHW
  int image_size = input_height * input_width;
  const float *image_data = reinterpret_cast<const float *>(norm_image.data);
  float32x4_t vmean0 = vdupq_n_f32(input_mean[0]);
  float32x4_t vmean1 = vdupq_n_f32(input_mean[1]);
  float32x4_t vmean2 = vdupq_n_f32(input_mean[2]);
  float32x4_t vscale0 = vdupq_n_f32(1.0f / input_std[0]);
  float32x4_t vscale1 = vdupq_n_f32(1.0f / input_std[1]);
  float32x4_t vscale2 = vdupq_n_f32(1.0f / input_std[2]);
  float *input_data_c0 = input_data;
  float *input_data_c1 = input_data + image_size;
  float *input_data_c2 = input_data + image_size * 2;
  int i = 0;
  for (; i < image_size - 3; i += 4) {
    float32x4x3_t vin3 = vld3q_f32(image_data);
    float32x4_t vsub0 = vsubq_f32(vin3.val[0], vmean0);
    float32x4_t vsub1 = vsubq_f32(vin3.val[1], vmean1);
    float32x4_t vsub2 = vsubq_f32(vin3.val[2], vmean2);
    float32x4_t vs0 = vmulq_f32(vsub0, vscale0);
    float32x4_t vs1 = vmulq_f32(vsub1, vscale1);
    float32x4_t vs2 = vmulq_f32(vsub2, vscale2);
    vst1q_f32(input_data_c0, vs0);
    vst1q_f32(input_data_c1, vs1);
    vst1q_f32(input_data_c2, vs2);
    image_data += 12;
    input_data_c0 += 4;
    input_data_c1 += 4;
    input_data_c2 += 4;
  }
  for (; i < image_size; i++) {
    *(input_data_c0++) = (*(image_data++)-input_mean[0]) / input_std[0];
    *(input_data_c1++) = (*(image_data++)-input_mean[1]) / input_std[1];
    *(input_data_c2++) = (*(image_data++)-input_mean[2]) / input_std[2];
  }
}

int main(int argc, char** argv){

    //预加载循线模型
    paddle::lite_api::MobileConfig configLineDetection;
    configLineDetection.set_model_from_file("./models/lineDetection.nb");
    configLineDetection.set_power_mode(paddle::lite_api::PowerMode::LITE_POWER_HIGH);
    configLineDetection.set_threads(2);
    std::shared_ptr<paddle::lite_api::PaddlePredictor> predictorLineDetection = paddle::lite_api::CreatePaddlePredictor<paddle::lite_api::MobileConfig>(configLineDetection);
    //预加载停车模型
    paddle::lite_api::MobileConfig configPark;
    configPark.set_model_from_file("./models/Park.nb");
    configPark.set_power_mode(paddle::lite_api::PowerMode::LITE_POWER_HIGH);
    configPark.set_threads(2);
    std::shared_ptr<paddle::lite_api::PaddlePredictor> predictorPark = paddle::lite_api::CreatePaddlePredictor<paddle::lite_api::MobileConfig>(configPark);

    Mat pre_img;
    
    ros::init(argc, argv, "cruiser");    // 创建节点
    ros::NodeHandle nh;                                       // 定义句柄
    ros::Publisher laneVel_pub = nh.advertise<std_msgs::Int32>("/lane_vel", 1);

/*
    ros::Publisher laneVel_pub = nh.advertise<std_msgs::Int32>("/auto_driver/send/direction", 1);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Int32>("/auto_driver/send/speed", 1);
    ros::Publisher gear_pub = nh.advertise<std_msgs::Int32>("/auto_driver/send/gear", 1);
*/
    ros::Subscriber gameStage = nh.subscribe("/gameStage", 10, gameStage_callback);


    VideoCapture cap("/dev/video10");
    cap.set(CAP_PROP_FRAME_WIDTH,640);
    cap.set(CAP_PROP_FRAME_HEIGHT,480);
    cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
    cap.set(CAP_PROP_BRIGHTNESS, 0.7);
    cap.set(CAP_PROP_CONTRAST, 1);//
    cap.set(CAP_PROP_SATURATION, 1);//
    cap.set(CAP_PROP_EXPOSURE, 0.01);//exposure time
    bool ifsuccess = cap.read(pre_img);

    if (!cap.isOpened())
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    if (!ifsuccess)
    {
        cout << "Cannot read a frame from video stream" << endl;
	return -1;
    }
    int input_width = INPUT_SHAPE[3];
    int input_height = INPUT_SHAPE[2];
    std::unique_ptr<paddle::lite_api::Tensor> input_tensor_linedetection(std::move(predictorLineDetection->GetInput(0)));
    input_tensor_linedetection->Resize(INPUT_SHAPE);
    auto* input_data_linedetection = input_tensor_linedetection->mutable_data<float>();

    std::unique_ptr<paddle::lite_api::Tensor> input_tensor_park(std::move(predictorPark->GetInput(0)));
    input_tensor_park->Resize(INPUT_SHAPE);
    auto* input_data_park = input_tensor_park->mutable_data<float>();

    double postprocess_start_time = get_current_us();
    float pre_angle = 50;
    float now_angle = 50;
    int turn_angle = 50;
/*
    speed.data = 50;
    speed_pub.publish(speed);
    gear.data = 1;
    gear_pub.publish(gear);
*/

    Rect img_select;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9), Point(-1, -1));
    Mat img;
    Mat post_img;
    Mat imgLab;
    Mat imgBinary;
    Mat imgInput;
    int i=0;
    float Kp=1;
    float Ki=0.02;
    float kd=0;
    float lastErr=0;
    float last2Err=0;

    while(ros::ok()){
        
        cap.read(pre_img);

        img_select = Rect(0, 270, 640, 210);
        post_img = pre_img(img_select);
        vector<Mat> Lab;
        vector<Mat> Binary(3);
        cv::cvtColor(post_img,imgLab,COLOR_BGR2Lab);
        split(imgLab,Lab);
        blur(Lab[2], Lab[2], cv::Size(11, 11));
        GaussianBlur(Lab[2], Lab[2], cv::Size(9, 9), 0, 0); 
        if(!isSidewalk){
            inRange(Lab[2], 145, 165, imgBinary);
        }
        else{
            inRange(Lab[2], 163, 200, imgBinary);
        }
        //inRange(Lab[2], 160, 200, imgBinary);
        dilate(imgBinary, imgBinary, kernel);
        Binary[0]=imgBinary;
        Binary[1]=imgBinary;
        Binary[2]=imgBinary;
        merge(Binary,imgInput);
        preprocess(imgInput, INPUT_MEAN, INPUT_STD, input_width, input_height,input_data_linedetection);
        predictorLineDetection->Run();
        std::unique_ptr<const paddle::lite_api::Tensor> output_tensor_linedetection(std::move(predictorLineDetection->GetOutput(0)));
        now_angle = 50 * output_tensor_linedetection->data<float>()[0] + 50;

        //float error_angle = now_angle-pre_angle;
        int turn_angle=now_angle;
        //turn_angle = pre_angle+Kp*(error_angle-lastErr)+Ki*error_angle+kd*(error_angle-2*lastErr+last2Err);
        //turn_angle=MAX(3,MIN(85,turn_angle));
        /*
        if (abs(turn_angle-now_angle)<5||abs(turn_angle-now_angle)>20)
        {
            turn_angle=now_angle;
            last2Err=lastErr;
            lastErr=error_angle;
        }
        */
        //namedWindow("image",1);
        //imshow("image", imgBinary);
        //waitKey(1);
        ros::spinOnce();
        //turn_angle = now_angle;
        lane_vel.data = turn_angle;
        laneVel_pub.publish(lane_vel);
        pre_angle = now_angle;
        //cout<<isSidewalk<<endl;
        std::cout << turn_angle << std::endl;
        /*
        if (now_angle >= 50)
        {
            int turn_angle = now_angle;
            lane_vel.data = turn_angle;
            laneVel_pub.publish(lane_vel);
            pre_angle = now_angle;
            std::cout << turn_angle << std::endl;
            i++;
        }
        else {
            int turn_angle = 0.5 * now_angle + 0.5 * pre_angle;
            lane_vel.data = turn_angle;
            laneVel_pub.publish(lane_vel);
            pre_angle = now_angle;
            std::cout << turn_angle << std::endl;
            i++;
        }
 */       
//        int turn_angle = 50 * output_tensor->data<float>()[0] + 50;
//        std::cout << i << std::endl;
        //cout << cap.get(CAP_PROP_AUTO_EXPOSURE) << endl;
        //cout << cap.get(CAP_PROP_EXPOSURE) << endl;

        //json[i] = turn_angle;
        i++;
    }
    double postprocess_end_time = get_current_us();
    double ms = postprocess_end_time - postprocess_start_time;
    std::cout << i/(ms/1000000.0f) << std::endl;
    cap.release();
    //cvDestroyAllWindows();
        return 0;
}

