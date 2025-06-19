#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <string>

using namespace std;
using namespace cv;

std::string IMAGE_TOPIC;
std::string dataset;


ros::Publisher pub_image;
cv::Mat lut_gamma_1_4;
cv::Mat lut_gamma_1_2;
cv::Mat lut_gamma_2_1;
cv::Mat lut_gamma_4_1;
float g[256];
bool modelFlag = true;
cv::Mat lut_EI_1_4;
cv::Mat lut_EI_1_2;
cv::Mat lut_EI_2_1;
cv::Mat lut_EI_4_1;
int get_near_index(float value)
{
    int ret_index;                      
    int mid_index;                      
    int left_index;                     
    int right_index;                    
    float left_abs;                     
    float right_abs;                    

    ret_index = 0;
    left_index = 0;
    right_index = 256 - 1;
    mid_index = 0;
    left_abs = 0;
    right_abs = 0;

    while (left_index != right_index) {
        mid_index = (right_index + left_index) / 2;
        if (value <= g[mid_index]) {
            right_index = mid_index;
        }
        else {
            left_index = mid_index;
        }
        if (right_index - left_index < 2) {
            break;
        }
    }
    left_abs = fabs(g[left_index] - value);
    right_abs = fabs(g[right_index] - value);
    ret_index = right_abs <= left_abs ? right_index : left_index;
    return ret_index;
}

void correct_vector_creat(uchar* gamma_i, float num)
{
    float temp;
    int xx;
    for (int i = 0; i <= 255; i++)
    {
        temp = g[i] + log(num);
        if (temp >= 2.0842)
            temp = 2.0842;
        else if (temp <= -5.1898)
            temp = -5.1898;
        xx = get_near_index(temp);
        *(gamma_i + i) = xx;
    }
}
void process(std::vector<cv::Mat> images, cv::Mat& dst)
{
    int channels = images[0].channels();
    CV_Assert(channels == 1 || channels == 3);
    Size size = images[0].size();
    int CV_32FCC = CV_MAKETYPE(CV_32F, channels);

    std::vector<cv::Mat> weights(images.size());
    cv::Mat weight_sum = cv::Mat::zeros(size, CV_32F);

    for (size_t i = 0; i < images.size(); i++) {
        cv::Mat img, gray, contrast, saturation, wellexp;
        std::vector<Mat> splitted(channels);

        images[i].convertTo(img, CV_32F, 1.0f / 255.0f);
        if (channels == 3) {
            cvtColor(img, gray, COLOR_RGB2GRAY);
        }
        else {
            img.copyTo(gray);
        }
        cv::split(img, splitted);

        cv::Laplacian(gray, contrast, CV_32F);
        contrast = cv::abs(contrast);

        Mat mean = cv::Mat::zeros(size, CV_32F);
        for (int c = 0; c < channels; c++) {
            mean += splitted[c];
        }
        mean /= channels;

        saturation = cv::Mat::zeros(size, CV_32F);
        for (int c = 0; c < channels; c++) {
            cv::Mat deviation = splitted[c] - mean;
            pow(deviation, 2.0f, deviation);
            saturation += deviation;
        }
        sqrt(saturation, saturation);

        wellexp = cv::Mat::ones(size, CV_32F);
        for (int c = 0; c < channels; c++) {
            cv::Mat expo = splitted[c] - 0.5f;
            pow(expo, 2.0f, expo);
            expo = -expo / 0.08f;
            exp(expo, expo);
            wellexp = wellexp.mul(expo);
        }

        pow(contrast, 1, contrast);
        pow(saturation, 1, saturation);
        pow(wellexp, 0, wellexp);

        weights[i] = contrast;
        if (channels == 3) {
            weights[i] = weights[i].mul(saturation);
        }
        weights[i] = weights[i].mul(wellexp) + 1e-12f;
        weight_sum += weights[i];
    }
    int maxlevel = static_cast<int>(logf(static_cast<float>(min(size.width, size.height))) / logf(2.0f));
    std::vector<Mat> res_pyr(maxlevel + 1);

    for (size_t i = 0; i < images.size(); i++) {
        weights[i] /= weight_sum;
        cv::Mat img;
        images[i].convertTo(img, CV_32F, 1.0f / 255.0f);

        std::vector<Mat> img_pyr, weight_pyr;
        cv::buildPyramid(img, img_pyr, maxlevel);
        cv::buildPyramid(weights[i], weight_pyr, maxlevel);

        for (int lvl = 0; lvl < maxlevel; lvl++) {
            cv::Mat up;
            cv::pyrUp(img_pyr[lvl + 1], up, img_pyr[lvl].size());
            img_pyr[lvl] -= up;
        }
        for (int lvl = 0; lvl <= maxlevel; lvl++) {
            std::vector<Mat> splitted(channels);
            cv::split(img_pyr[lvl], splitted);
            for (int c = 0; c < channels; c++) {
                splitted[c] = splitted[c].mul(weight_pyr[lvl]);
            }
            cv::merge(splitted, img_pyr[lvl]);
            if (res_pyr[lvl].empty()) {
                res_pyr[lvl] = img_pyr[lvl];
            }
            else {
                res_pyr[lvl] += img_pyr[lvl];
            }
        }
    }
    for (int lvl = maxlevel; lvl > 0; lvl--) {
        cv::Mat up;
        cv::pyrUp(res_pyr[lvl], up, res_pyr[lvl - 1].size());
        res_pyr[lvl - 1] += up;
    }
    dst.create(size, CV_32FCC);
    res_pyr[0].copyTo(dst);
}

cv::Mat imageEnhancement(cv::Mat img8)
{
    uchar *p;
    double mean_value;
    int num;
    for (int i = 0; i < img8.rows; i++)
    {
        p = img8.ptr<uchar>(i);
        for (int j = 0; j < img8.cols; ++j)
        {
            if(p[j]<5&&p[j]>250)
                continue;
            else
            {
                mean_value = mean_value + p[j];
                num = num+1;
            }
        }
    }
    mean_value = mean_value/num;
    //double temp = mean_value;
    cv::Mat lut_temp1;
    cv::Mat lut_temp2;
    std::vector<cv::Mat>sim_imgs;
    cv::Mat img_sim1, img_sim2;
    sim_imgs.push_back(img8);
    if (modelFlag == true)
    {
        if (mean_value < 255 / 4)
        {
            lut_temp1 = lut_gamma_2_1;
            lut_temp2 = lut_gamma_4_1;
            std::cout<<1<<std::endl;
        }
        else if (mean_value > 3 * 255 / 4)
        {
            lut_temp1 = lut_gamma_1_4;
            lut_temp2 = lut_gamma_1_2;
            std::cout<<2<<std::endl;
        }
        else
        {
            lut_temp1 = lut_gamma_1_2;
            lut_temp2 = lut_gamma_2_1;
            std::cout<<3<<std::endl;
        }
        cv::LUT(img8, lut_temp1, img_sim1);  sim_imgs.push_back(img_sim1);
        cv::LUT(img8, lut_temp2, img_sim2);  sim_imgs.push_back(img_sim2);
    }
    else
    {
        if (mean_value < 255 / 3)
        {
            lut_temp1 = lut_EI_2_1;
            lut_temp2 = lut_EI_4_1;
            std::cout<<1<<std::endl;
        }
        else if (mean_value > 2 * 255 / 3)
        {
            lut_temp1 = lut_EI_1_4;
            lut_temp2 = lut_EI_1_2;
            std::cout<<2<<std::endl;
        }
        else
        {
            lut_temp1 = lut_EI_1_2;
            lut_temp2 = lut_EI_2_1;
            std::cout<<3<<std::endl;
        }
        cv::LUT(img8, lut_temp1, img_sim1);  sim_imgs.push_back(img_sim1);
        cv::LUT(img8, lut_temp2, img_sim2);  sim_imgs.push_back(img_sim2);
    }
    cv::Mat exposureFusion;
    process(sim_imgs, exposureFusion);
    exposureFusion = exposureFusion * 255;
    exposureFusion.convertTo(exposureFusion, CV_8UC1);
    return exposureFusion;
}



void callback(const sensor_msgs::ImageConstPtr& img_msg) 
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);

    int height = img_msg->height;
    int width = img_msg->width;
	  Mat raw_image = cv_ptr->image;
    cv::Mat img8(height, width, CV_8UC1);
    if(raw_image.channels()>1)
      cvtColor(raw_image, raw_image, COLOR_BGR2GRAY);
    Mat tmp;
    // ushort upper = 60000;
    // ushort lower = 15000;
    // ushort* p;
	// for(int i=0;i<height;i++)
	// {
	
	// 	for(int j=0;j<width;++j)
	// 	{
	// 		if(p[j]>upper)
	// 			p[j]=upper;
	// 		else if(p[j]<lower)
	// 			p[j]=lower;
	// 	}
	// }




    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0)); // create a black mask image with the desired size
     // start Mask only for sthereo and hitsz dataset @note
    if(dataset=="sthereo"){
        // std::cout<<"sthereo"<<std::endl;     
        cv::rectangle(mask, cv::Rect(0, 0, width, height-90), cv::Scalar(255), -1); // draw a white rectangle in the top half 
    }
    else if(dataset == "hitsz")
    {
        cv::rectangle(mask, cv::Rect(5, 0, width-10, height), cv::Scalar(255), -1); // draw white rectangle
    }
    else {
        cv::rectangle(mask, cv::Rect(0, 0, width, height), cv::Scalar(255), -1); // draw a whole white rectangle
    }
    // end Mask only for sthereo and hitsz dataset


    if(dataset!="etio"){ // TODO
        raw_image.convertTo(img8, CV_8UC1,1 / 256.0); // convert image to 8bit
        normalize(raw_image, tmp, 0, 255, NORM_MINMAX,-1,mask);
        convertScaleAbs(tmp, img8);
    }
    //raw_image.convertTo(img8, CV_8UC1,1 / 256.0); // convert image to 8bit
    //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    //clahe->apply(img8, img8);
    //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(16, 16));
    //clahe->apply(img8, img8);
    // equalizeHist(img8,img8);
    //img8 = imageEnhancement(img8);

    // resize the image 
    // cv::resize(img8,img8,cv::Size(640,300));
    //img8=img8(cv::Range(0,400),cv::Range(0,640));

    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", raw_image).toImageMsg();
    output_msg->header = img_msg->header;
    pub_image.publish(output_msg);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "image_convert"); 
    ros::NodeHandle nh;   

    nh.getParam("img_topic",IMAGE_TOPIC); 
    nh.getParam("dataset",dataset); 

    uchar vec_gamma_1_4[256];
    uchar vec_gamma_1_2[256];
    uchar vec_gamma_2_1[256];
    uchar vec_gamma_4_1[256];
    for (int i = 0; i < 256; i++)
    {
        *(vec_gamma_1_4 + i) = (pow(double((i + 0.5) / 256), 3) * 256 );
        *(vec_gamma_1_2 + i) = (pow(double((i + 0.5) / 256), 2) * 256 );
        *(vec_gamma_2_1 + i) = (pow(double((i + 0.5) / 256), 0.75) * 256) ;
        *(vec_gamma_4_1 + i) = (pow(double((i + 0.5) / 256), 0.5) * 256 );
    }
    // for (int i = 0; i < 256; i++)
    // {
    //     *(vec_gamma_1_4 + i) = (pow(double((i + 0.5) / 256), 2) * 256);
    //     *(vec_gamma_1_2 + i) = (pow(double((i + 0.5) / 256), 1.75) * 256);
    //     *(vec_gamma_2_1 + i) = (pow(double((i + 0.5) / 256), 0.75) * 256);
    //     *(vec_gamma_4_1 + i) = (pow(double((i + 0.5) / 256), 0.5) * 256);
    // }

    cv::Mat lut_gamma_1(1, 256, CV_8UC1, vec_gamma_1_4); lut_gamma_1_4 = lut_gamma_1.clone();
    cv::Mat lut_gamma_2(1, 256, CV_8UC1, vec_gamma_1_2); lut_gamma_1_2 = lut_gamma_2.clone();
    cv::Mat lut_gamma_3(1, 256, CV_8UC1, vec_gamma_2_1); lut_gamma_2_1 = lut_gamma_3.clone();
    cv::Mat lut_gamma_4(1, 256, CV_8UC1, vec_gamma_4_1); lut_gamma_4_1 = lut_gamma_4.clone();

    // build look up table for EI estimation
    uchar EI_1[256];
    uchar EI_2[256];
    uchar EI_3[256];
    uchar EI_4[256];
    correct_vector_creat(EI_1, 0.25);
    correct_vector_creat(EI_2, 0.5);
    correct_vector_creat(EI_3, 0.25);
    correct_vector_creat(EI_4, 0.5);
    cv::Mat lut_1(1, 256, CV_8UC1, EI_1); lut_EI_1_4 = lut_1.clone();
    cv::Mat lut_2(1, 256, CV_8UC1, EI_2); lut_EI_1_2 = lut_2.clone();
    cv::Mat lut_3(1, 256, CV_8UC1, EI_3); lut_EI_2_1 = lut_3.clone();
    cv::Mat lut_4(1, 256, CV_8UC1, EI_4); lut_EI_4_1 = lut_4.clone();

    pub_image = nh.advertise<sensor_msgs::Image>("/convert_image",100);
    // ros::Subscriber sub_img = nh.subscribe("/convert_image16", 100, callback);
    ros::Subscriber sub_img = nh.subscribe(IMAGE_TOPIC, 100, callback);
    // ros::Subscriber sub_img = nh.subscribe("/thermal_image_raw", 100, callback);
    //ros::Subscriber sub_img = nh.subscribe("/boson/image_16bit", 100, callback);
    ros::Rate loop_rate(30);
    while (nh.ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

