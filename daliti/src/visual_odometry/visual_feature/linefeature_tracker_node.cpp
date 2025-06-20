#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "linefeature_tracker.h"

// #include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;

LineFeatureTracker trackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;
bool init_pub = 0;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
    }

    
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    TicToc t_r;


    //dango revise M2DGR
    sensor_msgs::Image image_raw=*img_msg;
    image_raw.encoding=img_msg->encoding;
    sensor_msgs::ImageConstPtr image_msg = boost::make_shared<sensor_msgs::Image>(image_raw);


    if (PUB_THIS_FRAME)
    {
        cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    //cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
//    cv::imshow("lineimg",show_img);
//    cv::waitKey(1);
    
    frame_cnt++;
    trackerData.readImage(ptr->image.rowRange(0 , ROW));   

        pub_count++;
        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v

        feature_lines->header = img_msg->header;
        feature_lines->header.frame_id = "camera_init";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)  
            {
                auto un_lines = trackerData.undistortedLineEndPoints();

                //auto &cur_lines = trackerData.curframe_->vecLine;
                auto &ids = trackerData.curframe_->lineID;

                for (unsigned int j = 0; j < ids.size(); j++)
                {

                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_lines[j].StartPt.x;
                    p.y = un_lines[j].StartPt.y;
                    p.z = 1;

                    feature_lines->points.push_back(p);
                    id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
                    //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
                    u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
                    v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
                    //ROS_ASSERT(inBorder(cur_pts[j]));
                }
            }

        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);
        ROS_INFO("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
        if (!init_pub) {
            init_pub = 1;
        } else {
            pub_img.publish(feature_lines);
        }
        //pub_img.publish(feature_lines);
    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
    // ROS_INFO("whole Line feature tracker processing costs: %f", mean_time);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Line Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData.readIntrinsicParameter(CAM_NAMES[i]);

    ROS_INFO("start line feature");
    //ros::Subscriber sub_img = n.subscribe("/image_raw", 100, img_callback);
    ros::Subscriber sub_img = n.subscribe("/camera/edge", 100, img_callback);

    pub_img = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/linefeature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>(PROJECT_NAME + "/vins/feature/linefeature_img",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}
