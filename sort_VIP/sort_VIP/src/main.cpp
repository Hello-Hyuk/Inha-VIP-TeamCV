#include "ros/ros.h"
#include <string>
// darknet_ros_msgs
#include "sort_VIP/BoundingBox2D.h"
#include "sort_VIP/ObjectHypothesis.h"
#include "sort_VIP/Detector2D.h"
#include "sort_VIP/Detector2DArray.h"

using namespace std;

/*
norm_cx
norm_cy
norm_w
norm_h
d_frame
*/

void msgcallback4(const sort_VIP::Detector2DArray::ConstPtr& msg){
    unsigned int d_frame=msg->header.seq;
    printf("d_frame : %d\n",d_frame);
    printf("%d",sizeof(msg->detections));
    if(msg->detections.empty()==0){
        float norm_cx=msg->detections[0].bbox.center.x;
        float norm_cy=msg->detections[0].bbox.center.y;
        float norm_w=msg->detections[0].bbox.size_x;
        float norm_h=msg->detections[0].bbox.size_y;
        printf("norm_cx : %.3f\n",norm_cx);
        printf("norm_cy : %.3f\n",norm_cy);
        printf("norm_w : %.3f\n",norm_w);
        printf("norm_h : %.3f\n",norm_h);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sort_node");
    ros::NodeHandle n;
    ros::Subscriber sub4=n.subscribe<sort_VIP::Detector2DArray>("detections",100,msgcallback4);
    ros::spin();

    return 0;
}