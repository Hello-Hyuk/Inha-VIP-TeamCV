///////////////////////////////////////////////////////////////////////////////
//  SORT: A Simple, Online and Realtime Tracker
//  
//  This is a C++ reimplementation of the open source tracker in
//  https://github.com/abewley/sort
//  Based on the work of Alex Bewley, alex@dynamicdetection.com, 2016
//
//  Cong Ma, mcximing@sina.cn, 2016
//  
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <iomanip> // to format image names using setw() and setfill()
#include <set>
#include <string>

#include "ros/ros.h"

using namespace std;

int main(int argc, char **argv)
{
    /*rosros !! */
    ros::init(argc, argv, "sort_node");
    ros::NodeHandle n;
    ros::Subscriber sub4=n.subscribe<sort_VIP::Detector2DArray>("detections",100,msgcallback4);
    ros::spin();

    // -> perspective transformation
    sort_VIP::Detector2DArray::ConstPtr msg; //여기에 담으시면 됩니다 ^^ 
    ros::Publisher pub=n.advertise<sort_VIP::Detector2DArray>("sort_msg",100);
    //topic 이름이 sort_msg message 입니다 ^^ 
    sort_VIP::Detector2DArray::ConstPtr msg1;
    while(ros::ok()){
        /*담으시면 됩니다 !!*/
        /*sort/msg 에 msg 만들고 그대로 빌드하면 message 생성되게 setting해놨습니다. 입맛 따라 */
        //msg1.sorted_cx=100
        //msg1.sorted_cy=200
        //msg1.sorted_width=250
        //msg1.sorted_height=250
        //msg1.삐리리=10
        //pub.publish(msg1);
    }
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_publisher_node_custom_msgs");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sort_VIP::noetic_basics_part_1_msg>("noetic_basics_part_1/message", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        noetic_basics_part_1::noetic_basics_part_1_msg msg;
        msg.A = 1;
        msg.B = 2;
        msg.C = 3;

        pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}