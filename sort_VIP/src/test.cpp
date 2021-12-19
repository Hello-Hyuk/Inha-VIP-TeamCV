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
#include "sort_VIP/SortedBbox.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_publisher_node_custom_msgs");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sort_VIP::SortedBbox>("sort_VIP/msg", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sort_VIP::SortedBbox msg;
        msg.sorted_cx=100
        msg.sorted_cy=200
        msg.sorted_width=250
        msg.sorted_height=250

        pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}