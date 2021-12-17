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

#include "Hungarian.h"
#include "KalmanTracker.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ros/ros.h"
#include <string>
// darknet_ros_msgs
#include "sort_VIP/BoundingBox2D.h"
#include "sort_VIP/ObjectHypothesis.h"
#include "sort_VIP/Detector2D.h"
#include "sort_VIP/Detector2DArray.h"

using namespace std;
using namespace cv;

// global variables for counting
#define CNUM 20
int total_frames = 0;
double total_time = 0.0;

float ROI[4][2] = { {812, 419}, {1088, 423}, {216, 1080}, {1698, 1080} };

float src_width = 1920;
float src_height = 1080;

// ros로 부터 가져오는 data값
typedef struct TrackingBox
{
   int frame;
   int id = 0;
   Rect_<float> box;
}TrackingBox;

// yolo로 부터 받아오는 param을 저장하는 structure
typedef struct YoloDetectBox{
   float x;
   float y;
   float w;
   float h;
   int d_frame;
   int d_id;
}YoloDetectBox;

// subscribe 한 data 저장 vector
vector<TrackingBox> yoloData;
vector<vector<TrackingBox>> getdetData;

/////////////testSort() params////////////////////
// frame 관련 parameter
int frame_count = 0;
int max_age = 1;
int min_hits = 3;
double iouThreshold = 0.3;
vector<KalmanTracker> trackers;
KalmanTracker::kf_count = 0; // tracking id relies on this, so we have to reset it in each seq.

// variables used in the for-loop
vector<Rect_<float>> predictedBoxes;
vector<vector<double>> iouMatrix;
vector<int> assignment;
set<int> unmatchedDetections;
set<int> unmatchedTrajectories;
set<int> allItems;
set<int> matchedItems;
vector<cv::Point> matchedPairs;
vector<TrackingBox> frameTrackingResult;
unsigned int trkNum = 0;
unsigned int detNum = 0;

double cycle_time = 0.0;
int64 start_time = 0;
////////////////////////////////////////////////////

// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
   float in = (bb_test & bb_gt).area();
   float un = bb_test.area() + bb_gt.area() - in;

   if (un < DBL_EPSILON)
      return 0;

   return (double)(in / un);
}


int eqation(int y) {
   int x_range[2];
   x_range[0] = ((y - ROI[1][2]) * (ROI[0][0] - ROI[0][2]) / (ROI[1][0] - ROI[1][2])) + ROI[0][2];
   x_range[1] = ((y - ROI[1][1]) * (ROI[0][3] - ROI[0][1]) / (ROI[1][3] - ROI[1][1])) + ROI[0][1];

   return x_range;
}


bool isInROI(TrackingBox tb) {
   int x_range[2] = eqation(tb.box.y);
   if (tb.box.y < 280)
      return false;
   else {
      if (tb.box.x < x_range[0] || tb.box.x > x_range[1])
         return false;
   }
   return true;
}


void printbox(float x, float y, float w, float h)
{
    printf("x : %.3f\n", x);
    printf("y : %.3f\n", y);
    printf("w : %.3f\n", w);
    printf("h : %.3f\n", h);
}

int testSort(vector<vector<TrackingBox>>& getdetData)
{
   vector<TrackingBox> detData;
   if (total_frames == getdetData.frame)
   {
      break;
   }
   // update frame
   double Frame = frame_count;

   // 1. input real-time bbox
   vector<vector<TrackingBox>> detFrameData;
   TrackingBox temptb;
   for (unsigned int i = 0; i < getdetData[0].size(); i++)
   {
      if (GetIOU(bb_center, getdetData[0][i].box) < bb_iou_th)
      {
         getdetData[0][i].erase();
      }
      else
      {
         temptb.box = Rect_<float>(Point_<float>(getdetData[0].box.x, getdetData[0].box.y),
                                   Point_<float>(getdetData[0].box.x + getdetData[0].box.width, getdetData[0].box.y + getdetData[0].box.height));
         detData.push_back(temptb);
      }
   }
   detFrameData.push_back(detData);
   detData.clear();

   ////////////////main algorithm//////////////////
   start_time = getTickCount();

   if (trackers.size() == 0) // the first frame met
   {
      // initialize kalman trackers using first detections.
      for (unsigned int i = 0; i < detFrameData[Frame].size(); i++)
      {
         KalmanTracker trk = KalmanTracker(detFrameData[Frame][i].box);
         cout << "init_trk_kf_count : " << trk.kf_count << endl;
         cout << "kf_count : " << KalmanTracker::kf_count << endl;
         trackers.push_back(trk);
      }
      continue;
   }

   ///////////////////////////////////////
   // 3.1. get predicted locations from existing trackers.
   predictedBoxes.clear();

   for (auto it = trackers.begin(); it != trackers.end();)
   {
      Rect_<float> pBox = (*it).predict();
      if (pBox.x >= 0 && pBox.y >= 0)
      {
         predictedBoxes.push_back(pBox);
         it++;
      }
      else
      {
         it = trackers.erase(it);
         //cerr << "Box invalid at frame: " << frame_count << endl;
      }
   }

   ///////////////////////////////////////
   // 3.2. associate detections to tracked object (both represented as bounding boxes)
   // dets : detFrameData[Frame]
   trkNum = predictedBoxes.size();
   detNum = detFrameData[Frame].size();

   iouMatrix.clear();
   iouMatrix.resize(trkNum, vector<double>(detNum, 0));

   for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
   {
      for (unsigned int j = 0; j < detNum; j++)
      {
         // use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
         iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detFrameData[Frame][j].box);
      }
   }

   // solve the assignment problem using hungarian algorithm.
   // the resulting assignment is [track(prediction) : detection], with len=preNum
   HungarianAlgorithm HungAlgo;
   assignment.clear();
   HungAlgo.Solve(iouMatrix, assignment);

   // find matches, unmatched_detections and unmatched_predictions
   unmatchedTrajectories.clear();
   unmatchedDetections.clear();
   allItems.clear();
   matchedItems.clear();

   if (detNum > trkNum) //   there are unmatched detections
   {
      for (unsigned int n = 0; n < detNum; n++)
         allItems.insert(n);

      for (unsigned int i = 0; i < trkNum; ++i)
         matchedItems.insert(assignment[i]);

      set_difference(allItems.begin(), allItems.end(),
                     matchedItems.begin(), matchedItems.end(),
                     insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
   }
   else if (detNum < trkNum) // there are unmatched trajectory/predictions
   {
      for (unsigned int i = 0; i < trkNum; ++i)
         if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
            unmatchedTrajectories.insert(i);
   }
   else
      ;

   // filter out matched with low IOU
   matchedPairs.clear();
   for (unsigned int i = 0; i < trkNum; ++i)
   {
      if (assignment[i] == -1) // pass over invalid values
         continue;
      if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
      {
         unmatchedTrajectories.insert(i);
         unmatchedDetections.insert(assignment[i]);
      }
      else
         matchedPairs.push_back(cv::Point(i, assignment[i]));
   }

   ///////////////////////////////////////
   // 3.3. updating trackers

   // update matched trackers with assigned detections.
   // each prediction is corresponding to a tracker
   int detIdx, trkIdx;
   for (unsigned int i = 0; i < matchedPairs.size(); i++)
   {
      trkIdx = matchedPairs[i].x;
      detIdx = matchedPairs[i].y;
      trackers[trkIdx].update(detFrameData[Frame][detIdx].box);
   }

   // create and initialise new trackers for unmatched detections
   for (auto umd : unmatchedDetections)
   {
      KalmanTracker tracker = KalmanTracker(detFrameData[Frame][umd].box);
      cout << "tracker_kf_count : " << tracker.kf_count << endl;
      cout << "kf_count : " << KalmanTracker::kf_count << endl;
      trackers.push_back(tracker);
   }

   // get trackers' output
   vector<TrackingBox> Perpective_trans_bbox;
   int pt_bbox_cnt = 0;
   frameTrackingResult.clear();
   for (auto it = trackers.begin(); it != trackers.end();)
   {
      if (((*it).m_time_since_update < 1) &&
          ((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
      {
         TrackingBox res;
         res.box = (*it).get_state();
         res.id = (*it).m_id + 1;
         res.frame = frame_count;
         frameTrackingResult.push_back(res);
         if (isInROI(res))
         {
            Perpective_trans_bbox.push_back(res);
         }
         it++;
      }
      else
         it++;

      // remove dead tracklet
      if (it != trackers.end() && (*it).m_time_since_update > max_age)
         it = trackers.erase(it);
   }

   cycle_time = (double)(getTickCount() - start_time);
   total_time += cycle_time / getTickFrequency();

   // tracker 별 result
   cout << Frame << " frame result" << endl;
   for (auto tb : frameTrackingResult)
      cout << tb.frame << "," << tb.id << ","
           << tb.box.x << "," << tb.box.y << ","
           << tb.box.width << "," << tb.box.height << endl;

   ////// ros publisher "custom msg"
   float pt_x = Perpective_trans_bbox[Frame].box.x;
   float pt_y = Perpective_trans_bbox[Frame].box.y;
   float pt_width = Perpective_trans_bbox[Frame].box.width;
   float pt_height = Perpective_trans_bbox[Frame].box.height;

   ///// 수정 필요 /////
   ros::NodeHandle nh;
   // ros::Publisher pub = n.advertise<Rect_<float>>("sort_bbox", 1000);
   // ros::Rate rate(10);
   ros::gFacePublisher = nh.advertise<perception::TrackingBox>(“face”, 1);
   perception::TrackingBox trackingbox;

   int count = 0;
   while (ros::ok())
   {
      std_msgs::Rect_<float> bbox;
      bbox.data = frameTrackingResult.box;
      pub.publish(bbox);
      ros::spinOnce();
      rate.sleep();
      ++count;
   }
   getdetData.clear();
   yoloData.claer();

   return 100000;
}

/*변수 꺼내오는 예제*/
void msgcallback4(const sort_VIP::Detector2DArray::ConstPtr& msg){

   total_frames++;
   frame_count++;

   unsigned int d_frame = msg->header.seq;
   printf("d_frame : %d\n", d_frame);
   printf("%d", sizeof(msg->detections));

   if (msg->detections.empty() == 0)
   {
      for (int i = 0; i < sizeof(msg->detections); i++)
      {
         float cx = msg->detections[i].bbox.center.x;
         float cy = msg->detections[i].bbox.center.y;
         float w = msg->detections[i].bbox.size_x;
         float h = msg->detections[i].bbox.size_y;
      }
      TrackingBox tb;
      // size()는 yolo의 한 frame에서 가져온 detect 된 bbox의 개수
      for (int i = 0; i < sizeof(msg->detections); i++)
      {
         float cx = msg->detections[i].bbox.center.x;
         float cy = msg->detections[i].bbox.center.y;
         float w = msg->detections[i].bbox.size_x;
         float h = msg->detections[i].bbox.size_y;
         if (cx == 0 && cy == 0 && w == 0 && h == 0)
            break;
         else
         {
            tb.box.width.push_back(w[i]);
            tb.box.height.push_back(h[i]);
            tb.box.x.push_back(x[i] - w[i] / 2);
            tb.box.y.push_back(y[i] - h[i] / 2);
            tb.frame.push_back(d_frame[i]);
            tb.frame.push_back(d_id[i]);
            yoloData.push_back(tb);
         }
      }
      getdetData.push_back(yoloData);
      cout << "test sort result" << endl;
      cout << testSort(getdetData) << endl;
    }
    
}

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
    while(ros::ok){
        /*담으시면 됩니다 !!*/
        /*sort/msg 에 msg 만들고 그대로 빌드하면 message 생성되게 setting해놨습니다. 입맛 따라 */
        //msg1.sorted_cx=?
        //msg1.sorted_cy=?
        //msg1.sorted_width=?
        //msg1.sorted_height=?
        //msg1.삐리리=10
        //pub.publish(msg1);
    }

    return 0;
}