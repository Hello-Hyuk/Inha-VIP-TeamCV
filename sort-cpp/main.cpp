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
#include <io.h>    // to check file existence using POSIX function access(). On Linux include <unistd.h>.
#include <set>

#include "Hungarian.h"
#include "KalmanTracker.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ros/ros.h"
#include "ros_sort/trackingbox.h"

using namespace std;
using namespace cv;


float ROI[4][2] = { {812, 419}, {1088, 423}, {216, 1080}, {1698, 1080} };

float src_width = 1920;
float src_height = 1080;


// ros로 부터 가져오는 data값
vector<TrackingBox> yoloData;
vector<vector<TrackingBox>> getdetData;

// yolo로 부터 받아오는 param을 저장하는 structure
typedef struct YoloDetectBox{
   float x;
   float y;
   float w;
   float h;
   int d_frame;
   int d_id;
}YoloDetectBox;

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

typedef struct TrackingBox
{
   int frame;
   int id = 0;
   Rect_<float> box;
}TrackingBox;


// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
   float in = (bb_test & bb_gt).area();
   float un = bb_test.area() + bb_gt.area() - in;

   if (un < DBL_EPSILON)
      return 0;

   return (double)(in / un);
}


float* equation(float y) {
    float x_range[2];
    x_range[0] = ((y - ROI[2][1]) * (ROI[0][0] - ROI[2][0]) / (ROI[0][1] - ROI[2][1])) + ROI[2][0];
    x_range[1] = ((y - ROI[1][1]) * (ROI[3][0] - ROI[1][0]) / (ROI[3][1] - ROI[1][1])) + ROI[1][0];

    return x_range;
}


bool isInROI(TrackingBox tb) {
   if (tb.box.y < 419)
      return false;

   float* x_range = equation(tb.box.y);

   if (tb.box.x < x_range[0] || tb.box.x > x_range[1]){
         return false;
   }
   return true;
}

// global variables for counting
#define CNUM 20
int total_frames = 0;
double total_time = 0.0;

void testSort(bool display);

int main(int argc, char **argv)
{
   ros::init(argc, argv, "sort_node");

   //////////// 코드 수정 ////////////
   TrackingBox tb;
   // size()는 yolo의 한 frame에서 가져온 detect 된 bbox의 개수
   for (; size())
   {
      // float width = (src_width * norm_w);
      // float height = (src_height * norm_h);
      // float x = src_width * norm_cx - width / 2;
      // float y = src_height * norm_cy - height / 2;
      tb.box.width.push_back(w[i]);
      tb.box.height.push_back(h[i]);
      tb.box.x.push_back(x[i] - w[i] / 2);
      tb.box.y.push_back(y[i] - h[i] / 2);
      tb.frame.push_back(d_frame[i]);
      tb.frame.push_back(d_id[i]);
      yoloData.push_back(tb);
   }
   getdetData.push_back(yoloData);

   testSort(getdetData);
   ////////////////////////////////////

   // Note: time counted here is of tracking procedure, while the running speed bottleneck is opening and parsing detectionFile.
   cout << "Total Tracking took: " << total_time << " for " << total_frames << " frames or " << ((double)total_frames / (double)total_time) << " FPS" << endl;

   return 0;
}



void testSort(vector<vector<TrackingBox>>& getdetData)
{
   vector<TrackingBox> detData;
   if (total_frames == getdetData.frame)
   {
      break;
   }

   total_frames++;
   frame_count++;

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
}