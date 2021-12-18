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

using namespace std;
using namespace cv;

typedef struct TrackingBox
{
	int frame;
	int id;
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

void printbox(float x, float y, float w, float h)
{
    printf("x : %.3f\n", x);
    printf("y : %.3f\n", y);
    printf("w : %.3f\n", w);
    printf("h : %.3f\n", h);
}

// global variables for counting
#define CNUM 20
int total_frames = 0;
double total_time = 0.0;

float ROI[4][2] = { {812, 419}, {1088, 423}, {216, 1080}, {1698, 1080} };

float src_width = 1920;
float src_height = 1080;

void TestSORT(string seqName, bool display);

// 3. update across frames
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

//detdata 만드는 변수
string detLine;
istringstream ss;
vector<TrackingBox> getdetData;
float tpx, tpy, tpw, tph;

// detFrameData로 저장 변수
int maxFrame = 0;
vector<vector<TrackingBox>> getdetFrameData;
vector<TrackingBox> tempVec;

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
            TrackingBox tb;

            float cx = msg->detections[i].bbox.center.x;
            float cy = msg->detections[i].bbox.center.y;
            tpw = msg->detections[i].bbox.size_x;
            tph = msg->detections[i].bbox.size_y;
            tpx = cx - tpw / 2;
            tpy = cy - tph / 2;

            // d_id 받아와야함
            tb.frame.push_back(d_frame[i]);
            tb.id.push_back(d_id[i]);
            tb.box = Rect_<float>(Point_<float>(tpx, tpy), Point_<float>(tpx + tpw, tpy + tph));
            getdetData.push_back(tb);
        }

        // frame별 data담r기
        for (auto tb : getdetData)
            if (tb.frame == frame_count + 1) // frame num starts from 1
                tempVec.push_back(tb);
        getdetFrameData.push_back(tempVec);
        tempVec.clear();
    }
    TestSORT(getdetFrameData);
}

void TestSORT(vector<vector<TrackingBox>>& detFrameData)
{
	cout << "Processing " << seqName << "..." << endl;
	//////////////////////////////////////////////
	// main loop
	int fi = 0; fi < maxFrame; fi++;
    
	total_frames++;
    frame_count++;
    int frameIdx = fraframe_count-1;
    cout << frame_count << endl;

    // I used to count running time using clock(), but found it seems to conflict with cv::cvWaitkey(),
    // when they both exists, clock() can not get right result. Now I use cv::getTickCount() instead.
    start_time = getTickCount();

    if (trackers.size() == 0) // the first frame met
    {
        // initialize kalman trackers using first detections.
        // vector[0] == frame 1
        for (unsigned int i = 0; i < detFrameData[frameIdx].size(); i++)
        {
            KalmanTracker trk = KalmanTracker(detFrameData[frameIdx][i].box);
            trackers.push_back(trk);
        }
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
        }
    }

    ///////////////////////////////////////
    // 3.2. associate detections to tracked object (both represented as bounding boxes)
    // dets : detFrameData[frame_count]
    trkNum = predictedBoxes.size();
    detNum = detFrameData[frameIdx].size();

    iouMatrix.clear();
    iouMatrix.resize(trkNum, vector<double>(detNum, 0));

    for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
    {
        for (unsigned int j = 0; j < detNum; j++)
        {
            // use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
            iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detFrameData[frame_count][j].box);
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

    if (detNum > trkNum) //	there are unmatched detections
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
        trackers[trkIdx].update(detFrameData[frameIdx][detIdx].box);
    }

    // create and initialise new trackers for unmatched detections
    for (auto umd : unmatchedDetections)
    {
        KalmanTracker tracker = KalmanTracker(detFrameData[frameIdx][umd].box);
        trackers.push_back(tracker);
    }

    // get trackers' output
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

    // result
    cout << "///////////SORT RESULT" << endl;
    cout << "box info\n" << "x :" << Perpective_trans_bbox[frameIdx].box.x << endl;
    cout << "y : " << Perpective_trans_bbox[frameIdx].box.y << endl;
    cout << "width : " << Perpective_trans_bbox[frameIdx].box.width << endl;
    cout << "height : " << Perpective_trans_bbox[frameIdx].box.height << endl;            
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
