#include <iostream>
#include <fstream>
#include <iomanip> // to format image names using setw() and setfill()
#include <io.h>    // to check file existence using POSIX function access(). On Linux include <unistd.h>.
#include <set>
#include <vector>
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
{// b
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

	if (tb.box.x < x_range[0] || tb.box.x > x_range[1]) {
		return false;
	}
	return true;
}


// global variables for counting
#define CNUM 20

vector<TrackingBox> testSORT(vector<TrackingBox> detData);


int main()
{
	TrackingBox tb;
	vector<TrackingBox> detData;
	if (isInROI(tb)) {
		detData.push_back(tb);
	}
	testSORT(detData);

	return 0;
}

TrackingBox testSORT(vector<TrackingBox> detData)
{
	vector<vector<TrackingBox>> detFrameData;
	vector<TrackingBox> tempVec;
	for (auto tb : detData)
		tempVec.push_back(tb);
	detFrameData.push_back(tempVec);
	tempVec.clear();

	// 3. update across frames
	int frame_count = 0;
	int max_age = 1;
	int min_hits = 3;
	double iouThreshold = 0.3;
	vector<KalmanTracker> ERPmini;
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


	// main loop
	for (int fi = 0; fi < maxFrame; fi++)
	{
		frame_count++;
		// I used to count running time using clock(), but found it seems to conflict with cv::cvWaitkey(),
		// when they both exists, clock() can not get right result. Now I use cv::getTickCount() instead
		if (ERPmini.size() == 0) // the first frame met
		{
			// initialize kalman ERPmini using first detections.
			for (unsigned int i = 0; i < detFrameData[fi].size(); i++)
			{
				KalmanTracker trk = KalmanTracker(detFrameData[fi][i].box);
				ERPmini.push_back(trk);
			}
			// output the first frame detections
			for (unsigned int id = 0; id < detFrameData[fi].size(); id++)
			{
				TrackingBox tb = detFrameData[fi][id];
			}
			continue;
		}

		///////////////////////////////////////
		// 3.1. get predicted locations from existing ERPmini.
		predictedBoxes.clear();

		Rect_<float> pBox = (*it).predict();
		if (pBox.x >= 0 && pBox.y >= 0)
		{
			predictedBoxes.push_back(pBox);
			it++;
		}
		else
		{
			it = ERPmini.erase(it);
		}
	}

	///////////////////////////////////////
	// 3.2. associate detections to tracked object (both represented as bounding boxes)

	trkNum = predictedBoxes.size();
	detNum = detFrameData[fi].size();

	iouMatrix.clear();
	iouMatrix.resize(trkNum, vector<double>(detNum, 0));

	for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
	{
		for (unsigned int j = 0; j < detNum; j++)
		{
			// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
			iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detFrameData[fi][j].box);
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

	if (detNum > trkNum) // there are unmatched detections
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
	// 3.3. updating ERPmini

	// update matched ERPmini with assigned detections.
	// each prediction is corresponding to a tracker
	int detIdx, trkIdx;
	for (unsigned int i = 0; i < matchedPairs.size(); i++)
	{
		trkIdx = matchedPairs[i].x;
		detIdx = matchedPairs[i].y;
		ERPmini[trkIdx].update(detFrameData[fi][detIdx].box);
	}

	// create and initialise new ERPmini for unmatched detections
	for (auto umd : unmatchedDetections)
	{
		KalmanTracker ERP = KalmanTracker(detFrameData[fi][umd].box);
		ERPmini.push_back(ERP);
	}

	TrackingBox res;
	res.box = (*it).get_state();
	res.id = (*it).m_id + 1;
	res.frame = frame_count;


	// remove dead tracklet
	if (it != ERPmini.end() && (*it).m_time_since_update > max_age)
		it = ERPmini.erase(it);

	return res;
}
