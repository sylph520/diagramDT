#include "stdafx.h"
#include "diagram_parsing.h"
//basic functions
vector<Point2f> getPointPositions(Mat binaryImage)
{
	vector<Point2f> pointPositions;

	for (unsigned int y = 0; y < binaryImage.rows; ++y)
	{
		//unsigned char* rowPtr = binaryImage.ptr<unsigned char>(y);
		for (unsigned int x = 0; x < binaryImage.cols; ++x)
		{
			//if(rowPtr[x] > 0) pointPositions.push_back(Point2i(x,y));
			if (binaryImage.at<unsigned char>(y, x) > 0) pointPositions.push_back(Point2f(x, y));
		}
	}

	return pointPositions;
}
double p2pdistance(Vec2i pt1, Vec2i pt2)
{
	double distance;
	distance = sqrt(powf((pt1[0] - pt2[0]), 2) + powf((pt1[1] - pt2[1]), 2));
	return distance;
}
bool on_line(Vec4i line, Vec2i pt)
{
	double linedis_eps = 3; double pointdis_eps = 5;
	Vec2i ref_point = { line[0], line[1] };
	Vec2i pt2 = { -pt[1], pt[0] };
	Vec2i ref_point_t = { line[1], -line[0] };
	Vec2i vec = { line[2] - line[0], line[3] - line[1] };
	double point2line = abs((pt2.dot(vec) + ref_point_t.dot(vec))) / sqrt(vec.dot(vec));
	if (point2line < linedis_eps)
		return true;
	else
		return false;
}
int on_circle(Vec2i pt, vector<Vec3f> circles)
{
	// check if the point pt is on one of the circles,or joints of multiple circles, or nothing to do with circles
	// joint_flag parameter: 0 means not on any circle, 1 means on a circle, 2 means on two circles and so on
	int dis = 1;// this parameter is to be set to check on the distance tolerants within the distance between radius and distance of pt and circle center point
	int count = 0;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec2f center = (circles[i][0], circles[i][1]);
		double radius = circles[i][2];
		double distance = p2pdistance(center, pt);
		while (distance - radius <= dis)
		{
			count++;
		}

	}
	return count;
}
bool same_pt(Vec2i pt1, Vec2i pt2)
{
	double eps = 7;//to be set parameter
	if (p2pdistance(pt1, pt2) <= eps)
		return true;
	else
		return false;
}
inline void getCircle(Point2f& p1, Point2f& p2, Point2f& p3, Point2f& center, float& radius)
{
	float x1 = p1.x;
	float x2 = p2.x;
	float x3 = p3.x;

	float y1 = p1.y;
	float y2 = p2.y;
	float y3 = p3.y;

	// PLEASE CHECK FOR TYPOS IN THE FORMULA :)
	center.x = (x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2);
	center.x /= (2 * (x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2));

	center.y = (x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1);
	center.y /= (2 * (x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2));

	radius = sqrt((center.x - x1)*(center.x - x1) + (center.y - y1)*(center.y - y1));
}

// specific fucntions
float evaluateCircle(Mat dt, Point2f center, float radius)
{

	float completeDistance = 0.0f;
	int counter = 0;

	float maxDist = 1.0f;   //TODO: this might depend on the size of the circle!

	float minStep = 0.001f;
	// choose samples along the circle and count inlier percentage

	//HERE IS THE TRICK that no minimum/maximum circle is used, the number of generated points along the circle depends on the radius.
	// if this is too slow for you (e.g. too many points created for each circle), increase the step parameter, but only by factor so that it still depends on the radius

	// the parameter step depends on the circle size, otherwise small circles will create more inlier on the circle
	float step = 2 * 3.14159265359f / (6.0f * radius);
	if (step < minStep) step = minStep; // TODO: find a good value here.

	//for(float t =0; t<2*3.14159265359f; t+= 0.05f) // this one which doesnt depend on the radius, is much worse!
	for (float t = 0; t < 2 * 3.14159265359f; t += step)
	{
		float cX = radius*cos(t) + center.x;
		float cY = radius*sin(t) + center.y;

		if (cX < dt.cols)
			if (cX >= 0)
				if (cY < dt.rows)
					if (cY >= 0)
						if (dt.at<float>(cY, cX) <= maxDist)
						{
							completeDistance += dt.at<float>(cY, cX);
							counter++;
						}

	}

	return counter;
}

void detect_circle(Mat& colorgeo, Mat& graygeo_blob, Mat& geometry_graph_bw, vector<Vec3f>& circles)
{
	unsigned int numberOfCirclesToDetect = 2;   // TODO: if unknown, you'll have to find some nice criteria to stop finding more (semi-) circles
	for (unsigned int j = 0; j < numberOfCirclesToDetect; ++j)
	{
		vector<Point2f> edgePositions;
		edgePositions = getPointPositions(geometry_graph_bw);

		std::cout << "number of edge positions: " << edgePositions.size() << endl;

		// create distance transform to efficiently evaluate distance to nearest edge
		Mat dt;
		distanceTransform(255 - geometry_graph_bw, dt, CV_DIST_L1, 3);

		unsigned int nIterations = 0;

		Point2f bestCircleCenter;
		float bestCircleRadius;
		//float bestCVal = FLT_MAX;
		float bestCVal = -1;

		//float minCircleRadius = 20.0f; // TODO: if you have some knowledge about your image you might be able to adjust the minimum circle radius parameter.
		float minCircleRadius = 0.0f;

		//TODO: implement some more intelligent ransac without fixed number of iterations
		for (unsigned int i = 0; i < 2000; ++i)
		{
			//RANSAC: randomly choose 3 point and create a circle:
			//TODO: choose randomly but more intelligent,
			//so that it is more likely to choose three points of a circle.
			//For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
			unsigned int idx1 = rand() % edgePositions.size();
			unsigned int idx2 = rand() % edgePositions.size();
			unsigned int idx3 = rand() % edgePositions.size();

			// we need 3 different samples:
			if (idx1 == idx2) continue;
			if (idx1 == idx3) continue;
			if (idx3 == idx2) continue;

			// create circle from 3 points:
			Point2f center; float radius;
			getCircle(edgePositions[idx1], edgePositions[idx2], edgePositions[idx3], center, radius);

			if (radius < minCircleRadius)continue;


			//verify or falsify the circle by inlier counting:
			//float cPerc = verifyCircle(dt,center,radius, inlierSet);
			float cVal = evaluateCircle(dt, center, radius);

			if (cVal > bestCVal)
			{
				bestCVal = cVal;
				bestCircleRadius = radius;
				bestCircleCenter = center;
			}

			++nIterations;
		}
		std::cout << "current best circle: " << bestCircleCenter << " with radius: " << bestCircleRadius << " and nInlier " << bestCVal << endl;
		Vec3f circle = { bestCircleCenter.x, bestCircleCenter.y, bestCircleRadius };
		circles.push_back(circle);
		//draw the cicle detected in red within the colorgeo blob image
		cv::circle(colorgeo, bestCircleCenter, bestCircleRadius, Scalar(0, 0, 255));

		//TODO: hold and save the detected circle.

		//TODO: instead of overwriting the mask with a drawn circle it might be better to hold and ignore detected circles and dont count new circles which are too close to the old one.
		// in this current version the chosen radius to overwrite the mask is fixed and might remove parts of other circles too!

		// update mask: remove the detected circle!
		cv::circle(geometry_graph_bw, bestCircleCenter, bestCircleRadius, 0, 3); // here the radius is fixed which isnt so nice.
		cv::circle(graygeo_blob, bestCircleCenter, bestCircleRadius, Scalar(255,255,255), 3);
	}

	//namedWindow("edges"); imshow("edges", geometry_graph_bw);
	//namedWindow("graygeo blob without circles"); imshow("graygeo blob without circles" , graygeo_blob);
	namedWindow("colorgeo"); cv::imshow("colorgeo", colorgeo);
}

void detect_circle2(Mat& colorgeo,Mat& graygeo, Mat& geometry_graph_bw, vector<Vec3f>& circles)
{
	Mat gray_blured;
	GaussianBlur(graygeo, gray_blured, Size(7, 7), 2, 2);
	//apply the hough transfrom  to find the circles
	HoughCircles(gray_blured, circles, CV_HOUGH_GRADIENT, 1, graygeo.rows / 8, 100, 100, 0, 0);
	std::cout << circles.size() << endl;
	// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle outline
		circle(gray_blured, center, radius, Scalar(0, 0, 0), 3, 8, 0);
		circle(geometry_graph_bw, center, radius, Scalar(0, 0, 0), 10, 8, 0);
		circle(graygeo, center, radius, Scalar(255, 255, 255), 10, 8, 0);
	}
	cv::namedWindow("Hough circle Transform", CV_WINDOW_AUTOSIZE);
	cv::imshow("Hough circle Transform", gray_blured);
	cv::namedWindow("after", CV_WINDOW_AUTOSIZE);
	cv::imshow("after", geometry_graph_bw);


}
struct distance_info
{
	Vec2i pt1; Vec2i pt2;// the points to calculate on
	double distance;//computed distance results with pt1 and pt2
};
//void findLineEnd(Vec4i temp_line, vector<Vec2i> temp_points, vector<Vec2i>& temp_points2)
//{
//	Vec2i  lineEnd1, lineEnd2;
//	vector<Vec2i> tempCollinearPoints;
//	vector<distance_info> distance_infos;
//	Vec2i temp_pt1 = { temp_line[0], temp_line[1] }; Vec2i temp_pt2 = { temp_line[2], temp_line[3] };
//	tempCollinearPoints.push_back(temp_pt1); tempCollinearPoints.push_back(temp_pt2);
//	double tempdistance0 = p2pdistance(temp_pt1, temp_pt2);
//	distance_info temp_distance_info = { temp_pt1, temp_pt2, tempdistance0 };
//	distance_infos.push_back(temp_distance_info);
//	for (size_t i = 0; i < temp_points.size(); i++)
//	{
//		if (on_line(temp_line, temp_points[i])&&(temp_points[i]!=temp_pt1)&&(temp_points[i]!=temp_pt2))
//		{
//			tempCollinearPoints.push_back(temp_points[i]);
//			double tempdistance1 = p2pdistance(temp_pt1, temp_points[i]); double tempdistance2 = p2pdistance(temp_pt2, temp_points[i]);
//			distance_info temp_distance_info1 = { temp_points[i], temp_pt1, tempdistance1 };
//			distance_info temp_distance_info2 = { temp_points[i], temp_pt2, tempdistance2 };
//			distance_infos.push_back(temp_distance_info1); distance_infos.push_back(temp_distance_info2);
//		}
//	}
//	sort(distance_infos.begin(), distance_infos.end(),
//		[](distance_info a, distance_info b){
//		return (a.distance > b.distance);
//	});
//	//now the distance_infos[0] have the biggest distance value on the same line so it should be the end points of the line
//	lineEnd1 = distance_infos[0].pt1; lineEnd2 = distance_infos[0].pt2;	
//	temp_points2.push_back(lineEnd1); temp_points2.push_back(lineEnd2);
//	for (vector<Vec2i>::iterator iter = distan)
//	{
//
//	}
//}

void findLineEnds(vector<Vec2i> colpoints, vector<Vec2i>& lineEnds, vector<Vec2i>& temp_points)
{
	vector<distance_info> distance_infos;
	for (vector<Vec2i>::iterator iter1 = colpoints.begin(); iter1 != colpoints.end(); iter1++)
	{
		Vec2i pt1 = *iter1;
		for (vector<Vec2i>::iterator iter2 = iter1 + 1; iter2 != colpoints.end(); iter2++)
		{
			Vec2i pt2 = *iter2;
			double tempDistance = p2pdistance(pt1, pt2);
			distance_info dis_info = { pt1, pt2, tempDistance };
			distance_infos.push_back(dis_info);
		}
	}
	std::sort(distance_infos.begin(), distance_infos.end(), 
	[](distance_info a, distance_info b){
		return (a.distance > b.distance);
	});
	Vec2i pt3 = distance_infos[0].pt1; Vec2i pt4 = distance_infos[0].pt2;
	lineEnds.push_back(pt3); lineEnds.push_back(pt4);
	int count0 = 0;
	cout << "colpoints size " << colpoints.size();
	for (vector<Vec2i>::iterator iter3 = colpoints.begin(); iter3 != colpoints.end(); iter3++)
	{
		Vec2i tempPt1 = *iter3;
		for (vector<Vec2i>::iterator iter4 = temp_points.begin(); iter4 != temp_points.end();)
		{
			Vec2i tempPt2 = *iter4;
			if (tempPt2 == tempPt1&&(tempPt2 != pt3)&&(tempPt2 != pt4))
			{
				iter4 = temp_points.erase(iter4);
				count0++;
				//cout << "iter4 value " << ++count0 <<" "<<tempPt2<< endl;
				break;
			}
			else
			{
				iter4++;
				continue;
			}
		}
	}
	cout << " count0 " << count0 << endl;
}
void detect_lines_endpoints(Mat&colorgeo, Mat& graygeo_blob, Mat& geometry_graph_bw, vector<Vec4i>& lines, vector<spe_point>& points)
{
	//imshow("test", geometry_graph_bw);
	//FileStorage fs(".\\geometry_graph_bw.xml", FileStorage::WRITE);
	//fs << "geometry_graph_bw" << geometry_graph_bw;
	//fs.release();
	cv::imshow("test", geometry_graph_bw);
	HoughLinesP(geometry_graph_bw, lines, 1, CV_PI / 180, 30, 30, 10);
	std::cout << "original pht detected lines is " << lines.size() << endl;
	vector<Vec2i> temp_points;
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec2i pt1 = Vec2i(lines[i][0], lines[i][1]); Vec2i pt2 = Vec2i(lines[i][2], lines[i][3]);
		temp_points.push_back(pt1); temp_points.push_back(pt2);
	}
	//handle the candidate points from candidate lines
	std::cout << "The initial size of points is " << temp_points.size() << endl;
	for (vector<Vec2i>::iterator iter1 = temp_points.begin(); iter1 != temp_points.end(); ++iter1)
	{
		Vec2i pt1 = *iter1;
		for (vector<Vec2i>::iterator iter2 = iter1 + 1; iter2 != temp_points.end(); ++iter2)
		{
			if (same_pt(*iter1, *iter2))
			{
				temp_points.erase(iter2--);
			}
			//else
			//{
			//	cv::circle(colorgeo, Point(*iter2), 1, Scalar(255, 0, 0), 5, 8, 0);
			//}
		}
		//cv::circle(colorgeo, Point(pt1[0],pt1[1]), 1, Scalar(255, 0, 0), 5, 8, 0);
	}
	std::sort(temp_points.begin(), temp_points.end(), [](Vec2i a, Vec2i b)
	{
		if (a[0] != b[0])
			return (a[0] < b[0]);
		else
			return (a[1] < b[1]);
	});
	for (size_t i = 0; i < temp_points.size(); i++)
	{
		cout << temp_points[i][0] << " " << temp_points[i][1] << endl;
	}
	std::cout << "Now, the size of points is second " << temp_points.size() << endl;
	vector<Vec2i> lineEnds; vector<Vec4i> newlines;
	int count = 0;
	for (vector<Vec4i>::iterator iter3 = lines.begin(); iter3 != lines.end(); iter3++)
	{
		Vec4i temp_line = *iter3;
		Vec2i pt1 = { temp_line[0], temp_line[1] }; Vec2i pt2 = { temp_line[2], temp_line[3] };
		
		vector<Vec2i>::iterator iter5, iter6;
		iter5 = find(temp_points.begin(), temp_points.end(), pt1);
		iter6 = find(temp_points.begin(), temp_points.end(), pt2);
		
		if (iter5==temp_points.end() || iter6==temp_points.end())
		{
			count++;
			continue;
		}
		else
		{
			vector<Vec2i> tempCollinearPoints;
			tempCollinearPoints.push_back(pt1); tempCollinearPoints.push_back(pt2);
			for (vector<Vec2i>::iterator iter4 = temp_points.begin(); iter4 != temp_points.end(); iter4++)
			{
				Vec2i temp_pt = *iter4;
				if (on_line(temp_line, temp_pt))
				{
					tempCollinearPoints.push_back(temp_pt);
				}
			}
			std::sort(tempCollinearPoints.begin(), tempCollinearPoints.end(), [](Vec2i a, Vec2i b)
			{
				if (a[0] != b[0])
					return (a[0] < b[0]);
				else
					return (a[1] < b[1]);
			});
			tempCollinearPoints.erase(unique(tempCollinearPoints.begin(), tempCollinearPoints.end()), tempCollinearPoints.end());
			findLineEnds(tempCollinearPoints, lineEnds, temp_points);
			//size_t tempSize = lineEnds.size();
			//Vec4i newLine = { lineEnds[tempSize - 2][0], lineEnds[tempSize - 2][1], lineEnds[tempSize - 1][0], lineEnds[tempSize - 1][1] };
			//newlines.push_back(newLine);
		}
		std::sort(lineEnds.begin(), lineEnds.end(), [](Vec2i a, Vec2i b)
		{
			if (a[0] != b[0])
				return (a[0] < b[0]);
			else
				return (a[1] < b[1]);
		});
		lineEnds.erase(unique(lineEnds.begin(), lineEnds.end()), lineEnds.end());
		//std::cout << "the num of lines now " << lines.size() << endl;
		//std::cout << endl;
		for (size_t i = 0; i < lineEnds.size(); i++)
		{
			Vec2i pt1 = lineEnds[i];
			Scalar temp_color = Scalar((rand() & 255), (rand() & 255), (rand() & 255));
			cv::circle(colorgeo, Point(pt1[0], pt1[1]), 1, temp_color, 5, 8, 0);
			cout << pt1[0] << " " << pt1[1] << endl;
		}

	}
	cout << "count " << count << endl;
	std::cout << "Now the size of lines is " << lines.size() << endl;
	std::cout << "Now, the size of points is third " << lineEnds.size() << endl;
	cv::imshow("points test", colorgeo);
	//imshow("graygeo blob", graygeo_blob);
}
//void detect_lines(Mat& geometry_graph_bw, Mat graygeo_blob, vector<Vec4i>& lines)
//{
//	HoughLinesP(geometry_graph_bw, lines, 1, CV_PI / 180, 50, 30, 10);
//	cout << "original pht detected lines" << lines.size() << endl;
//	for (size_t i = 0; i < lines.size(); i++)
//	{
//		Scalar temp_color = Scalar((rand() & 255), (rand() & 255), (rand() & 255));
//		line(graygeo_blob, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), temp_color, 1, 8);
//	}
//	imshow("graygeo blob", graygeo_blob);
//	//process the lines info in a further way
//	vector<Vec4i> temp_lines;
//	vector<bool> flag;
//	for (size_t i = 0; i < lines.size(); i++)
//	{
//		flag.push_back(false);
//	}
//	for (size_t i = 0; i < lines.size(); i++)
//	{
//		if (flag[i] == false)
//		{
//			temp_lines.push_back(lines[i]);
//			flag[i] = true;
//		}
//		else
//			continue;
//		for (size_t j = i + 1; j < lines.size(); j++)
//		{
//			if (flag[j] == false)
//			{
//				Vec2i pt1 = { lines[j][0], lines[j][1] };
//				Vec2i pt2 = { lines[j][2], lines[j][3] };
//				if (!(on_line(lines[i], pt1) && on_line(lines[i], pt2)))
//				{
//					flag[j] = false;
//				}
//				else
//					flag[j] = true;
//			}
//			else
//				continue;
//		}
//	}
//	lines.clear();
//	lines.assign(temp_lines.begin(), temp_lines.end());
//	std::cout << "the num of lines now " << lines.size() << endl;
//}

void c_joint_rec(vector<Vec2i>::iterator iter1, vector<Vec2i>& pts, vector<Vec3f> circles, vector<spe_point> points)
{
	// decide whether the point is a joint point and which kind of joints it is 
	Vec2i pt = (*iter1);
	spe_point spe_pt; Vec3f c = (0, 0, 0);
	int flag = on_circle(pt, circles);
	spe_pt.pt_location = (pt[0], pt[1]);
	string temp;
	if (flag == 1)
	{
		temp = "on_circle";
	}
	else if (flag != 0)
	{
		 temp = flag + "_circles_joints";
	}
	spe_pt.pt_properties.push_back(temp);
	for (vector<Vec2i>::iterator iter2 = iter1 + 1; iter2 != pts.end();iter2++)
	{
		int count = 0;
		if (same_pt(pt, *iter2))
		{
			++count;
			pts.erase(iter2--);
		}	
	}

	
	
}
void parse_points(vector<Vec4i> lines, vector<Vec3f> circles, vector<spe_point>& points)
{
	/* this function tend to parse the points with the location and property info in the graph */
	// first push the circles center to the points list
	for (size_t i = 0; i < circles.size(); i++)
	{
		spe_point pt; pt.pt_location = (circles[i][0], circles[i][1]); pt.pt_properties.push_back("circle_center");
		points.push_back(pt);
	}
	// then begin to handle other points
	vector<Vec2i> tempPointsList;
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec2i pt1, pt2;
		pt1 = (lines[i][0], lines[i][1]);
		pt2 = (lines[i][2], lines[i][3]);
		tempPointsList.push_back(pt1);
		tempPointsList.push_back(pt2);
	}
	for (vector<Vec2i>::iterator iter1 = tempPointsList.begin(); iter1 != tempPointsList.end();++iter1)
	{
		c_joint_rec(iter1, tempPointsList, circles, points);
	}


}
void parse_primitive(Mat &colorgeo, Mat &graygeo_blob, Mat& geometry_graph_bw, vector<Vec3f>& circles, vector<Vec4i>& lines, vector<spe_point>& points)
{
	/*the function meant to parse the primitives in the geometry graph, here we adopt the strategy to detect from up to bottom*/
	detect_circle(colorgeo, graygeo_blob, geometry_graph_bw, circles);
	//detect_circle2(colorgeo, graygeo_blob, geometry_graph_bw, circles);
	detect_lines_endpoints(colorgeo, graygeo_blob, geometry_graph_bw, lines, points);
	


}