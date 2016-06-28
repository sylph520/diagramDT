#include "stdafx.h"
struct spe_point
{
	Vec2i pt_location;
	vector<string> pt_properties;
	vector<Vec4i> on_which_line;
	vector<Vec3f> on_which_circle;
};
void parse_primitive(Mat& color, Mat& graygeo, Mat& bw, vector<Vec3f>& circles, vector<Vec4i>& lines, vector<spe_point>& points);
