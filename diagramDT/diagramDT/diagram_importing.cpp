// functions for diagram importing
#include "stdafx.h"
#include <iostream>
using namespace std;
void diagram_file_importing(char* diagram_file)
{
	char* diagram_path = "D:\\testpic";
	char* diagram_filename = "test2.png";
	sprintf_s(diagram_file, 200, "%s\\%s", diagram_path, diagram_filename);
	//cout << diagram_file << endl;
}



