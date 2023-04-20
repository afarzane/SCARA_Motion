// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "function.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <exception>
#include <iomanip>
using namespace std;
//#define resolution 10;

int main(int argc, char* argv[])
{
	JOINT home;
	JOINT start_frame;
	JOINT q_input = { 0, 0, 0, 0 };
	JOINT q_output = { 0,0,0,0 };
	double fourtuple[4];
	double Phi;
	double timef;
	char which_err;
	//double* q_temp;
	double** four_tuple = new double* [4];
	for (int i = 0; i < 4; i++) {
		four_tuple[i] = new double[4];
	}
	double** user_tuple = new double* [4];
	for (int i = 0; i < 4; i++) {
		user_tuple[i] = new double[4];
	}
	double** traj_frame = new double* [4];
	for (int i = 0; i < 4; i++) {
		traj_frame[i] = new double[4];
	}
	double** traj_joint = new double* [5];
	for (int i = 0; i < 5; i++) {
		traj_joint[i] = new double[4];
	}
	double** traj_tuple = new double* [4];
	for (int i = 0; i < 4; i++) {
		traj_tuple[i] = new double[4];
	}

	double* SOLVE_joint;

	// Set up the user interface
	printf("Keep this window in focus, and...\n");
	char ch;
	int c = 1;
	const int ESC = 27;

	printf("Press any key to continue \n");
	printf("Press ESC to exit \n");
	ch = _getch();

	bool a = GetConfiguration(home);

	while (1)
	{
		if (c != ESC)
		{
			// Provide options for user
			cout << "Press 1 for forward kinematics (joints input)" << endl;
			cout << "Press 2 for inverse kinematics (X,Y,Z,Phi input)" << endl;
			cout << "Press 3 to return to home position" << endl;
			cout << "Press 4 to control the gripper" << endl;
			cout << "Press 5 to stop the robot" << endl;
			cout << "Press 6 to reset the robot" << endl;
			cout << "Press 7 to plan a trajectory" << endl;
			ch = _getch();

			if (ch == '1')
			{

				// Ask for input for each joint and check if it exceeds joint limits
				try {
					printf("What is your input for revolute joint 1? ");
					cin >> q_input[0];
					if (q_input[0] > 150 || q_input[0] < -150)
						throw 0;

					printf("What is your input for revolute joint 2? ");
					cin >> q_input[1];
					if (q_input[1] > 100 || q_input[1] < -100)
						throw 1;

					printf("What is your input for prismatic joint 3? ");
					cin >> q_input[2];
					if (q_input[2] > -100 || q_input[2] < -200)
						throw 2;

					printf("What is your input for revolute joint 4? ");
					cin >> q_input[3];
					if (q_input[3] > 160 || q_input[3] < -160)
						throw 3;

					// Find Transformation matrix of tool frame w.r.t. station frame
					//four_tuple = WHERE(q_input);
					four_tuple = ForwKin(q_input);

					printf("The transformation matrix T_ST is: \n");
					printMatrix(four_tuple);
					MoveToConfiguration(q_input);
					//DisplayConfiguration(q_input);
					Phi = RAD2DEG(acos(four_tuple[0][0]));

					fourtuple[0] = four_tuple[0][3];
					fourtuple[1] = four_tuple[1][3];
					fourtuple[2] = four_tuple[2][3];
					fourtuple[3] = Phi;
					cout << "Four tuple is [" << fourtuple[0] << ", " << fourtuple[1] << ", " << fourtuple[2] << ", " << fourtuple[3] << "]" << endl;
				}
				catch (int x) {
					printf("Input exceeds limit! Refer to the instruction manual for the joint angle limits.\n");
				}


			}
			else if (ch == '2')
			{
				bool xbool = false;
				bool ybool = false;
				bool zbool = false;
				bool abool = false;
				try {

					printf("What is your input for X? ");
					cin >> q_input[0];
					if (q_input[0] > 337 || q_input[0] < -311)
					{
						xbool = true;
						throw 4;
					}

					printf("What is your input for Y? ");
					cin >> q_input[1];
					if (q_input[1] > 337 || q_input[1] < -337)
					{
						ybool = true;
						throw 5;
					}
					printf("What is your input for Z? ");
					cin >> q_input[2];
					//if (q_input[2] > 125 || q_input[2] < 25)
					//{
					//	zbool = true;
					//	throw 6;
					//}
					if (sqrt(q_input[0] * q_input[0] + q_input[1] * q_input[1]) > 337 || sqrt(q_input[0] * q_input[0] + q_input[1] * q_input[1] < 53)) {
						abool = true;
						throw 7;
					}
					printf("What is your input for Phi? ");
					cin >> q_input[3];
					q_input[3] = DEG2RAD(q_input[3]);
					user_tuple = Joint_To_Tuple(q_input);
					printMatrix(user_tuple);

					SOLVE_joint = SOLVE(user_tuple);
					//SOLVE_joint = SOLVE(four_tuple);
					q_output[0] = SOLVE_joint[0];
					q_output[1] = SOLVE_joint[1];
					q_output[2] = SOLVE_joint[2];
					q_output[3] = SOLVE_joint[3];

					for (int i = 0; i < 4; i++) {
						cout << "joint " << i + 1 << ": " << SOLVE_joint[i] << endl;
					}

					MoveToConfiguration(q_output);
					//DisplayConfiguration(q_output);
				}
				catch (int error)
				{
					if (xbool == true)
					{
						which_err = 'X';
						xbool = false;
						cout << which_err << " value is out of bounds of workspace. Please refer to the SCARA workspace limits for more information." << endl;
					}
					else if (ybool == true)
					{
						which_err = 'Y';
						ybool = 0;
						cout << which_err << " value is out of bounds of workspace. Please refer to the SCARA workspace limits for more information." << endl;
					}
					else if (zbool == true)
					{
						which_err = 'Z';
						zbool = 0;
						cout << which_err << " value is out of bounds of workspace. Please refer to the SCARA workspace limits for more information." << endl;
					}
					else if (abool == true) {
						cout << "It is out of the worksapce" << endl;
						abool = 0;

					}

				}


			}
			else if (ch == '3')
			{
				MoveToConfiguration(home);
				//DisplayConfiguration(home);
			}
			else if (ch == '4') {
				Is_Grasp();
			}
			else if (ch == '5') {
				StopRobot();
			}
			else if (ch == '6') {
				ResetRobot();
				MoveToConfiguration(home);
				Grasp(false);
			}
			else if (ch == '7') {
				bool is_within_workspace = false;

				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++) {
						traj_frame[i][j] = 0;
					}
				}

				for (int i = 0; i < 5; i++) {
					for (int j = 0; j < 4; j++) {
						traj_joint[i][j] = 0;
					}
				}

				try {
					//ask user to input the local of START, three intermediate, and goal tool frame
					cout << "Choose the first intermediate tool frame (X,Y,Z,Phi): " << endl;
					cin >> traj_frame[0][0] >> traj_frame[0][1] >> traj_frame[0][2] >> traj_frame[0][3];
					if (sqrt(traj_frame[0][0] * traj_frame[0][0] + traj_frame[0][1] * traj_frame[0][1]) > 337 || sqrt(traj_frame[0][0] * traj_frame[0][0] + traj_frame[0][1] * traj_frame[0][1] < 53)) {
						is_within_workspace = true;
						throw 2;
					}
					cout << "Choose the second intermediate tool frame (X,Y,Z,Phi): " << endl;
					cin >> traj_frame[1][0] >> traj_frame[1][1] >> traj_frame[1][2] >> traj_frame[1][3];
					if (sqrt(traj_frame[1][0] * traj_frame[1][0] + traj_frame[1][1] * traj_frame[1][1]) > 337 || sqrt(traj_frame[1][0] * traj_frame[1][0] + traj_frame[1][1] * traj_frame[1][1] < 53)) {
						is_within_workspace = true;
						throw 3;
					}
					cout << "Choose the third intermediate tool frame (X,Y,Z,Phi): " << endl;
					cin >> traj_frame[2][0] >> traj_frame[2][1] >> traj_frame[2][2] >> traj_frame[2][3];
					if (sqrt(traj_frame[2][0] * traj_frame[2][0] + traj_frame[2][1] * traj_frame[2][1]) > 337 || sqrt(traj_frame[2][0] * traj_frame[2][0] + traj_frame[2][1] * traj_frame[2][1] < 53)) {
						is_within_workspace = true;
						throw 4;
					}
					cout << "Choose the GOAL tool frame (X,Y,Z,Phi): " << endl;
					cin >> traj_frame[3][0] >> traj_frame[3][1] >> traj_frame[3][2] >> traj_frame[3][3];
					if (sqrt(traj_frame[3][0] * traj_frame[3][0] + traj_frame[3][1] * traj_frame[3][1]) > 337 || sqrt(traj_frame[3][0] * traj_frame[3][0] + traj_frame[3][1] * traj_frame[3][1] < 53)) {
						is_within_workspace = true;
						throw 4;
					}
					cout << "Choose the total time for the manipulator to travel: " << endl;
					cin >> timef;

					cout << "You have input the following locations: " << endl;

					// For testing
					/*traj_frame[0][0] = 0;
					traj_frame[0][1] = 337;
					traj_frame[0][2] = 100;
					traj_frame[0][3] = 0;*/

					// Test Case 1
					/*traj_frame[0][0] = -206;
					traj_frame[0][1] = 260;
					traj_frame[0][2] = 105;
					traj_frame[0][3] = 140;

					traj_frame[1][0] = -265;
					traj_frame[1][1] = 174;
					traj_frame[1][2] = 125;
					traj_frame[1][3] = 125;

					traj_frame[2][0] = -291;
					traj_frame[2][1] = 125;
					traj_frame[2][2] = 125;
					traj_frame[2][3] = 120;

					traj_frame[3][0] = -291;
					traj_frame[3][1] = 27;
					traj_frame[3][2] = 125;
					traj_frame[3][3] = 110;

					timef = 10;*/

					// Test case 2
					/*traj_frame[0][0] = -206;
					traj_frame[0][1] = 260;
					traj_frame[0][2] = 105;
					traj_frame[0][3] = 140;

					traj_frame[1][0] = -265;
					traj_frame[1][1] = 174;
					traj_frame[1][2] = 125;
					traj_frame[1][3] = 125;

					traj_frame[2][0] = -291;
					traj_frame[2][1] = 125;
					traj_frame[2][2] = 125;
					traj_frame[2][3] = 120;

					traj_frame[3][0] = -291;
					traj_frame[3][1] = 27;
					traj_frame[3][2] = 125;
					traj_frame[3][3] = 110;

					timef = 0.1;
					timef = 1.7;*/

					// Test case 3
					/*traj_frame[0][0] = -206;
					traj_frame[0][1] = 260;
					traj_frame[0][2] = 105;
					traj_frame[0][3] = 140;

					traj_frame[1][0] = -265;
					traj_frame[1][1] = 17400;
					traj_frame[1][2] = 125;
					traj_frame[1][3] = 125;

					traj_frame[2][0] = -291;
					traj_frame[2][1] = 125;
					traj_frame[2][2] = 125;
					traj_frame[2][3] = 120;

					traj_frame[3][0] = -291;
					traj_frame[3][1] = 27;
					traj_frame[3][2] = 125;
					traj_frame[3][3] = 110;

					timef = 5;*/

					// Test case 4
					/*traj_frame[0][0] = 337;
					traj_frame[0][1] = 0;
					traj_frame[0][2] = 40;
					traj_frame[0][3] = 0;

					traj_frame[1][0] = 195;
					traj_frame[1][1] = 142;
					traj_frame[1][2] = 40;
					traj_frame[1][3] = 90;

					traj_frame[2][0] = 0;
					traj_frame[2][1] = -337;
					traj_frame[2][2] = 50;
					traj_frame[2][3] = 45;

					traj_frame[3][0] = 195;
					traj_frame[3][1] = -142;
					traj_frame[3][2] = 50;
					traj_frame[3][3] = 0;

					timef = 20;*/

					// Test case 5
					/*traj_frame[0][0] = -115;
					traj_frame[0][1] = 316;
					traj_frame[0][2] = 100;
					traj_frame[0][3] = 110;

					traj_frame[1][0] = -168;
					traj_frame[1][1] = 292;
					traj_frame[1][2] = 100;
					traj_frame[1][3] = 120;

					traj_frame[2][0] = -216;
					traj_frame[2][1] = 258;
					traj_frame[2][2] = 100;
					traj_frame[2][3] = 130;

					traj_frame[3][0] = -291;
					traj_frame[3][1] = 168;
					traj_frame[3][2] = 100;
					traj_frame[3][3] = 150;

					timef = 5;*/
					/*traj_frame[0][0] = 337;
					traj_frame[0][1] = 0;
					traj_frame[0][2] = 80;
					traj_frame[0][3] = 130;

					traj_frame[1][0] = -279;
					traj_frame[1][1] = 137;
					traj_frame[1][2] = 100;
					traj_frame[1][3] = 130;

					traj_frame[2][0] = -279;
					traj_frame[2][1] = 137;
					traj_frame[2][2] = 120;
					traj_frame[2][3] = 130;

					traj_frame[3][0] = -279;
					traj_frame[3][1] = 137;
					traj_frame[3][2] = 120;
					traj_frame[3][3] = 110;

					timef = 15;*/

					//funciton to check the workspace for each tool frame, function writen but do this later
					//is_within_workspace = Check_Workspace(traj_frame[0][0], traj_frame[0][1], traj_frame[0][2], traj_frame[0][3]);

					cout << "x" << setw(4) << "y" << setw(4) << "z" << setw(4) << "phi" << setw(4) << "time (s)" << timef << endl;
					//displace our input for each tool frame in a 5 by 5 matrix
					printMatrix(traj_frame);

					export_csv(traj_frame);

					//Use inverse kin to conver from cartesian to joints and print the joints for each tool frame
					for (int i = 0; i < 5; i++) {
						if (i != 0) {
							traj_frame[i - 1][3] = DEG2RAD(traj_frame[i - 1][3]);
							traj_tuple = Joint_To_Tuple(traj_frame[i - 1]);
							traj_joint[i] = SOLVE(traj_tuple);
						}
						else {
							GetConfiguration(start_frame);
							traj_joint[i] = start_frame;
						}

					}
					cout << "joint 1" << setw(5) << "joint 2" << setw(5) << "joint 3" << setw(5) << "joint 4" << endl;
					printMatrix5(traj_joint);


					//Perform Trajectory Path
					Trajectory_Planner(traj_joint, timef);

					ResetRobot();
				}

				catch (int error) {
					if (is_within_workspace == true) {
						cout << "It is out of the worksapce" << endl;
						is_within_workspace = false;

					}
				}
			}





			printf("Press any key to continue \n");
			printf("Press ESC to exit \n");
			c = _getch();
			cout << endl;
		}
		else {
			break;
		}
	}

	return 0;
}
