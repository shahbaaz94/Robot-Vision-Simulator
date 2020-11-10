
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
#define PI 3.14159265

#include "image_transfer5.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 ) // needed for manual control



// obstacle position
double x_obs = 350;
double y_obs = 240;

// robots initial position
double x_0 = 400;
double y_0 = 100;
double theta_0 = -3.14159 / 4;

//ATTACK
double angle2p(double ig, double jg, double io, double jo, double &angle);
double distance(double ig, double jg, double io, double jo, double &dist);

//detect obstacle
void average_colour(image &rgb, image &label_image, int label_num, double &R, double &G, double &B, int &area);
int obs_properties(image rgbb, vector <int> &obs_label, vector <double> &obs_area, vector <double> &obs_centroids);

// trace, shoot or avoid
int go_there(double ig, double jg, double io, double jo, double angle_laser, int &pw_l, int &pw_r);
int parabellum(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double io, double jo, double il, double jl, static volatile int &flag, int &E);
int hunt(double anglecar, double angletarget, int &laser, int &pw_laser, double tc);
int turning(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double il, double jl, double io, double jo, static volatile int &turn);
int go_perpendicular(static volatile int turn, int io, int jo, int il, int jl, double angle_robot, double angle_l_o, int &pw_l, int &pw_r);
int avoid_obstacle(vector <double> obs_area, vector <double> obs_centroids, double ig, double jg, vector <double> &obs_turnr);

// functions not used
int detect_obs(double iO1_1, double jO1_1, double iO1_2, double jO1_2, double il, double jl, double angle_l_o, int &flag);
int screen_limit(double ig, double jg, double ir, double jr, int &pw_l, int &pw_r);
double twopoints(double ig, double jg, double iO1, double jO1, double area, double &iO1_1, double &jO1_1, double &iO1_2, double &jO1_2);


int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double xobs[50], yobs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	ibyte *p, *pc; // a pointer to a single byte
	ibyte R, G, B; // 0 - 255
	int height, width;
	double iO1_1, jO1_1, iO1_2, jO1_2, mig, mjg, mg, mir, mjr, mr, mio, mjo, mo, mib, mjb, mb;
	double angle, angle_laser, angle_robot, angle_l_o, dist, d_go, d_gb, d_ro, d_rb;
	int i, j, k, size, aa;
	double il, jl, ir, jr, ig, jg, io, jo, ib, jb;
	double area;
	double iO1;
	double jO1;
	static volatile int side, turnr;
	double ileft, iright, jleft, jright;
	int mode, level, avoid;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	double icen, jcen;
	vector <double> obs_turnr;
	int atrn;
	double icob, jcob, pdist, A, C, dist_l, dist_left, dist_right, angle_r_o, angle_bot, dist_g;
	int rcob, turnr1, out, got;

	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 1;

	xobs[1] = x_obs; // pixels
	yobs[1] = y_obs; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	//x_obs[2] = 135; // pixels
	//y_obs[2] = 135; // pixels
	//size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, xobs, yobs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
	//	ofstream fout("sim1.txt");
	//	fout << scientific;
	mode = 2;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = x_0;
	y0 = y_0;
	theta0 = theta_0;
	set_robot_position(x0, y0, theta0);	// set opponent initial position (pixels) and angle (rad)
	// x0 = 100;
	// y0 = 400;
	// theta0 = 3.14159/4;
	// set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// opponent inputs
	// pw_l_o = 1500; // pulse width for left wheel servo (us)
	// pw_r_o = 1500; // pulse width for right wheel servo (us)
	// pw_laser_o = 1500; // pulse width for laser servo (us)
	// laser_o = 0; // laser input (0 - off, 1 - fire)
	// set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		// opponent_max_speed);

	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.
	
	image rgb;
//	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);

	acquire_image_sim(rgb);
	
	//view_rgb_image(rgb);

	join_player();

	//vectors
	vector <int> obs_label;
	vector <double> obs_area;
	vector <double> obs_centroids;

	obs_properties(rgb, obs_label, obs_area, obs_centroids);
	for (i = 0; i < obs_centroids.size(); i = i + 1){
		cout << "\n vector" << obs_centroids[i];
	}
	
	aa=obs_label.size();
	
	tc0 = high_resolution_time(); 

	static volatile int flag, flag1;

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		p = rgb.pdata; // get image pointer

		mig = mjg = mg = 0.0;
		mir = mjr = mr = 0.0;
		mio = mjo = mo = 0.0;
		mib = mjb = mb = 0.0;

		// always initialize summation variables
		for (j = 0; j < height; j++) {
			for (i = 0; i < width; i++) { // image col, coord i

				k = i + width*j;
				pc = p + 3 * k; // pointer to the kth pixel (3 bytes / pixel)
				B = *pc;
				G = *(pc + 1);
				R = *(pc + 2);

				if ((50 < R) && (R < 90) && (160 < G) && (G < 200) && (110 < B) && (B < 150)){

					mg += 1.0; // m = 1 * G
					mig += 1.0 * i; // moment = pixel mass * i
					mjg += 1.0 * j; // moment = pixel mass * j
				}
				//find robot red
				if ((215 < R) && (100 > G) && (B < 90)){
					
					mr += 1.0; // m = 1 * G
					mir += 1.0 * i; // moment = pixel mass * i
					mjr += 1.0 * j; // moment = pixel mass * j
				}
				//find robot orange
				if ((240 < R) && (160 < G) && (B > 120)){
				
					mo += 1.0; // m = 1 * G
					mio += 1.0 * i; // moment = pixel mass * i
					mjo += 1.0 * j; // moment = pixel mass * j
				}
				//find robot blue
				if ((90 > R) && (100 < G) && (B > 200)){

					mb += 1.0; // m = 1 * G
					mib += 1.0 * i; // moment = pixel mass * i
					mjb += 1.0 * j; // moment = pixel mass * j
				}
			}
		}

		ir = mir / mr;
		jr = mjr / mr;
		ig = mig / mg;
		jg = mjg / mg;
		io = mio / mo;
		jo = mjo / mo;
		ib = mib / mb;
		jb = mjb / mb;

		//to move around opponent
		distance(ig, jg, io, jo, d_go);
		distance(ig, jg, ib, jb, d_gb);
		distance(ir, jr, io, jo, d_ro);
		distance(ir, jr, ib, jb, d_rb);

		/*for (j = jg - 2; j < jg + 2; j++) {
			for (i = ig - 2; i < ig + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}

		for (j = jr - 2; j < jr + 2; j++) {
			for (i = ir - 2; i < ir + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}

		for (j = jo - 2; j < jo + 2; j++) {
			for (i = io - 2; i < io + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}
*/
		/*for (j = jb - 2; j < jb + 2; j++) {
			for (i = ib - 2; i < ib + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}*/

		

		angle2p(ig, jg, ir, jr, angle_robot);
		il = ig + (31.0 * cos(angle_robot*(3.14159 / 180)));
		jl = jg + (31.0 * sin(angle_robot*(3.14159 / 180)));
		/*for (j = jl - 2; j < jl + 2; j++) {
			for (i = il - 2; i < il + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}*/
		ileft = il + (41.0 * cos((angle_robot + 90)*(3.14159 / 180)));
		iright = il - (41.0 * cos((angle_robot + 90)*(3.14159 / 180)));
		jleft = jl + (41.0 * sin((angle_robot + 90)*(3.14159 / 180)));
		jright = jl - (41.0 * sin((angle_robot + 90)*(3.14159 / 180)));
		/*for (j = jleft - 2; j < jleft + 2; j++) {
			for (i = ileft - 2; i < ileft + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}
		for (j = jright - 2; j < jright + 2; j++) {
			for (i = iright - 2; i < iright + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 255;
				*(pc + 1) = 255;
				*(pc + 2) = 255;
			}
		}*/
		angle2p(io, jo, il, jl, angle_l_o);
		angle_laser = angle_l_o - angle_robot;
		avoid_obstacle(obs_area, obs_centroids, ig, jg, obs_turnr);
		turnr1 = 0;
		

		parabellum(rgb, obs_area, obs_centroids, io, jo, il, jl, flag, avoid);

			if (flag == 0){
				hunt(angle_robot, angle_l_o, laser, pw_laser, tc);
		
				for (i = 0; i < obs_turnr.size(); i = i + 1){
					got = 0;
					if (obs_turnr[i] != 0){
						got = 1;
					}	
				}
				if (got == 0)
				{
					go_there(il, jl, io, jo, angle_laser, pw_l, pw_r);
				}
				for (i = 0; i < obs_centroids.size(); i = i + 2){
					atrn = obs_turnr[i / 2];

					if (atrn == 1){
						icob = obs_centroids[i];
						jcob = obs_centroids[i + 1];
						rcob = sqrt(obs_area[i / 2] / PI);
						turnr1 = atrn;
					}
				}
				angle2p(icob, jcob, ig, jg, angle_r_o);
				angle_bot = angle_r_o - angle_robot;
				distance(il, jl, icob, jcob, dist_l);
				distance(ileft, jleft, icob, jcob, dist_left);
				distance(iright, jright, icob, jcob, dist_right);

				if (turnr1 == 1){
					if ((dist_left > dist_right)){
						//turn anticlockwise
						pw_l = 2000;
						pw_r = 2000;
						if (((angle_bot <-60.00) && (angle_bot >-100.00)) || ((dist_l>rcob + 50))){
							pw_l = 1000;
							pw_r = 2000;
						}
					}
					else{
						//turn clockwise
						pw_l = 1000;
						pw_r = 1000;
						if (((angle_bot > 60.00) && (angle_bot < 100.00)) || ((dist_l>rcob + 50))){
							pw_l = 1000;
							pw_r = 2000;
						}
					}
				}				
			}			
			else{
				/*parabellum(rgb, obs_area, obs_centroids, il, jl, io, jo, flag1, avoid);*/
				
				/*if (flag1 == 1){
					turning(rgb, obs_area, obs_centroids, il, jl, io, jo, side);*/
					for (i = 0; i < obs_turnr.size(); i = i + 1){
						got = 0;
						if (obs_turnr[i] != 0){
							got = 1;
						}
					}
					if (got == 0)
					{
						//go_perpendicular(side, io, jo, ir, jr, angle_robot, angle_l_o, pw_l, pw_r);
						go_there(il, jl, io, jo, angle_laser, pw_l, pw_r);
					}
					for (i = 0; i < obs_centroids.size(); i = i + 2){
						atrn = obs_turnr[i / 2];

						if (atrn == 1){
							icob = obs_centroids[i];
							jcob = obs_centroids[i + 1];
							rcob = sqrt(obs_area[i / 2] / PI);
							turnr1 = atrn;
						}
					}
					angle2p(icob, jcob, ig, jg, angle_r_o);
					angle_bot = angle_r_o - angle_robot;
					distance(il, jl, icob, jcob, dist_l);
					distance(ileft, jleft, icob, jcob, dist_left);
					distance(iright, jright, icob, jcob, dist_right);

					if (turnr1 == 1){
						if ((dist_left > dist_right)){
							//turn anticlockwise
							pw_l = 2000;
							pw_r = 2000;
							if (((angle_bot <-60.00) && (angle_bot >-100.00)) || ((dist_l > rcob + 50))){
								pw_l = 1000;
								pw_r = 2000;
							}
						}
						else{
							//turn clockwise
							pw_l = 1000;
							pw_r = 1000;
							if (((angle_bot > 60.00) && (angle_bot < 100.00)) || ((dist_l>rcob + 50))){
								pw_l = 1000;
								pw_r = 2000;
							}
						}
					}
				}
			//}
		
			
			
			set_inputs(pw_l, pw_r, pw_laser, laser,
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);

			// set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
				// opponent_max_speed);
			//view_rgb_image(rgb);

			// don't need to simulate too fast
			Sleep(10); // 100 fps max
		
	}

		// free the image memory before the program completes
		free_image(rgb);

		deactivate_vision();

		deactivate_simulation();

		cout << "\ndone.\n";

		return 0;
	
}


void average_colour(image &rgb, image &label_image, int label_num, double &R, double &G, double &B, int &area)
{
	int i, j, k, label, N;
	int height, width; // ints are 4 bytes on the PC
	ibyte *p; // pointer to colour components in the rgb image
	i2byte *pl; // pointer to the label image

	height = rgb.height;
	width = rgb.width;

	p = rgb.pdata;
	pl = (i2byte *)label_image.pdata;

	// initialize the summation varibles to compute average colour
	R = 0.0;
	G = 0.0;
	B = 0.0;
	N = 0; // number of pixels with the label number of interest

	// method #3 -- pointers only !
	for (k = 0; k<width*height; k++) { // loop for kth pixel

		// how to get j and i from k ?
		i = k % width;
		j = (k - i) / width;
		label = *pl;

		// collect data if the pixel has the label of interest
		if (label == label_num) {
			N++;
			// 3 bytes per pixel -- colour in order BGR
			B += *p; // 1st byte in pixel
			G += *(p + 1); // 2nd byte in pixel
			R += *(p + 2); // 3rd
		}
		area = N;
		// increment pointers
		p += 3; // 3 bytes per pixel
		pl++;

	}

	// compute average colour
	R = R / N;
	G = G / N;
	B = B / N;

}

int obs_properties(image rgbb, vector <int> &obs_label, vector <double> &obs_area, vector <double> &obs_centroids)
{
	int nhist, j, nlabels, k, i, z, w, p, q, r, s, t,n;
	double hist[255], hmin, hmax, x, icen, jcen;
	image a, b, rgb0; // declare some image structures
	image label;
	int cam_number, height, width;
	int R, G, B, M;
	double are;
	double R_ave, G_ave, B_ave;

	activate_vision();

	// set camera number (normally 0 or 1)
	cam_number = 0;
	width = 640;
	height = 480;

	//	activate_camera(cam_number,height,width);

	rgbb.type = RGB_IMAGE;
	rgbb.width = 640;
	rgbb.height = 480;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	// set the type and size of the images
	a.type = GREY_IMAGE;
	a.width = 640;
	a.height = 480;

	b.type = GREY_IMAGE;
	b.width = 640;
	b.height = 480;

	label.type = LABEL_IMAGE;
	label.width = 640;
	label.height = 480;

	// allocate memory for the images
	allocate_image(a);
	allocate_image(b);
	allocate_image(label);
	allocate_image(rgbb);
	allocate_image(rgb0);

	acquire_image_sim(rgbb); // acquire an image from a video source (RGB format)

	//convert RGB image to a greyscale image
	copy(rgbb, rgb0);

	copy(rgbb, a);

	copy(a, rgbb);    // convert to RGB image format

	// scale the image to enhance contrast
	scale(a, b);
	copy(b, a); // put result back into image a

	copy(a, rgbb);    // convert to RGB image format

	save_rgb_image("grey.bmp", rgbb);

	lowpass_filter(a, b);
	copy(b, a);

	copy(a, rgbb);    // convert to RGB image format

	// make a histogram
	nhist = 60; // make 60 bins -- each bin is 255/60 range of intensity
	// eg bin1 = 0-3 
	// bin2 = 4-8,
	// etc.
	histogram(a, hist, nhist, hmin, hmax);

	// save to a csv file you can open/plot with microsoft excel
	// make sure the file is not currently open in excel before saving
	ofstream fout("hist1.csv");

	for (j = 0; j<nhist; j++) {
		x = hmin + (hmax - hmin) / nhist*j;
		fout << x << "," << hist[j] << "\n";
	}

	fout.close();

	// use threshold function to make a binary image (0,255)
	threshold(a, b, 70);
	copy(b, a);

	copy(a, rgbb);    // convert to RGB image format

	// invert the image
	invert(a, b);
	copy(b, a);

	copy(a, rgbb);    // convert to RGB image format

	// perform an erosion function to remove noise (small objects)
	erode(a, b);
	copy(b, a);

	// perform an erosion function to remove noise (small objects)
	erode(a, b);
	copy(b, a);

	copy(a, rgbb);    // convert to RGB image format

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(a, b);
	copy(b, a);

	dialate(a, b);
	copy(b, a);

	copy(a, rgbb);    // convert to RGB image format

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(a, label, nlabels);

	for (k = 1; k <= nlabels; k++) {
		average_colour(rgb0, label, k, R_ave, G_ave, B_ave, M);

		if ((42.00 <= R_ave&&R_ave <= 44.00) && (42.00 <= G_ave&&G_ave <= 44.00) && (45.00 <= B_ave&&B_ave <= 46.00)){
			//add k to array //maybe label
			obs_label.push_back(k);
			are=(double)M;
			obs_area.push_back(are);
			centroid(a, label, k, icen, jcen);
			obs_centroids.push_back(icen);
			obs_centroids.push_back(jcen);
		}

	}

	// save the image (make sure to use an RGB image)
	save_rgb_image("aout.bmp", rgbb);

	// free the image memory before the program completes
	free_image(a);
	free_image(b);
	free_image(label);
	free_image(rgbb);


	deactivate_vision();

	return 0;
}


double angle2p(double ig, double jg, double io, double jo, double &angle){

	if ((ig > io) && (jg >= jo)){
		angle = atan((double)(jg - jo) / (ig - io)) * 180.0 / 3.14159;
		
	}
	else if (ig < io){
		angle = atan((double)(jg - jo) / (ig - io)) * 180.0 / 3.14159 + 180;
	}
	else if ((ig > io) && (jg < jo)){
		angle = atan((double)(jg - jo) / (ig - io)) * 180.0 / 3.14159 + 360;
	}
	else{
		if (jg > jo){
			angle = 90;
		}
		else{
			angle = 270;
		}
	}
	
	return 0;

}

double distance(double ig, double jg, double io, double jo, double &dist){
	
	dist = sqrt(pow(ig-io, 2) + pow(jg-jo, 2));

	return 0;
}

double twopoints(double ig, double jg, double iO1, double jO1, double area, double &iO1_1, double &jO1_1, double &iO1_2, double &jO1_2){
	int i, j, k, height = 480, width = 640;
	ibyte *p, *pc;
	double radius = sqrt(area/PI); //to find manually or by labelling
	double ang;
	angle2p(iO1, jO1,ig, jg, ang);
	if ((ig<iO1) && (jg<jO1) ){		//1st quadrant
		iO1_1 = iO1 + (radius*cos((90 + ang)*(3.14159 / 180)));
		jO1_1 = jO1 + (radius*sin((90 + ang)*(3.14159 / 180)));
		iO1_2 = (2 * iO1) - iO1_1;
		jO1_2 = (2 * jO1) - jO1_1;
	}
	if ((ig > iO1) && (jg>jO1)){		//3nd quadrant
		iO1_1 = iO1 + (radius*cos((90 + ang)*(3.14159 / 180)));
		jO1_1 = jO1 + (radius*sin((90 + ang)*(3.14159 / 180)));
		iO1_2 = (2 * iO1) - iO1_1;
		jO1_2 = (2 * jO1) - jO1_1;
	}
	if ((ig>iO1) && (jg<jO1)){		//2nd quadrant
		iO1_1 = iO1 + (radius*cos((90 + ang)*(3.14159 / 180)));
		jO1_1 = jO1 + (radius*sin((90 + ang)*(3.14159 / 180)));
		iO1_2 = (2 * iO1) - iO1_1;
		jO1_2 = (2 * jO1) - jO1_1;
	}
	if ((ig<iO1) && (jg>jO1)){		//4th quadrant
		iO1_1 = iO1 + (radius*cos((90 + ang)*(3.14159 / 180)));
		jO1_1 = jO1 + (radius*sin((90 + ang)*(3.14159 / 180)));
		iO1_2 = (2 * iO1) - iO1_1;
		jO1_2 = (2 * jO1) - jO1_1;
	}
	
	return 0;
}

int detect_obs(double iO1_1, double jO1_1, double iO1_2, double jO1_2, double il, double jl, double angle_l_o,int &flag)
{
	double theta1, theta2;
	
	angle2p(iO1_1, jO1_1, il, jl, theta1);
	angle2p(iO1_2, jO1_2, il, jl, theta2);
	if (theta2 > theta1){
		if ((angle_l_o > theta2) || (angle_l_o < theta1))
			flag = 1;
		else(flag = 0);
	}
	else{
		if ((angle_l_o < theta2) || (angle_l_o > theta1))
			flag = 1;
			else(flag = 0);

	}
	return 0;
}

int hunt(double anglecar, double angletarget, int &laser, int &pw_laser, double tc){

	static volatile double t1, t2;
	static volatile int wait = 1, hitpermit = 0;

	// shoot when ready. don't shoot if not ready. (you can't instantly shut off the laser in this simulation)
	if ((angletarget <= 180) && (angletarget > 90) && (anglecar > angletarget - 90) && (anglecar < angletarget + 90))	{
		hitpermit = 1;
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 90) && (angletarget > 0) && (anglecar > 0) && (anglecar < angletarget + 90)){
		hitpermit = 1;
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 90) && (angletarget > 0) && (anglecar < 360) && (angletarget + 270 < anglecar)){
		hitpermit = 1;
		pw_laser = (-(anglecar - angletarget - 270) / 180 + 2) * 1000;
	}
	else if ((angletarget <= 270) && (angletarget > 180) && (anglecar > angletarget - 90) && (anglecar < angletarget + 90)){
		hitpermit = 1;
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 360) && (angletarget > 270) && (anglecar > 0) && (anglecar < angletarget - 270)){
		hitpermit = 1;
		pw_laser = ((-anglecar + angletarget -270) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 360) && (angletarget > 270) && (anglecar < 360) && (angletarget - 90 < anglecar)){
		hitpermit = 1;
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000; 
	}
	else {
		hitpermit = 0;
		laser = 0;
	}

	if (wait == 1 && hitpermit == 1) {//first round set initial timer and wait for pw_laser to be sent to simulation
		t1 = tc;
		wait = 0;
		//cout << "reload!" << endl;
		exit;
	}
	
	if (wait == 0){
		t2 = tc;

		if (((t2 - t1) > 1) && (hitpermit == 1)) {
			laser = 1;
		}

		if (((t2 - t1) > 2) && (laser == 1)){
			laser = 0;
			wait = 1;
		}
	}
	
	return 0;
}



int parabellum(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double io, double jo, double il, double jl, static volatile int &flag,int &E){
	//detect obs using obs_centroids obs_area io jo il jl are inputs
	//flag is output

	// there're 3 parts of this function: obstacle detection, collision detection and the algorithm to combine them.
	// if you want to comment them, always do it manually, don't use hotkey"Ctrl+K+C". Just don't
	// need A LOT OF TUNING.

	int i, j,k, width,a,b,c;
	double angle_obs, angle_l_o;
	double r;
	double theta1;
	double icen, jcen, area, distance_obs;
	ibyte *p, *pc;
	width = 640;
	
	for (i = 0; i < obs_centroids.size(); i = i + 2){
		icen = obs_centroids[i];
		jcen = obs_centroids[i + 1];
		area = obs_area[i/2];
		angle2p(icen, jcen, il, jl, angle_obs); // calculate the angle of our robot and obstacle to avoid
		distance(icen, jcen, il, jl, distance_obs);
		r = sqrt(area / PI);
		theta1 = asin(r / distance_obs) * 180.0 / PI;
		angle2p(io, jo, il, jl, angle_l_o);
		if (distance_obs <= r + 50){ E = 1; }
		else(E = 0);

		// check if laser is blocked by obstacle1
		if ((angle_obs >= theta1 && angle_obs <= 360 - theta1) && (angle_l_o <= angle_obs + theta1 && angle_l_o >= angle_obs - theta1)){
			a = 1;
		}
		else if (angle_obs < theta1 && (angle_l_o <= angle_obs + theta1 || angle_l_o >= 360 + angle_obs - theta1)){
			b = 1;
		}
		else if (360 - angle_obs < theta1 && (angle_l_o >= angle_obs - theta1 || angle_l_o <= angle_obs + theta1 - 360)){
			c = 1;
		}
	}
	if (a == 1 || b == 1 || c == 1){
		flag = 1;
	}
	else
		flag = 0;
	return 0;
}


int turning(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double il, double jl, double io, double jo, static volatile int &turn){
	//detect obs using obs_centroids obs_area io jo il jl are inputs
	//flag is output

	// there're 3 parts of this function: obstacle detection, collision detection and the algorithm to combine them.
	// if you want to comment them, always do it manually, don't use hotkey"Ctrl+K+C". Just don't
	// need A LOT OF TUNING.

	int i;
	double angle_obs, angle_l_o;
	double icen, jcen;

	for (i = 0; i < obs_centroids.size(); i = i + 2){
		icen = obs_centroids[i];
		jcen = obs_centroids[i + 1];
		angle2p(icen, jcen, io, jo, angle_obs); // calculate the angle of opponent robot and obstacle to avoid
		angle2p(il, jl, io, jo, angle_l_o);
		// check if laser is blocked by obstacle1
		if (angle_l_o <= angle_obs) {
			turn = 1;
		}
		else if (angle_l_o > angle_obs){
			turn=-1;
		}
	}
	return 0;
}




int go_perpendicular(static volatile int turn, int io, int jo, int il, int jl, double angle_robot, double angle_l_o, int &pw_l, int &pw_r){

	int v;
	double angle_diff;
	angle_diff = angle_l_o - angle_robot;
	v = 500;
	
	if (turn == 1){
		//q1
		if ((il > io) && (jl >= jo)){
			if (((angle_diff) > -90) && ((angle_diff) < 90)){
				// turn anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < -50.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff > -60.00)&&(angle_diff<-40.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}

		//q2
		if ((il < io) && (jl >= jo)){
			if (((angle_diff) > 90) && ((angle_diff) < 270)){
				//turn cloclwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff > 305.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < 325.00) && (angle_diff > 305.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}
		//q3
		if ((il < io) && (jl <= jo)){
			if (((angle_diff) < -90) && ((angle_diff) > -270)){
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff > -50)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//turn anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < -40.00) && (angle_diff >-60.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}
		//q4
		if ((il > io) && (jl <= jo)){
			if (((angle_diff) < 90) && ((angle_diff) > -90)){
				//anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < -50.00)){
				
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff < -40)&&(angle_diff>-60)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}
	}

	if (turn == -1){
		//q1
		if ((il > io) && (jl >= jo)){
			if (((angle_diff) > -90) && ((angle_diff) < 90)){
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff > 50.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff > 40.00) && (angle_diff < 60.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}

		//q2
		if ((il < io) && (jl >= jo)){
			if (((angle_diff) > 90) && ((angle_diff) < 270)){
				//anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < 50)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff > 40) && (angle_diff < 60)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}
		//q3
		if ((il < io) && (jl <= jo)){
			if (((angle_diff) < -90) && ((angle_diff) > -270)){
				//turn anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < -305.00) && (angle_diff >-325.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
				if ((angle_diff < -305.00) && (angle_diff >-325.00)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}
		//q4
		if ((il > io) && (jl <= jo)){
			if (((angle_diff) < 90) && ((angle_diff) > -90)){
				//turn clockwise
				pw_l = 1500 - v;
				pw_r = 1500 - v;
			    if ((angle_diff < 60)&&(angle_diff>40)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
			else{
				//turn anticlock
				pw_l = 1500 + v;
				pw_r = 1500 + v;
				if ((angle_diff < 60) && (angle_diff>40)){
					pw_l = 1500 - v;
					pw_r = 1500 + v;
				}
			}
		}
	}
	return 0;
}


int go_there(double ig, double jg,double io, double jo,double angle_laser, int &pw_l, int &pw_r){
	
	int v = 500;
	
	
	if ((ig > io) && (jg >= jo)){
		if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -10) && (angle_laser < +10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;

			}

		}
		else{
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}


	if ((ig < io) && (jg >= jo)){
		if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if ((ig < io) && (jg <= jo)){
		if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if ((ig > io) && (jg <= jo)){
		if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -10) && (angle_laser < 10)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}
	return 0;
}

int avoid_obstacle(vector <double> obs_area, vector <double> obs_centroids, double ig, double jg, vector <double> &obs_turnr) {
	
	double  dist_l, r, area, icen, jcen;
	int turnr,i;
	obs_turnr.clear(); 
	
	for (i = 0; i < obs_centroids.size(); i = i + 2){
		icen = obs_centroids[i];
		jcen = obs_centroids[i + 1];
		area = obs_area[i / 2];
		r = sqrt(area / PI);
		distance(ig, jg, icen, jcen, dist_l);
		turnr = 0;
		if ((dist_l < r + 120)){
			turnr = 1;
		}
	
		obs_turnr.push_back(turnr);
		
	}
	return 0;
}
int screen_limit(double ig, double jg, double ir, double jr, int &pw_l, int &pw_r){

	//double ig, jg, ir, jr,
	double angle_robot;
	angle2p(ig, jg, ir, jr, angle_robot);

	if ((ig > 580)){
		if ((angle_robot >= 0) && (angle_robot <= 180)){
			//turn anticlockwise
			pw_l = 1300;
			pw_r = 1300;
		}
		else {
			//turn clockwise
			pw_l = 1700;
			pw_r = 1700;
		}
	}


	if ((ig < 60)){
		if ((angle_robot >= 0) && (angle_robot<180)){
			//turn clockwise
			pw_l = 1700;
			pw_r = 1700;
		}
		else {
			//turn anticlockwise
			pw_l = 1300;
			pw_r = 1300;
		}
	}

	if ((jg > 420)){
		if ((angle_robot >= 90) && (angle_robot<270)){
			//turn anticlockwise
			pw_l = 1300;
			pw_r = 1300;
		}
		else {
			//turn clockwise
			pw_l = 1700;
			pw_r = 1700;
		}
	}
	if ((jg < 60)){
		if ((angle_robot >= 90) && (angle_robot<270)){
			//turn clockwise
			pw_l = 1700;
			pw_r = 1700;
		}
		else {
			//turn anticlockwise
			pw_l = 1300;
			pw_r = 1300;
		}
	}

	return 0;
}