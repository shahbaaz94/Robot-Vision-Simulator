
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


// DEFEND
// some basic functions
int go_there(double ig, double jg, double io, double jo, double angle_robot, int &pw_l, int &pw_r);
int set_up(image &rgb, double &ig, double &jg, double &ir, double &jr, double &io, double &jo, double &ib, double &jb);
double angle2p(double ig, double jg, double io, double jo, double &angle);
double distance(double ig, double jg, double io, double jo, double &dist);
double twopoints(double ig, double jg, double iO1, double jO1, double area, double &iO1_1, double &jO1_1, double &iO1_2, double &jO1_2);

//calculates average RBG values of labels to identify obstacles
void average_colour(image &rgb, image &label_image, int label_num, double &R, double &G, double &B, int &area);
//returns vectors of centroids, areas and label numbers of obstacles
int obs_properties(image rgbb, vector <int> &obs_label, vector <double> &obs_area, vector <double> &obs_centroids);

// defense mode functions
int enemycontrol(image &rgb, int &pw_l_o, int &pw_r_o, double io, double jo, double ib, double jb);
int hide(image &rgb, double io, double jo, double ib, double jb, double icen, double jcen, double r, double ig, double jg, double angle_robot, int &pw_l, int &pw_r, double ir, double jr);
int drawtrajectory(vector <int> &trajectory, image &rgb, double il, double jl);
int go_there_defense(double icen, double jcen, double ig, double jg, double io, double jo, double ir, double jr, double angle_robot, int &pw_l, int &pw_r);
int satellite(double ig, double jg, double angle_robot, double icen, double jcen, int &pw_l, int &pw_r, int &inorbit, double io, double jo);

// function not used(attack mode) or unfinished
int detect_obs(double iO1_1, double jO1_1, double iO1_2, double jO1_2, double il, double jl, double angle_l_o, int &flag);
int hunt(double anglecar, double angletarget, int &laser, int &pw_laser, double tc);
int parabellum(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double io, double jo, double il, double jl, vector <int> &flag, int &E);
int turning(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double il, double jl, double io, double jo, static volatile int &turn);
int go_perpendicular(static volatile int turn, int io, int jo, int il, int jl, double angle_robot, double angle_l_o, int &pw_l, int &pw_r);
int avoid_obstacle(vector <double> obs_area, vector <double> obs_centroids, double angle_laser, double il, double jl, double io, double jo, double ig, double jg, double ir, double jr, double ileft, double iright, double jleft, double jright, int  &pw_l, int &pw_r, static volatile int &obstacle);
int rotate(double il, double jl, double angle_robot, int &pw_l, int &pw_r, double icen, double jcen, double collideangle, static volatile int &avoided);


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
	int height, width;
	double iO1_1, jO1_1, iO1_2, jO1_2, ir, jr, ig, jg, io, jo, ib, jb, angle, angle_laser, angle_robot, angle_l_o, dist, d_go, d_gb, d_ro, d_rb;
	int i, j, k, size, aa;
	double il, jl, area, iO1, jO1;
	static volatile int side;
	double ileft, iright, jleft, jright;
	int mode, level, avoid;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;

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


	//x_obs[1] = 350; // pixels
	//y_obs[1] = 240; // pixels
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
	set_robot_position(x0, y0, theta0);
	//set_opponent_position(x0,y0,theta0);

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
	// pw_l_o = 1300; // pulse width for left wheel servo (us)
	// pw_r_o = 1600; // pulse width for right wheel servo (us)
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
	
	//center = obs_properties(rgb, obs_label, obs_area);
	// measure initial clock time

	obs_properties(rgb, obs_label, obs_area, obs_centroids);
	//cout << "\n vector" << obs_centroids[0];
	//aa =2;
	aa=obs_label.size();

	// new variables for parabellum and defense
	vector <int> flag(aa);		
	vector <int> trajectory;
	double icen, jcen, area1, r, distobmin = 720.0, distob;
	int blocked, imin;
	static volatile int flag1;
	int inorbit = 0, adjust = 0;

	tc0 = high_resolution_time(); 


	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;		
		
		set_up(rgb, ig, jg, ir, jr, io, jo, ib, jb);
		
		// more preparation		
		distance(ig, jg, io, jo, d_go);
		distance(ig, jg, ib, jb, d_gb);
		distance(ir, jr, io, jo, d_ro);
		distance(ir, jr, ib, jb, d_rb);		
		
		angle2p(ig, jg, ir, jr, angle_robot);
		il = ig + (31.0 * cos(angle_robot*(3.14159 / 180)));
		jl = jg + (31.0 * sin(angle_robot*(3.14159 / 180)));
		draw_point_rgb(rgb, il, jl, 255, 255, 255);
		
		angle2p(io, jo, il, jl, angle_l_o);
		
		angle_laser = angle_l_o - angle_robot;
		
		
		// angle2p(io, jo, ig + (31.0 * cos(angle_robot*(3.14159 / 180))), jg + (31.0 * sin(angle_robot*(3.14159 / 180))), angle_l_o);
		//if (angle_l_o > angle_robot)
		//cout << "\nangle_robot = " << angle_robot;
		//cout << "\tangle_l_o = " << angle_l_o;
		//cout << "\tdifference = " << angle_laser; 
		//if ((d_go < 37.0) || (d_gb < 50.0) || (d_ro < 50.0) || (d_rb < 50.0)){
		//pw_l = 1700;
		//pw_r = 1300;
		//}
		//cout << "\ndistance =" << d_go <<"\t"<< d_gb;

/*		turning(rgb, obs_area, obs_centroids, il, jl, io, jo, side);
		go_perpendicular(side, io, jo, ig, jg, angle_robot, angle_l_o, pw_l, pw_r);
		int avoided;

		rotate(il, jl, angle_robot, pw_l, pw_r, 270, 270, 300.0, avoided);

		avoid_obstacle(obs_area, obs_centroids, angle_laser, il, jl, io, jo, ig, jg, ir, jr, ileft, iright, jleft, jright, pw_l, pw_r, avoid);

		drawtrajectory(trajectory, rgb, il, jl);
		go_there(il, jl, 270, 270, angle_robot, pw_l, pw_r);

		 enemycontrol
		enemycontrol(rgb, pw_l_o, pw_r_o, io, jo, ib, jb);*/	


		// defense algorithm
		parabellum(rgb, obs_area, obs_centroids, io, jo, ig, jg, flag, avoid);
		
		// Algorithm1: if blocked,hide behind;if not, go find the closest and hide behind
		//blocked = 0;
		//for (i = 0; i < flag.size(); i++){	
		//	if (flag[i]){
		//		icen = obs_centroids[2 * i];
		//		jcen = obs_centroids[2 * i + 1];
		//		area1 = obs_area[i];
		//		r = sqrt(area1 / PI);
		//		cout << "avoiding i: " << i << "flag : " << flag[i] << endl;
		//		//cout << " icen : " << icen << " jcen : " << jcen << " r = " << r << endl;
		//		hide(rgb, io, jo, ib, jb, icen, jcen, r, ig, jg, angle_robot, pw_l, pw_r,ir,jr);
		//	}
		//	blocked += flag[i];
		//	//cout << "i: " << i << "flag : " << flag[i] << endl;
		//}
		//if (blocked == 0){
		//	cout << "DANGER!!" << endl;
		//	for (i = 0; i < flag.size(); i++){
		//		icen = obs_centroids[2 * i];
		//		jcen = obs_centroids[2 * i + 1];				
		//		distance(icen, jcen, il, jl, distob);
		//		if (distob < distobmin) 	imin = i;			
		//	}
		//	icen = obs_centroids[2 * imin];
		//	jcen = obs_centroids[2 * imin + 1];
		//	hide(rgb, io, jo, ib, jb, icen, jcen, r, ig, jg, angle_robot, pw_l, pw_r,ir,jr);
		//}		

		//tactic2: hide behind no matter what
		blocked = 0;
		for (i = 0; i < flag.size(); i++){
				icen = obs_centroids[2 * i];
				jcen = obs_centroids[2 * i + 1];				
				distance(icen, jcen, il, jl, distob);
				cout << icen << "   " << jcen << "  " << i << endl;
				if (distob < distobmin) 	{
					imin = i;
					distobmin = distob;
				}
				if(flag[i]) blocked += flag[i];
		}
		if (blocked == 0)  cout << "DANGER!!" << endl;
		icen = obs_centroids[2 * imin];
		jcen = obs_centroids[2 * imin + 1];
		cout << icen << "   " << jcen << endl;
		area1 = obs_area[imin];
		r = sqrt(area1 / PI);		
		hide(rgb, io, jo, ib, jb, icen, jcen, r, ig, jg, angle_robot, pw_l, pw_r,ir,jr);
		
		//tactic3: satellite method
		//blocked = 0;
		//for (i = 0; i < flag.size(); i++){
		//	icen = obs_centroids[2 * i];
		//	jcen = obs_centroids[2 * i + 1];
		//	distance(icen, jcen, il, jl, distob);
		//	if (distob < distobmin) 	imin = i;
		//	if (flag[i]) blocked += flag[i];
		//}
		//if (blocked == 0)  cout << "DANGER!!" << endl;
		//icen = obs_centroids[2 * imin];
		//jcen = obs_centroids[2 * imin + 1];
		//distance(icen, jcen, ig, jg, distob);
		//area1 = obs_area[imin];
		//r = sqrt(area1 / PI);
		//int range = 65;
		////cout << "Real dist = " << distob << "  Range = " << r + range << endl;
		//if (distob >= r + range && inorbit == 0){
		//	cout << "launching" << endl;
		//	hide(rgb, io, jo, ib, jb, icen, jcen, r, ig, jg, angle_robot, pw_l, pw_r, ir, jr);
		//}
		//else if (inorbit == 0){
		//	cout << "entering orbit...";
		//	if (fabs(ig - icen)>5 && fabs(jg - jcen)>5 && adjust == 0)	{
		//		hide(rgb, io, jo, ib, jb, icen, jcen, r, ig, jg, angle_robot, pw_l, pw_r, ir, jr);
		//		cout << "finding orbit point..." << endl;
		//	}
		//	else {
		//		adjust = 1;
		//		cout << "adjusting to orbit..." << endl;
		//		satellite(ig, jg, angle_robot, icen, jcen, pw_l, pw_r, inorbit, io, jo);
		//	}
		//}
		//else {
		//	cout << "in orbit! orbiting..." << endl;
		//	satellite(ig, jg, angle_robot, icen, jcen, pw_l, pw_r, inorbit, io, jo);
		//}



			set_inputs(pw_l, pw_r, pw_laser, laser,
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);

			// set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
				// opponent_max_speed);
			// view_rgb_image(rgb);

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


int set_up(image &rgb, double &ig, double &jg, double &ir, double &jr, double &io, double &jo, double &ib, double &jb){

	double mig, mjg, mg, mir, mjr, mr, mio, mjo, mo, mib, mjb, mb;
	ibyte *p, *pc; // a pointer to a single byte
	ibyte R, G, B; // 0 - 255
	int i, j, k,height = 480, width = 640;


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

			// find robot green
			//if ((90 > R) && (160 < G) && (B < 150)){
			if ((50 < R) && (R < 90) && (160 < G) && (G < 200) && (110 < B) && (B < 150)){

				//(90 > R > 50) && (160 < G < 200) && (110 < B < 150)
				mg += 1.0; // m = 1 * G
				mig += 1.0 * i; // moment = pixel mass * i
				mjg += 1.0 * j; // moment = pixel mass * j
			}
			//find robot red
			if ((215 < R) && (100 > G) && (B < 90)){
				//(90 > R > 50) && (160 < G < 200) && (110 < B < 150)
				mr += 1.0; // m = 1 * G
				mir += 1.0 * i; // moment = pixel mass * i
				mjr += 1.0 * j; // moment = pixel mass * j
			}
			//find robot orange
			if ((240 < R) && (160 < G) && (B > 120)){
				//(90 > R > 50) && (160 < G < 200) && (110 < B < 150)
				mo += 1.0; // m = 1 * G
				mio += 1.0 * i; // moment = pixel mass * i
				mjo += 1.0 * j; // moment = pixel mass * j
			}
			//find robot blue
			if ((90 > R) && (100 < G) && (B > 200)){
				//(90 > R > 50) && (160 < G < 200) && (110 < B < 150)
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
	
	// for player 2, orange is the new green=our color
	/*ib = mir / mr;
	jb = mjr / mr;
	io = mig / mg;
	jo = mjg / mg;
	ig = mio / mo;
	jg = mjo / mo;
	ir = mib / mb;
	jr = mjb / mb;*/
	
	//	cout << "\nir = " << ir << "\tjr = " << jr << "\tig = " << ig << "\tjg = " << jg << "\tio = " << io << "\tjo = " << jo << "\tib = " << ib << "\tjb = " << jb;

	for (j = jg - 2; j < jg + 2; j++) {
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

	for (j = jb - 2; j < jb + 2; j++) {
		for (i = ib - 2; i < ib + 2; i++) {

			k = i + width*j;
			pc = p + 3 * k;

			*pc = 255;
			*(pc + 1) = 255;
			*(pc + 2) = 255;
		}
	}

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
	//double R_out, G_out, B_out;
	//m = 2;

	//vector <vector<double>> centroids(n, vector<double>(m));

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

	/*	for (k = 1; k <= nlabels; k++) {

	// compute the centroid of the last object
	centroid(a, label, k, icen, jcen);

	copy(a, rgbb);    // convert to RGB image format

	// mark the centroid point on the image
	R = 255; G = 0; B = 255;
	}
	*/
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

	/*m = 2;
	n = obs_label.size();
	cout << "\n label size"<<n;
	for (r = 0; r < n; r++) {
		s = obs_label.at(r);
		cout << "\n label number" << s;
		centroid(a, label, s, icen, jcen);
		cout << "\n icen value"<<icen;

		t = 0;
		centroids[r][t] = { icen, jcen };
		t++;
		
		centroids[r][t] = jcen;
		cout << "\n jcen value" << jcen;

	}*/

	// save the image (make sure to use an RGB image)
	save_rgb_image("aout.bmp", rgbb);

	// free the image memory before the program completes
	free_image(a);
	free_image(b);
	free_image(label);
	free_image(rgbb);


	deactivate_vision();

	//cout << "\n\ndone.\n";
	////pause();

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
	/*if (((ig<iO1) && (jg<jO1)) || ((ig>iO1) && (jg>jO1))){
		iO1_1 = iO1 + (radius*cos((90 + ang)*(3.14159 / 180)));
		jO1_1 = jO1 + (radius*sin((90 + ang)*(3.14159 / 180)));
		iO1_2 = iO1 - (radius*cos((90 + ang)*(3.14159 / 180)));
		jO1_2 = jO1 - (radius*sin((90 + ang)*(3.14159 / 180)));
	}
	if (((ig>iO1) && (jg<jO1)) || ((ig<iO1) && (jg>jO1))){
		iO1_1 = iO1 + (radius*cos((90 - ang)*(3.14159 / 180)));
		jO1_1 = jO1 + (radius*sin((90 - ang)*(3.14159 / 180)));
		iO1_2 = iO1 - (radius*cos((90 - ang)*(3.14159 / 180)));
		jO1_2 = jO1 - (radius*sin((90 - ang)*(3.14159 / 180)));
	}*/
	

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
	//	cout << "4" << endl; // if opponent is the centroid, quadrant 4
	//	cout << "anlgeT " << angletarget << "angleC " << anglecar << endl;
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 90) && (angletarget > 0) && (anglecar > 0) && (anglecar < angletarget + 90)){
		hitpermit = 1;
	//	cout << "anlgeT " << angletarget << "angleC " << anglecar << endl;
	//	cout << "3" << endl; // quadrant 3
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 90) && (angletarget > 0) && (anglecar < 360) && (angletarget + 270 < anglecar)){
		hitpermit = 1;
	//	cout << "3" << endl; // quadrant 3
		//cout << "anlgeT " << angletarget << "angleC " << anglecar << endl;
		pw_laser = (-(anglecar - angletarget - 270) / 180 + 2) * 1000;
	}
	else if ((angletarget <= 270) && (angletarget > 180) && (anglecar > angletarget - 90) && (anglecar < angletarget + 90)){
		hitpermit = 1;
	//	cout << "anlgeT " << angletarget << "angleC " << anglecar << endl;
	//	cout << "1" << endl; // quadrant 1
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 360) && (angletarget > 270) && (anglecar > 0) && (anglecar < angletarget - 270)){
		hitpermit = 1;
		//cout << "2" << endl; // quadrant 2
	//	cout << "anlgeT " << angletarget << "angleC " << anglecar << endl;
		pw_laser = ((-anglecar + angletarget -270) / 180 + 1) * 1000;
	}
	else if ((angletarget <= 360) && (angletarget > 270) && (anglecar < 360) && (angletarget - 90 < anglecar)){
		hitpermit = 1;
		//cout << "2" << endl; // quadrant 2
		//cout << "anlgeT " << angletarget << "angleC " << anglecar << endl;
		pw_laser = ((angletarget - anglecar + 90) / 180 + 1) * 1000; 
	}
	else {
		cout << "impossible to hit" << endl;
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

int parabellum(image &rgb, vector <double> obs_area, vector <double> obs_centroids, double io, double jo, double il, double jl, vector <int> &flag, int &E){
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
	/*for (n = 0; n < 50; n++){
		draw_point_rgb(rgb, iL + (iB1 - iL) / 50.0 * n, jL + (jB1 - jL) / 50.0 * n, 255, 255, 255);
		}*/

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


	/*	for (j = jcen - 2; j < jcen + 2; j++) {
			for (i = icen - 2; i < icen + 2; i++) {

				k = i + width*j;
				pc = p + 3 * k;

				*pc = 0;
				*(pc + 1) = 0;
				*(pc + 2) = 0;
			}
		}*/

		// check if laser is blocked by obstacle1
		if ((angle_obs >= theta1 && angle_obs <= 360 - theta1) && (angle_l_o <= angle_obs + theta1 && angle_l_o >= angle_obs - theta1)){
			//cout << "blocked by"<< i << endl;
			//cout << "angleT " << angle_l_o << "angle_obs " << angle_obs << "theta " << theta1 << endl;
			a = 1;
		}
		else if (angle_obs < theta1 && (angle_l_o <= angle_obs + theta1 || angle_l_o >= 360 + angle_obs - theta1)){
			//cout << "blocked by" << i << endl;
			//cout << "angleT " << angle_l_o << "angle_obs " << angle_obs << "theta " << theta1 << endl;
			b = 1;
		}
		else if (360 - angle_obs < theta1 && (angle_l_o >= angle_obs - theta1 || angle_l_o <= angle_obs + theta1 - 360)){
			//cout << "blocked by" << i << endl;
			//cout << "angleT " << angle_l_o << "angle_obs " << angle_obs << "theta " << theta1 << endl;
			c = 1;
		}
		/*else {
			//flag = 0;
			cout << "not blocked by"<< i << endl;
			cout << "angleT " << angle_l_o << "angle_obs " << angle_obs << "theta " << theta1 << endl;
		}*/

		if (a == 1 || b == 1 || c == 1){
			flag[i / 2] = 1;
		}
		else
			flag[i / 2] = 0;
	}
	
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
	//cout << "\n side " << turn << endl;
	return 0;
}

int go_perpendicular(static volatile int turn, int io, int jo, int il, int jl, double angle_robot, double angle_l_o, int &pw_l, int &pw_r){

	double v, angle_diff;
	//double E = 0; // in the 'roundround' test, E = 5 work the best so far	


	//angle2p(io, jo, il, jl, angle_l_o);
	angle_diff = angle_l_o - angle_robot;

	//distance2pts(iT, jT, iL, jL, distance);
	//v = distance / 8 * 5 + 50.0; // ratio of velocity, the farer, the faster; the closer, the slower ---- simple fuzzy logic
	v = 200;
	
	if (turn == 1){
		//q1
		if ((il > io) && (jl >= jo)){
			if (((angle_diff) > -90) && ((angle_diff) < 90)){
				// turn anticlock
				pw_l = 1700;
				pw_r = 1700;
				if ((angle_diff < -50.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;
				if ((angle_diff > -60.00)&&(angle_diff<-40.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}

		//q2
		if ((il < io) && (jl >= jo)){
			if (((angle_diff) > 90) && ((angle_diff) < 270)){
				//turn cloclwise
				pw_l = 1300;
				pw_r = 1300;
				if ((angle_diff > 305.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				pw_l = 1700;
				pw_r = 1700;
				if ((angle_diff < 325.00) && (angle_diff > 305.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}
		//q3
		if ((il < io) && (jl <= jo)){
			if (((angle_diff) < -90) && ((angle_diff) > -270)){
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;
				if ((angle_diff > -50)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				//turn anticlock
				pw_l = 1700;
				pw_r = 1700;
				if ((angle_diff < -40.00) && (angle_diff >-60.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}
		//q4
		if ((il > io) && (jl <= jo)){
			if (((angle_diff) < 90) && ((angle_diff) > -90)){
				//anticlock
				pw_l = 1700;
				pw_r = 1700; 
				//cout << "\tdifference in Q4S+1 = " << angle_diff;
				if ((angle_diff < -50.00)){
				
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;			
			//	cout << "\tdifference in Q4S+1 = " << angle_diff;
				if ((angle_diff < -40)&&(angle_diff>-60)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}
	}

	if (turn == -1){
		//q1
		if ((il > io) && (jl >= jo)){
			if (((angle_diff) > -90) && ((angle_diff) < 90)){
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;
				if ((angle_diff > 50.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				//anticlock
				pw_l = 1700;
				pw_r = 1700;
				if ((angle_diff > 40.00) && (angle_diff < 60.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}

		//q2
		if ((il < io) && (jl >= jo)){
			if (((angle_diff) > 90) && ((angle_diff) < 270)){
				//anticlock
				pw_l = 1700;
				pw_r = 1700;
				if ((angle_diff < 50)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;
				if ((angle_diff > 40) && (angle_diff < 60)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}
		//q3
		if ((il < io) && (jl <= jo)){
			if (((angle_diff) < -90) && ((angle_diff) > -270)){
				//turn anticlock
				pw_l = 1700;
				pw_r = 1700;
				if ((angle_diff < -305.00) && (angle_diff >-325.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
			else{
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;
				if ((angle_diff < -305.00) && (angle_diff >-325.00)){
					pw_l = 1300;
					pw_r = 1700;
				}
			}
		}
		//q4
		if ((il > io) && (jl <= jo)){
			if (((angle_diff) < 90) && ((angle_diff) > -90)){
				//turn clockwise
				pw_l = 1300;
				pw_r = 1300;
				//cout << "\tdifference in Q4S-1 = " << angle_diff;
				//cout << "\n turning cloclwise" << endl;
			    if ((angle_diff < 60)&&(angle_diff>40)){
					pw_l = 1300;
					pw_r = 1700;
				//	cout << "\n going straight" << endl;
				}
			}
			else{
				//turn anticlock
				pw_l = 1700;
				pw_r = 1700;
				//cout << "\tdifference in Q4S-1 = " << angle_diff;
				if ((angle_diff < 60) && (angle_diff>40)){
					pw_l = 1300;
					pw_r = 1700;
				//	cout << "\n going straight" << endl;

				}
			}
		}
	}
	return 0;
}

int avoid_obstacle(vector <double> obs_area, vector <double> obs_centroids, double angle_laser, double il, double jl, double io, double jo, double ig, double jg, double ir, double jr, double ileft, double iright, double jleft, double jright, int  &pw_l, int &pw_r, static volatile int &obstacle){
	int i;
	double icen, jcen, dist_l, dist_left, dist_right, r, area, angleg, angler, distgcen, distrcen, A, B, C, D;
	int obstaclecle[2];
	double angleob, angle_robot,collideangle;
	static volatile int avoided = 1;
	



	angle2p(ig, jg, ir, jr, angle_robot);

	for (i = 0; i < obs_centroids.size(); i = i + 2){
		icen = obs_centroids[i];
		jcen = obs_centroids[i + 1];
		area = obs_area[i / 2];
		r = sqrt(area / PI);
		//cout << "r = " << r << endl; // r= 30.1248
		distance(ileft, jleft, icen, jcen, dist_left);
		distance(iright, jright, icen, jcen, dist_right);
		distance(il, jl, icen, jcen, dist_l);
		distance(ig, jg, icen, jcen, distgcen);
		distance(ir, jr, icen, jcen, distrcen);
		angleg = asin(40 / distgcen);
		angler = asin(40 / distrcen);
		/*A = ((jr - jg) / (ir - ig));
		C = -(A)*(ig)+jg;
		D = fabs((A * icen + (-1) * jcen + C)) /
		(sqrt(A * A + 1));*/

		

		if ((distgcen <= r + 50)){
			cout << "distance between our car and obstacle : " << distgcen << endl;
			obstaclecle[i / 2] = 1;
			angle2p(ig, jg, icen, jcen, collideangle);
			avoided = 0;
		}
		else if (dist_l > r + 50) 	{
			obstaclecle[i / 2] = 0;
		}
	}

	cout << "obstacle1 = " << obstaclecle[0] << "  obstacle2 =  " << obstaclecle[1] << endl;


	//rotate(il, jl, angle_robot, pw_l, pw_r, obs_centroids[2], obs_centroids[3], collideangle, avoided);



	if (obstaclecle[0] == 0 && obstaclecle[1] == 0)	{
		cout << "not colliding, going straight" << endl;
		
	}
	else if (obstaclecle[0] == 1){
		rotate(il, jl, angle_robot, pw_l, pw_r, obs_centroids[0], obs_centroids[1],collideangle,avoided);
		cout << "avoidng obstacle 1 " << endl;
	}
	else if (obstaclecle[1] == 1){
		rotate(il, jl, angle_robot, pw_l, pw_r, obs_centroids[2], obs_centroids[3], collideangle,avoided);
		cout << "avoidng obstacle 2 " << endl;
	}



	return 0;
}

int rotate(double il, double jl, double angle_robot, int &pw_l, int &pw_r, double icen, double jcen,double collideangle, static volatile int &avoided){
	// there's the "shivering" caused by interlocking change of (iL, jL) and the target angle.
	// although I used (iL0, jL0) to kind of lock the point, sometimes shivering still happens.

	int step = 100;	// R = 85,step = 100;
	static volatile int n = 0, reached = 1, init = 1, iT, jT, iL0, jL0;
	double theta, theta0, turnangle, R = 90;

	turnangle = 60.0;
	if (init == 1){
		theta0 = collideangle;
		init = 0;
		//n = 0;
		cout << "collide angle = " << collideangle << endl;
	}	
	//theta0 = collideangle;
	step = turnangle / 3.6;

	if (n <= step){
		if (reached == 1){
			theta = theta0 + turnangle / (double)step * (double)(n + 1);
			if (theta > 360) theta = theta - 360;
			if (theta < 0) theta = theta + 360;
			iT = icen + R * cos(theta *PI / 180.0);
			jT = jcen + R * sin(theta *PI / 180.0);
			iL0 = il;
			jL0 = jl;
			cout << "next n: iT : " << iT << " jT : " << jT << " theta : " << theta << endl;
			reached = 0;
		}
		else{
			if (abs(iT - il) < 35 && abs(iT - il) >= 20 && abs(jT - jl) < 35 && abs(jT - jl) >= 20){ // this is to solve the "shivering" of the robot
				go_there(iL0, jL0, iT, jT, angle_robot, pw_l, pw_r);
				//cout << "iT : " << iT << " jT : " << jT << " iL : " << il << " jL : " << jl << endl;
				cout << "almost there" << endl;
			}
			else if (abs(iT - il) < 20 && abs(jT - jl) < 20) {
				reached = 1;
				n++;
				cout << "reached this n, the next n = :" << n << endl;
			}
			else{
				go_there(il, jl, iT, jT, angle_robot, pw_l, pw_r);
				cout << "reaching next n " << n << endl;
				cout << "iT : " << iT << " jT : " << jT << " iL : " << il << " jL : " << jl << endl;
			}
		}
	}
	else{
		cout << " angle reached! " << endl;
		pw_r = 1500;
		pw_l = 1500;
		avoided = 1;
		init = 1;
	}


	return 0;
}

int enemycontrol(image &rgb, int &pw_l_o, int &pw_r_o,double io,double jo,double ib,double jb){

	double angle_enemy_robot;
	double angle_range;
	int i, j, k;
	double ile, jle;
	

	

	//display the hunting range of the opponent
	angle2p(io, jo, ib, jb, angle_enemy_robot);
	ile = io + (31.0 * cos(angle_enemy_robot*(3.14159 / 180)));
	jle = jo + (31.0 * sin(angle_enemy_robot*(3.14159 / 180)));	

	angle_range = angle_enemy_robot + 90;
	if (angle_range > 360) angle_range = angle_range - 360;
	if (angle_range < 0) angle_range = angle_range + 360;

	/*for (i = -50; i < 50; i++){
		draw_point_rgb(rgb, ile + 10*i*cos(angle_range*(3.14159 / 180)), jle + 10*i * sin(angle_range*(3.14159 / 180)), 255, 255, 255);
	}*/

	// draw_point_rgb(rgb, ile, jle, 80, 150, 200);


	// simple manual control system for testing-ZYW
	if (KEY('W')){
		// car move forward
		pw_l_o = 1000;
		pw_r_o = 2000;
	}
	else if (KEY('A')){
		// rotate left
		pw_l_o = 2000;
		pw_r_o = 2000;
	}
	else if (KEY('D')){
		// rotate right
		pw_l_o = 1000;
		pw_r_o = 1000;
	}
	else if (KEY('S')){
		// move backward
		pw_l_o = 2000;
		pw_r_o = 1000;
	}
	else if (KEY('E')){
		// stop
		pw_l_o = 1500;
		pw_r_o = 1500;
	}
	

	//manual control above


	return 0;
}

int hide(image &rgb, double io, double jo, double ib, double jb,double icen,double jcen,double r,double ig,double jg,double angle_robot,int &pw_l,int &pw_r,double ir,double jr){
	
	double angle_enemy_robot,angle_hide;
	double ih, jh;
	double ile, jle;	

	angle2p(io, jo, ib, jb, angle_enemy_robot);
	ile = io + (31.0 * cos(angle_enemy_robot*(3.14159 / 180)));
	jle = jo + (31.0 * sin(angle_enemy_robot*(3.14159 / 180)));
	
	angle2p(icen, jcen, ile, jle, angle_hide);	

	ih = icen + (r + 130.0)*cos(angle_hide * PI / 180);
	jh = jcen + (r + 130.0)*sin(angle_hide * PI / 180);

	//draw_point_rgb(rgb, ih, jh, 255, 255, 255);

	//cout << "ih = " << ih << " jh = " << jh << " anglehide = " << angle_hide << endl;
	//cout << "ig = " << ig << " cosanglehide = " << cos(angle_hide * PI / 180) << endl;
	go_there_defense(icen,jcen,ig,jg, ih,jh, ir,jr, angle_robot, pw_l, pw_r);
	//go_there_defense(double icen, double jcen, double ig, double jg, double io, double jo,double ir,double jr, double angle_robot, int &pw_l, int &pw_r)
	//go_there(ig, jg, ih, jh, angle_robot, pw_l, pw_r);
	

	return 0;
}

int drawtrajectory(vector <int> &trajectory,image &rgb, double il, double jl){

	
	int sizetrajectory;
	int i;

	sizetrajectory = trajectory.size();
	trajectory.push_back(il);	
	trajectory.push_back(jl);


	/*for (i = 0; i < trajectory.size(); i = i + 2){
		draw_point_rgb(rgb, trajectory[i], trajectory[i+1], 255, 255, 255);
	}*/
	

	return 0;
}

int go_there(double ig, double jg, double io, double jo, double angle_robot, int &pw_l, int &pw_r){

	double angle, angle_laser;
	int v = 500;
	int E = 10;

	angle2p(io, jo, ig, jg, angle);

	angle_laser = angle - angle_robot;

	if ((ig > io) && (jg >= jo)){
		if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}


	if ((ig < io) && (jg >= jo)){
		if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if ((ig < io) && (jg <= jo)){
		if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if ((ig > io) && (jg <= jo)){
		if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if (fabs(ig - io) < 1 && fabs(jg - jo) < 1){
		pw_l = 1500;
		pw_r = 1500;
	}


	return 0;
}

int go_there_defense(double icen, double jcen, double ig, double jg, double io, double jo,double ir,double jr, double angle_robot, int &pw_l, int &pw_r){

	double angle, angle_laser;
	int v1 = 500,v2 = 500;
	int E = 10;

	angle2p(io, jo, ig, jg, angle); // angle from our robot to go-to point

	angle_laser = (angle - angle_robot); // angle of Car-Goto minus angle of our Car

	/*double d, d1, d2, d3;

	distance(ig, jg, icen, jcen, d1);
	d2 = 80.0;
	d3 = sqrt(pow(d1, 2) + pow(d2, 2));
	distance(ir, jr, icen, jcen, d);


	if ((ig > io) && (jg >= jo)){
		if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}


	if ((ig < io) && (jg >= jo)){
		if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if ((ig < io) && (jg <= jo)){
		if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if ((ig > io) && (jg <= jo)){
		if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
		else{
			pw_l = 1500 + v;
			pw_r = 1500 + v;
			if ((angle_laser > -E) && (angle_laser < E)){
				pw_l = 1500 - v;
				pw_r = 1500 + v;
			}
		}
	}

	if (d < d3-5){
		if (pw_l > 1500 && pw_r > 1500){
			pw_l = 1500 - v;
			pw_r = 1500 - v;
		}
		if (pw_l < 1500 && pw_r < 1500){
			pw_l = 1500 + v;
			pw_r = 1500 + v;
		}
	}*/
	
	
	
	if ((ig > io) && (jg >= jo)){
		cout << "go-to point as centroid, our robot in quadrant 1" << endl;
		if (icen < ig && jcen > jg){
			cout << "opponent on the northwest" << endl;
			if (((angle_laser) < 0) && ((angle_laser) > -90)){
				cout << "turning clock" << endl;
				pw_l = 1500 - v1; //clock
				pw_r = 1500 - v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v2; //anticlock
				pw_r = 1500 + v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else if (icen > ig && jcen < jg){
			cout << "opponent on the southeast" << endl;
			if (((angle_laser) > 0) && ((angle_laser) < 90)){
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v1; //anticlock
				pw_r = 1500 + v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning clock" << endl;
				pw_l = 1500 - v2; //clock
				pw_r = 1500 - v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else{
			go_there(ig, jg, io, jo, angle_robot, pw_l, pw_r);
			/*if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 2000;
			pw_r = 2000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}
			else{
			pw_l = 1000;
			pw_r = 1000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}*/
		}
	}

	if ((ig < io) && (jg <= jo)){
		cout << "go-to point as centroid, our robot in quadrant 3" << endl;
		if (icen < ig && jcen > jg){
			cout << "opponent on the northwest" << endl;
			if (((angle_laser) < 0) && ((angle_laser) > -270)){
				cout << "turning clock" << endl;
				pw_l = 1500 - v1; //clock
				pw_r = 1500 - v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v2; //anticlock
				pw_r = 1500 + v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else if ((icen > ig && jcen < jg)){
			cout << "opponent on the southeast" << endl;
			if (((angle_laser) < 0) && ((angle_laser) > -90)){
				cout << "turning clock" << endl;
				pw_l = 1500 - v2; //clock
				pw_r = 1500 - v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v1; //anticlock
				pw_r = 1500 + v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else{
			go_there(ig, jg, io, jo, angle_robot, pw_l, pw_r);
			/*if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1000;
			pw_r = 1000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}
			else{
			pw_l = 2000;
			pw_r = 2000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}*/
		}
	}



	if ((ig < io) && (jg >= jo)){
		cout << "go-to point as centroid, our robot in quadrant 2" << endl;
		if (icen > ig && jcen > jg){
			cout << "opponent on northeast" << endl;
			if (((angle_laser) > 0) && ((angle_laser) < 90)){
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v2; //anticlock
				pw_r = 1500 + v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning clock" << endl;
				pw_l = 1500 - v1; //clock
				pw_r = 1500 - v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else if ((icen < ig && jcen < jg)){
			cout << "opponent on southwest" << endl;
			if (((angle_laser) > 0) && ((angle_laser) < 270)){
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v1; //anticlock
				pw_r = 1500 + v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning clock" << endl;
				pw_l = 1500 - v2; //clock
				pw_r = 1500 - v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else{
			go_there(ig, jg, io, jo, angle_robot, pw_l, pw_r);
			/*if (((angle_laser) > 0) && ((angle_laser) < 180)){
			pw_l = 2000;
			pw_r = 2000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}
			else{
			pw_l = 1000;
			pw_r = 1000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}*/

		}
	}



	if ((ig > io) && (jg <= jo)){
		cout << "go-to point as centroid, our robot in quadrant 4" << endl;
		if (icen > ig && jcen > jg){
			cout << "opponent on northeast" << endl;
			if (((angle_laser) < 0) && ((angle_laser) > -90)){
				cout << "turning clock" << endl;
				pw_l = 1500 - v1; //clock
				pw_r = 1500 - v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v2; //anticlock
				pw_r = 1500 + v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else if ((icen < ig && jcen < jg)){
			cout << "opponent on southwest" << endl;
			if (((angle_laser) < 0) && ((angle_laser) > -270)){
				cout << "turning clock" << endl;
				pw_l = 1500 - v1; //clock
				pw_r = 1500 - v1;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
			else{
				cout << "turning anticlock" << endl;
				pw_l = 1500 + v2; //anticlock
				pw_r = 1500 + v2;
				if ((angle_laser > -E) && (angle_laser < E)){
					pw_l = 1000; pw_r = 2000;
					/*pw_l = 1500 - v;
					pw_r = 1500 + v;*/
				}
			}
		}
		else{
			go_there(ig, jg, io, jo, angle_robot, pw_l, pw_r);
			/*if (((angle_laser) < 0) && ((angle_laser) > -180)){
			pw_l = 1000;
			pw_r = 1000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}
			else{
			pw_l = 2000;
			pw_r = 2000;
			if ((angle_laser > -E) && (angle_laser < E)){
			pw_l = 1000;
			pw_r = 2000;
			}
			}*/
		}

	}
	
	if (fabs(ig - io) < 1 && fabs(jg - jo) < 1){
		pw_l = 1500;
		pw_r = 1500;
	}

	


	return 0;
}

int satellite(double ig, double jg, double angle_robot, double icen, double jcen, int &pw_l, int &pw_r, int &inorbit, double io, double jo){

	int i, j;
	static volatile int clock = 0; // 1 for clockwise -1 for anticlockwise
	double v = 80;
	double E = 2;
	// v = 0  --- r = 65
	// v = 50 --- r = 80
	// v = 100--- r = 95
	// l=1500,r=2000 R=65 anticlock forward
	// l=1000,r=1500 R=65 clockwise forward
	// l=2000,r=1500       anticlock backward

	//entering orbit
	if (inorbit == 0){
		if (fabs(ig - icen) < E){ // robot is above or below the obstacle
			if ((angle_robot >= 90 && angle_robot < 180 - E) || (angle_robot >= 270 && angle_robot <360 - E)){
				pw_l = 2000;
				pw_r = 2000;
			}
			else if ((angle_robot > E && angle_robot < 90) || (angle_robot > 180 + E && angle_robot < 270)){
				pw_l = 1000;
				pw_r = 1000;
			}
			else{
				if (jg > jcen && (angle_robot >180 - E && angle_robot < 180 + E)){
					cout << "in orbit! robot is above the obstacle, facing left, turn anticlock" << endl;
					pw_l = 1500 - v;
					pw_r = 2000;
					clock = -1;
				}
				else if (jg > jcen && (angle_robot < E || angle_robot > 360 - E)){
					cout << "in orbit! robot is above the obstacle, facing right, turn clock" << endl;
					pw_l = 1000;
					pw_r = 1500 + v;
					clock = 1;
				}
				else if (jg < jcen && (angle_robot >180 - E && angle_robot < 180 + E)){
					cout << "in orbit! robot is below the obstacle, facing left, turn clock" << endl;
					pw_l = 1000;
					pw_r = 1500 + v;
					clock = 1;
				}
				else if (jg < jcen && (angle_robot < E || angle_robot > 360 - E)){
					cout << "in orbit! robot is below the obstacle, facing right, turn anticlock" << endl;
					pw_l = 1500 - v;
					pw_r = 2000;
					clock = -1;
				}
				inorbit = 1;
			}
		}
		else if (fabs(jg - jcen) < E){ // robot is on the left or right of the obstacle
			if ((angle_robot > 90 + E && angle_robot <= 180) || (angle_robot > 270 + E && angle_robot <= 360)){
				pw_l = 1000;
				pw_r = 1000;
			}
			else if ((angle_robot > 0 && angle_robot < 90 - E) || (angle_robot > 180 && angle_robot < 270 - E)){
				pw_l = 2000;
				pw_r = 2000;
			}
			else{
				if (ig < icen && (angle_robot > 90 - E && angle_robot < 90 + E)){
					cout << "in orbit! robot is on the left of the obstacle, facing up, turn clock" << endl;
					pw_l = 1000;
					pw_r = 1500 + v;
					clock = 1;
				}
				else if (ig < icen && (angle_robot > 270 - E && angle_robot < 270 + E)){
					cout << "in orbit! robot is on the left of the obstacle, facing down, turn anticlock" << endl;
					pw_l = 1500 - v;
					pw_r = 2000;
					clock = -1;
				}
				else if (ig > icen && (angle_robot > 90 - E && angle_robot < 90 + E)){
					cout << "in orbit! robot is on the right of the obstacle, facing up, turn anticlock" << endl;
					pw_l = 1500 - v;
					pw_r = 2000;
					clock = -1;
				}
				else if (ig < icen && (angle_robot > 270 - E && angle_robot < 270 + E)){
					cout << "in orbit! robot is on the right of the obstacle, facing down, turn clock" << endl;
					pw_l = 1000;
					pw_r = 1500 + v;
					clock = 1;
				}

				inorbit = 1;
			}
		}
	}


	// orbiting
	double angle_obop, angle_roob, theta;

	if (inorbit == 1){
		cout << "orbiting and avoiding opponent..." << endl;
		angle2p(io, jo, icen, jcen, angle_obop);
		angle2p(icen, jcen, ig, jg, angle_roob);
		theta = angle_roob - angle_obop;

		if (clock == 1){ // turning clockwise
			if (angle_obop >= 0 && angle_obop < 180){ // opponent on the upper half of the obstacle
				if (theta > 0 && theta <= 180){ // should turn clock: forward
					pw_l = 1000;
					pw_r = 1500 + v;
				}
				else if (theta < 0 || theta > 180){ //should turn anticlock:backward
					pw_l = 1500 + v;
					pw_r = 1000;
				}
				else{
					pw_l = 1500;
					pw_r = 1500;
				}
			}
			else{ // opponent on the downer half of the obstacle
				if (theta < 0 && theta >= -180){ // should turn anticlock:backward
					pw_l = 1500 + v;
					pw_r = 1000;
				}
				else if (theta > 0 || theta <-180){
					// should turn clock: forward
					pw_l = 1000;
					pw_r = 1500 + v;
				}
				else{
					pw_l = 1500;
					pw_r = 1500;
				}
			}
		}

		if (clock == -1){	// turning anticlockwise
			if (angle_obop >= 0 && angle_obop < 180){ // opponent on the upper half of the obstacle
				if (theta > 0 && theta <= 180){ // should turn clock:backward
					pw_l = 2000;
					pw_r = 1500 - v;
				}
				else if (theta < 0 || theta > 180){  // should turn anticlock:forward
					pw_l = 1500 - v;
					pw_r = 2000;
				}
				else{
					pw_l = 1500;
					pw_r = 1500;
				}
			}
			else{	// opponent on the upper downer of the obstacle
				if (theta < 0 && theta >= -180){ // should turn anticlock:forward
					pw_l = 1500 - v;
					pw_r = 2000;
				}
				else if (theta > 0 || theta <-180){ // should turn clock:backward
					pw_l = 2000;
					pw_r = 1500 - v;
				}
				else{
					pw_l = 1500;
					pw_r = 1500;
				}
			}
		}
	}





	return 0;
}