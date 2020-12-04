//reference url: http://www.doc88.com/p-738493052458.html
//https://blog.csdn.net/junshen1314/article/details/50472410
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <ctime>
using namespace std;

double *generate_obs(int num);
double sum(double *arr, int num); //arr is an array, num = size of array
class APF
{
public:
	double att_k;	  //attraction constant
	double rep_k;	  //repulsion constant
	double ob_dist;	  //certain obstacle distance; [constant param]
	double goal_dist; //beyond this distance attraction is a constant
	double step;
	double *cp; //current position; [x, y], this is array
	double *goal;
	double *obs;
	//constructor
	void APF_init(double att_k, double rep_k, double ob_dist, double goal_dist, double step, double *cp, double *goal, double *obs)
	{
		this->att_k = att_k;
		this->rep_k = rep_k;
		this->ob_dist = ob_dist;
		this->goal_dist = goal_dist;
		this->step = step;
		this->cp = cp; //cp is array {x,y}, pass the address of first element
		this->goal = goal;
		this->obs = obs;
	}

	//another way to define constructor

	/*void APF_init(double att_k, double rep_k, double ob_dist, double step, double *cp, double *goal, double *obs) : double att_k(att_k), double rep_k(rep_k), double ob_dist(ob_dist), double goal_dist(goal_dist), double step(step), double cp(cp), double goal(goal), double obs(obs){};*/

	double *compute_angle(const double *cp, int num); // num = obstacle numbers
	double *attraction(const double *cp, double *attraction_angle);
	double *repulsion(const double *cp, double *repulsion_angle, int num);
};

double *APF::compute_angle(const double *cp, int num)
{
	int sign;
	double dx, dy, r;
	double *angle = new double[num + 1];
	//angle of cp-goal
	dx = this->goal[0] - cp[0];
	dy = this->goal[1] - cp[1];
	r = sqrt(dx * dx + dy * dy);
	sign = dy < 0 ? -1 : 1; //enumerate all possible cases, when dy<0 the angle is negative, otherwise positive
	angle[0] = sign * acos(dx / r);
	//angle of cp-obstacle
	for (int i = 1; i < num + 1; i++)
	{
		dx = cp[0] - this->obs[2 * (i - 1)]; //repulsive force point from obs to cp, thus vector = cp - obstacles
		dy = cp[1] - this->obs[2 * (i - 1) + 1];
		r = sqrt(dx * dx + dy * dy);
		sign = dy < 0 ? -1 : 1;
		angle[i] = sign * acos(dx / r);
	}
	return angle;
}

double *APF::attraction(const double *cp, double *attraction_angle)
{
	double *Fatt = new double[2]; //static double Fatt[2]
	double r = sqrt(pow(this->goal[0] - cp[0], 2) + pow(this->goal[1] - cp[1], 2));
	if (r < goal_dist)
	{
		Fatt[0] = this->att_k * r * cos(attraction_angle[0]);
		Fatt[1] = this->att_k * r * sin(attraction_angle[0]);
	}
	else
	{
		Fatt[0] = this->att_k * this->goal_dist * cos(attraction_angle[0]);
		Fatt[1] = this->att_k * this->goal_dist * sin(attraction_angle[0]);
	}
	return Fatt;
}

double *APF::repulsion(const double *cp, double *repulsion_angle, int num)
{
	double *Frep = new double[2];
	//frx = f_repulsive_in_x-axis; fax = f_attractive_force_effect_by_goal_to_obstacles_in_x-axis
	//Frep[0] = frx + fry; Frep[1] = fax + fay;
	double *frx = new double[num], *fry = new double[num], *fax = new double[num], *fay = new double[num];
	double Rgoal = pow(this->goal[0] - cp[0], 2) + pow(this->goal[1] - cp[1], 2); //dx^2+dy^2 for cp to goal
	double rgoal = sqrt(Rgoal);
	for (int i = 0; i < num; i++)
	{
		double Robs = pow(cp[0] - this->obs[2 * i], 2) + pow(cp[1] - this->obs[2 * i + 1], 2); //[dx^2+dy^2] obs to goal
		double robs = sqrt(Robs);
		if (robs < this->ob_dist)
		{
			//http://www.doc88.com/p-738493052458.html, equation (3)(4), set n = 2
			Frep[0] = this->rep_k * (1 / robs - 1 / this->ob_dist) * Rgoal / Robs; //vector from nearest obs point to cp
			Frep[1] = this->rep_k * pow(1 / robs - 1 / this->ob_dist, 2) * rgoal;  //vector from cp point to goal
		}
		else
		{
			Frep[0] = 0;
			Frep[1] = 0;
		}
		frx[i] = Frep[0] * cos(repulsion_angle[i + 1]);
		fry[i] = Frep[0] * sin(repulsion_angle[i + 1]);
		fax[i] = Frep[1] * cos(repulsion_angle[0]);
		fay[i] = Frep[1] * sin(repulsion_angle[0]);
	}
	Frep[0] = sum(frx, num) + sum(fax, num);
	Frep[1] = sum(fry, num) + sum(fay, num);
	return Frep;
}

double sum(double *arr, int num)
{
	double sum_result = 0;
	for (int i = 0; i < num; i++)
	{
		sum_result += arr[i];
	}
	return sum_result;
}

double *generate_obs(int num)
{
	double max = 10; //map boundary, this is limitation for coordinates
	double *obstacles = new double[num];
	srand(time(NULL));
	for (int i = 0; i < num; i++)
	{
		obstacles[i] = rand() / double(RAND_MAX) * max;
	}
	return obstacles;
}

int main()
{
	cout << "APF start..." << endl;
	APF APF_demo;
	int obs_num = 10;		   //obstacle number
	const int max_cycle = 500; //iteration limit
	double att_k = 30;		   //attraction constant
	double rep_k = 10;		   //repulsion constant
	double ob_dist = 4;		   //certain obstacle distance; [constant param]
	double goal_dist = 5;	   //beyond this distance attraction is a constant
	double step = 0.2;
	double cp[2] = {0, 0}; //current position; [x, y], this is array
	double goal[2] = {10, 10};
	double *angle, *Fat, *Fre;				 //angle[]=angle-goal-obs, Fat[x,y]=attractive, Fre[x,y]=repulsive
	double *obs = generate_obs(2 * obs_num); //1 obstacle with 2 double data [x, y]
	vector<vector<double>> path;
	//double path[max_cycle][2];
	APF_demo.APF_init(att_k, rep_k, ob_dist, goal_dist, step, cp, goal, obs);
	//print obstacles
	cout << "Below are obstacles:" << endl;
	for (int i = 0; i < obs_num; i++)
	{
		cout << "[" << obs[2 * i] << ", " << obs[2 * i + 1] << "]" << endl;
	}
	for (int i = 0; i < max_cycle; i++)
	{
		path.emplace_back();
		path[i].push_back(cp[0]);
		path[i].push_back(cp[1]);
		angle = APF_demo.compute_angle(cp, obs_num);
		Fat = APF_demo.attraction(cp, angle);
		Fre = APF_demo.repulsion(cp, angle, obs_num);
		double Fx = Fat[0] + Fre[0];		   //sum of force in x-axis
		double Fy = Fat[1] + Fre[1];		   //sum of force in y-axis
		double Fsum = sqrt(Fx * Fx + Fy * Fy); //total force
		cp[0] += APF_demo.step * Fx / Fsum;	   //update current position to the next point
		cp[1] += APF_demo.step * Fy / Fsum;
		if (fabs(cp[0] - goal[0]) < 0.1 && fabs(cp[1] - goal[1]) < 0.1) //reach the goal
		{
			path.emplace_back();
			path[i + 1].push_back(cp[0]);
			path[i + 1].push_back(cp[1]);
			path.emplace_back();
			path[i + 2].push_back(goal[0]);
			path[i + 2].push_back(goal[1]);
			//print path

			cout << "Below are robot path:" << endl;
			for (int j = 0; j < i + 3; j++)
			{
				cout << "[" << path[j][0] << ", " << path[j][1] << "]" << endl;
			}
			cout << "Robot arrived!" << endl;
			cout << "Obstacle number: " << obs_num << endl;
			cout << "Iteration time: " << i << endl;
			break;
		}
		else if (i == max_cycle - 1)
		{
			cout << "Max iteration reached, no path found." << endl;
		}
	}

	//system("pause");
}