#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <stack>
#include <cmath>
#include "RRT.h"

using namespace std;

// World
char** world;
int worldCols;
int worldRows;
bool kinoVehicle = false;

bool collision(Vertex* start, Vertex* end)
{
	// Get the slope of the two points, and break it into ten increments, checking for collisions at each point
	double xDiff = end->getState()->getX() - start->getState()->getX();
	double yDiff = end->getState()->getY() - start->getState()->getY();

	// Check if end is in collision
	if (world[(int)floor(worldRows - end->getState()->getY())][(int)floor(end->getState()->getX())] == '#')
		return true;

	// Get every point along the line that is 0.05 units from start
	
	// First find how many line segments of length 0.05 are between start and end
	double distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2));
	int segments = floor(distance / 0.005) - 1;

	// Based on how many segments are between start and end, caluclate step size in x and y
	double stepX = xDiff / segments;
	double stepY = yDiff / segments;

	// Get the next point on line from start
	double pointX = start->getState()->getX() + stepX;
	double pointY = start->getState()->getY() + stepY;

	for (int i = 0; i < segments; i++)
	{
		// Check if this new point is in collision
		if (world[(int)floor(worldRows - pointY)][(int)floor(pointX)] == '#')
		{
			return true;
		}

		// Move along the line by stepX and stepY
		pointX += stepX;
		pointY += stepY;
	}

	// No points along the line collide
	return false;
}

Edge* kinoCollision(Vertex* start, pair<double,double> target, vector<pair<double, double> > controls)
{
	// Check if target is in an obstacle
	if (world[(int)floor(worldRows - target.second)][(int)floor(target.first)] == '#')
		return NULL;

	// Just book keeping for the best simulation
	pair<double, double> bestLoc;
	pair<double, double> bestSpeed;
	pair<double, double> bestCntrl;
	double distanceToTarget = numeric_limits<double>::infinity();

	// Simulate all the controls!
	for (int i = 0; i < controls.size(); i++)
	{
		// Get start location and speed
		double theta = controls[i].first;
		double s = controls[i].second;
		double x = start->getState()->getX();
		double y = start->getState()->getY();
		double dx = start->getState()->getSpeedX();
		double dy = start->getState()->getSpeedY();
		bool collision = false;

		// Simulate over 1/20 of a timestep
		for (double t = 1; t <= 20; t++)
		{
			double dt = 1.0 / 20.0;
			double thetaTot = atan2(dy, dx) + theta;
			dx = dx + (dt * s * cos(thetaTot));
			dy = dy + (dt * s * sin(thetaTot));
			x = x + (dt * dx);
			y = y + (dt * dy);

			// Check for collision or going out of bounds
			if (y >= worldRows || x >= worldCols || y < 0 || x < 0 || world[(int)floor(worldRows - y)][(int)floor(x)] == '#')
			{
				collision = true;
				break;
			}
		}

		// If no collision along the trajectory, then check if it is best so far
		if (!collision)
		{
			double d = sqrt(pow((target.first - x), 2) + pow((target.second - y), 2));

			if (d < distanceToTarget)
			{
				// This is the best simulated trajectory so far, save it
				distanceToTarget = d;
				bestLoc = pair<float, float>(x, y);
				bestSpeed = pair<float, float>(dx, dy);
				bestCntrl = pair<float, float>(theta, s);
			}
		}
	}

	// If distance to target is still inf, no trajectories were valid
	if (distanceToTarget == numeric_limits<double>::infinity())
	{
		return NULL;
	}
	else
	{
		Vertex* best = new Vertex(bestLoc.first, bestLoc.second, bestSpeed.first, bestSpeed.second, start, bestCntrl);
		Edge* trajectory = new Edge(start, best, bestCntrl);
		return trajectory;
	}
}

int main(int argc, char* argv[])
{
	// Check if using the kinodynamic vehicle
	if (argc == 2)
	{
		string flag = argv[1];
		if (flag == "-kino")
			kinoVehicle = true;
	}

	// starting info for start state
	double startX = 0;
	double startY = 0;
	double goalX = 0;
	double goalY = 0;

	// Read in the grid dimensions from stdin
	cin >> worldCols;
	cin >> worldRows;

	// Read in the grid cells from stdin
	world = new char*[worldRows];
	for (int i = 0; i < worldRows; i++)
		world[i] = new char[worldCols];

	string worldInfo = "";
	int row = 0;

	for (int i = 0; i < worldRows; i++)
	{
		for (int j = 0; j < worldCols; j++)
		{
			char worldInfo = ' ';
			cin >> worldInfo;
			world[i][j] = worldInfo;
		}
	}

	// Read in start and goal locations
	cin >> startX;
	cin >> startY;
	cin >> goalX;
	cin >> goalY;

	pair<double, double> start = pair<double, double>(startX, startY);
	pair<double, double> goal = pair<double, double>(goalX, goalY);
	bool(*collisionCheck)(Vertex*, Vertex*) = collision;
	Edge*(*kinoCollisionCheck)(Vertex*, pair<double, double>, vector<pair<double, double> >) = kinoCollision;

	RRT motionPlanner;
	Edge* solution;
	
	if (kinoVehicle)
	{
		solution = motionPlanner.kinoSearch(start, goal, kinoCollisionCheck, worldCols, worldRows);
		
		// Walk backwards from the goal node to the start node
		stack<pair<State*, pair<double,double> > > trajectory;
		Vertex* node = solution->getEnd();
		while (node->getParent() != NULL)
		{
			trajectory.push(pair<State*, pair<double,double> >(node->getParent()->getState(), node->getControl()));
			node = node->getParent();
		}

		// Print the trajectory
		cout << trajectory.size() << endl;

		while (!trajectory.empty())
		{
			pair<State*, pair<double,double> > top = trajectory.top();
			cout << top.first->getX() << " " << top.first->getY() << " " << top.first->getSpeedX() << " " << 
				top.first->getSpeedY() << " " << top.second.first << " " << top.second.second << endl;
			trajectory.pop();
		}

		// Print the motion tree
		cout << motionPlanner.getMotionTree()->getEdges().size() << endl;

		for (Edge* e : motionPlanner.getMotionTree()->getEdges())
		{
			cout << e->getStart()->getState()->getX() << " " << e->getStart()->getState()->getY() << 
				" " << e->getStart()->getState()->getSpeedX() << " " << e->getStart()->getState()->getSpeedY() << " " << e->getControl().first << " " << e->getControl().second << endl;;
		}
	}
	else
	{
		solution = motionPlanner.search(start, goal, collisionCheck, worldCols, worldRows);
		// Walk backwards from the goal node to the start node
		stack<Vertex*> trajectory;
		Vertex* node = solution->getEnd();
		while (node != NULL)
		{
			trajectory.push(node);
			node = node->getParent();
		}

		// Print the trajectory
		cout << trajectory.size() << endl;

		while (!trajectory.empty())
		{
			Vertex* top = trajectory.top();
			cout << top->getState()->getX() << " " << top->getState()->getY() << endl;
			trajectory.pop();
		}

		// Print the motion tree
		cout << motionPlanner.getMotionTree()->getEdges().size() << endl;

		for (Edge* e : motionPlanner.getMotionTree()->getEdges())
		{
			cout << e->getStart()->getState()->getX() << " " << e->getStart()->getState()->getY() << " " << e->getEnd()->getState()->getX() << " " << e->getEnd()->getState()->getY() << endl;
		}
	}

	// delete the grid
	for (int i = 0; i < worldRows; i++)
	{
		delete[] world[i];
	}
	delete[] world;
}