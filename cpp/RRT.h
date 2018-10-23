#include <iostream>
#include <random>
#include <limits>
#include "Tree.h"

using namespace std;

# define M_PI 3.14159265358979323846

class RRT
{
	Tree* T;

public:
	RRT() { T = new Tree; }
	~RRT() { delete T; }
	Tree* getMotionTree() { return T; }
	Edge* search(pair<double, double> start, pair<double, double> goal, bool(*collision)(Vertex*, Vertex*), int, int);
	Edge* kinoSearch(pair<double, double> start, pair<double, double> goal, Edge*(*collision)(Vertex*, pair<double, double>, vector<pair<double, double> >), int, int);
	float sample(double min, double max);
};

float RRT::sample(double min, double max)
{
	random_device seed;
	mt19937 generator(seed());
	uniform_real_distribution<double> distro(min, max);
	return distro(generator);
}

Edge* RRT::search(pair<double, double> start, pair<double, double> goal, bool(*collision)(Vertex*, Vertex*), int maxX, int maxY)
{
	// Initialize the tree with the start vertex
	Vertex* startV = new Vertex(start.first, start.second, 0, 0, NULL, pair<double, double>(0,0));
	T->addVertex(startV);

	// Sample until you reach the goal
	while (1)
	{
		// Sample a random point, with 0.05 goal bias...
		pair<double, double> point;
		double sampleGoal = sample(0, 1);
		if (sampleGoal > 0.05)
			point = pair<double, double>(sample(0, maxX), sample(0, maxY));
		else
			point = goal;
		
		Vertex* nearest;
		double nearestDist = numeric_limits<double>::infinity();
		// Find the nearest state in the tree to the sample point
		for (Vertex* v : T->getVerticies())
		{
			// Get distance from v to point
			double distance = sqrt(pow((point.first - v->getState()->getX()),2) + pow((point.second - v->getState()->getY()),2));

			if (distance < nearestDist)
			{
				nearestDist = distance;
				nearest = v;
			}
		}

		// Check for collision
		Vertex* end = new Vertex(point.first, point.second, 0, 0, nearest, pair<double, double>(0, 0));

		if (collision(nearest, end))
		{
			// There was a collision, abandon this new point
			delete end;
		}
		else
		{
			// No collision, extend from this point to sample
			T->addVertex(end);
			Edge* extend = new Edge(nearest, end, pair<double, double>(0, 0));
			T->addEdge(extend);

			// Check if this newest sample is within 0.1 of the goal
			double distance = sqrt(pow((goal.first - end->getState()->getX()), 2) + pow((goal.second - end->getState()->getY()), 2));
			// If distance from sample to goal is less than radius of circle around goal, sample is in radius
			if (distance <= 0.1)
				return extend;
		}
	}
}

Edge* RRT::kinoSearch(pair<double, double> start, pair<double, double> goal, Edge*(*collision)(Vertex*, pair<double, double>, vector<pair<double, double> >), int maxX, int maxY)
{
	// Initialize the tree with the start vertex
	Vertex* startV = new Vertex(start.first, start.second, 0, 0, NULL, pair<double, double>(0, 0));
	T->addVertex(startV);

	// Sample until you reach the goal
	while (1)
	{
		// Sample a random point, with 0.05 goal bias...
		pair<double, double> point;
		double sampleGoal = sample(0, 1);
		if (sampleGoal > 0.05)
			point = pair<double, double>(sample(0, maxX), sample(0, maxY));
		else
			point = goal;

		Vertex* nearest;
		double nearestDist = numeric_limits<double>::infinity();
		// Find the nearest state in the tree to the sample point
		for (Vertex* v : T->getVerticies())
		{
			// Get distance from v to point
			double distance = sqrt(pow((point.first - v->getState()->getX()), 2) + pow((point.second - v->getState()->getY()), 2));

			if (distance < nearestDist)
			{
				nearestDist = distance;
				nearest = v;
			}
		}
		
		vector<pair<double, double> > controls;

		// Generate ten random controls
		for (int i = 0; i < 10; i++)
		{
			double angle = sample(0, (2 * M_PI));
			double angularAcc = sample(0, 0.5);
			controls.push_back(pair<double, double>(angle, angularAcc));
		}

		// Check for collisions among the ten sampled trajectories
		Edge* trajectory = collision(nearest, point, controls);
		if (trajectory != NULL)
		{
			// Add the edge to the tree
			T->addEdge(trajectory);
			T->addVertex(trajectory->getEnd());

			// Check if the ship is within the goal
			double distance = sqrt(pow((goal.first - trajectory->getEnd()->getState()->getX()), 2) + pow((goal.second - trajectory->getEnd()->getState()->getY()), 2));
			// If distance from sample to goal is less than radius of circle around goal, sample is in radius
			if (distance <= 0.1)
				return trajectory;
		}
	}
}