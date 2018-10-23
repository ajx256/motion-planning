#include <iostream>
#include "State.h"

using namespace std;

class Vertex
{
	State* state;
	Vertex* parent = NULL;
	pair<double, double> controlToGen;

public:
	Vertex(double x, double y, double speedX, double speedY, Vertex* p, pair<double, double> control);
	~Vertex();
	State* getState() { return state; }
	void setParent(Vertex* p) { parent = p; }
	Vertex* getParent() { return parent; }
	pair<double, double> getControl() { return controlToGen; }
};

Vertex::Vertex(double x, double y, double speedX, double speedY, Vertex* p, pair<double,double> control)
{
	state = new State(x, y, speedX, speedY);
	parent = p;
	controlToGen = control;
}

Vertex::~Vertex()
{
	delete state;
}