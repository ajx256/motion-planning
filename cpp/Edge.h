#include <iostream>
#include "Vertex.h"

using namespace std;

class Edge
{
	Vertex* start;
	Vertex* end;
	// Control.first is angle, Control.second is speed
	pair<double, double> appliedControl;

public:
	Edge(Vertex* s, Vertex* e, pair<double, double> control);
	Vertex* getStart() { return start; }
	Vertex* getEnd() { return end; }
	pair<double, double> getControl() { return appliedControl; }
};

Edge::Edge(Vertex* s, Vertex* e, pair<double, double> control)
{
	start = s;
	end = e;
	appliedControl = control;
}