#include <iostream>

using namespace std;

class State
{
	double x;
	double y;
	double deltaX;
	double deltaY;
public:
	State(double x, double y, double deltaX, double deltaY) : x(x), y(y), deltaX(deltaX), deltaY(deltaY) {}
	double getX() { return x; }
	double getY() { return y; }
	double getSpeedX() { return deltaX; }
	double getSpeedY() { return deltaY; }
	void setSpeedX(double s) { deltaX = s; }
	void setSpeedY(double s) { deltaY = s; }
};