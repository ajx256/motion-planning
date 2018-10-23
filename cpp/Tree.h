#include <iostream>
#include <vector>
#include "Edge.h"

using namespace std;

class Tree
{
	vector<Vertex*> verticies;
	vector<Edge*> edges;

public:
	~Tree();
	void addVertex(Vertex* v) { verticies.push_back(v); }
	void addEdge(Edge* e) { edges.push_back(e); }
	vector<Vertex*> getVerticies() { return verticies; }
	vector<Edge*> getEdges() { return edges; }
};

Tree::~Tree()
{
	for (Vertex* v : verticies)
		delete v;

	for (Edge* e : edges)
		delete e;
}