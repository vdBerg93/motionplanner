#ifndef SIMULATION_H
#define SIMULATION_H

#include "rrt/datatypes.h"
#include "rrt/controller.h"

class Simulation{
	private:
		void propagate(const MyRRT& RRT, Controller control, const MyReference& ref, const Vehicle& veh);
	public:
		StateArray stateArray;
		vector<double> curvature;
		vector<int> closestPoints;
		double costS, costE;

		bool goalReached, endReached;
		Simulation(const MyRRT& RRT, Node& node, MyReference& ref, const Vehicle& veh);
		bool isvalid();
};
bool Simulation::isvalid()
{
	return endReached;
}

#endif