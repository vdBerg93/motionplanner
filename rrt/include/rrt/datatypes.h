#ifndef DATATYPES_H
#define DATATYPES_H

struct point2D{
    double x,y;
    point2D(double _x, double _y){
        x = _x; y = _y;
    }
};

typedef vector<double> state_type;
typedef vector<vector<double>> StateArray;

#endif
