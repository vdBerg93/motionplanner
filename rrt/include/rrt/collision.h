#ifndef COLLISION_H
#define COLLISION_H

// Define 2D vector class
struct Vector2D{
    double x;
    double y;
    Vector2D(double _x, double _y): x(_x), y(_y){};
};

// Define Vector2D dot operation
template <class T>
double myDot(T a, T b){
    return (a.x*b.x + a.y*b.y);
}

class OBB{
    public:
        Vector2D pos;    // Center
        double w;           // Width
        double h;           // Height
        double o;           // Orientation
        vector<Vector2D> norms;     // Normal axis
        OBB(Vector2D _pos, double _w, double _h, double _o): pos(_pos), w(_w), h(_h), o(_o){
            setVertices();
            setNorms();
        }
        vector<double> findMaxMin(Vector2D axis);
        vector<Vector2D> vertices; // TL, TR, RR, RL
    private:
        void setVertices();
        void setNorms();
};

// Primitives
bool intersects(OBB a, OBB b);
bool checkCollision(ros::Publisher* ptrPub,StateArray T);
vector<OBB> getObstacleVector(ros::Publisher* ptrPub);
void drawObstacles(ros::Publisher* ptrPub,vector<OBB> obstacleVector);

#endif