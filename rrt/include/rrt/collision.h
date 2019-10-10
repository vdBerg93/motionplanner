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
        float w;           // Width
        float h;           // Height
        float o;           // Orientation
        
        OBB(Vector2D _pos, float _w, float _h, float _o): pos(_pos), w(_w), h(_h), o(_o){
            setVertices();
            setNorms();
        }
        void findMaxMin(float x,float y);
        float normsX[4], normsY[4];         // Normal axis
        float verticesX[4], verticesY[4];   // Vertices
        float maxMin[2];
    private:
        void setVertices();
        void setNorms();
};

// Primitives
bool intersects(OBB a, OBB b);
bool checkCollision(ros::Publisher* ptrPub,StateArray T, const vision_msgs::Detection2DArray& det);
vector<OBB> getOBBvector(ros::Publisher* ptrPub, const vision_msgs::Detection2DArray& det);
void drawObstacles(ros::Publisher* ptrPub,vector<OBB> obstacleVector);

#endif