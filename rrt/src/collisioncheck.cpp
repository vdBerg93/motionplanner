/* -------------------------------------------
    COLLISION DETECTION FUNCTIONS
--------------------------------------------*/
#include "rrt/collision.h"

// Function primitives


vector<OBB> getObstacleVector(ros::Publisher* ptrPub){
    //(ptrSrv)->call(1);
    
    vector<OBB> obstacleVector;
    OBB obs1(Vector2D(125,0),2,5,0);
    OBB obs2(Vector2D(40,0),2,5,0);
    //obstacleVector.push_back(obs1);
    obstacleVector.push_back(obs2);
    if(draw_obs){    drawObstacles(ptrPub,obstacleVector);}
    return obstacleVector;
}

void drawObstacles(ros::Publisher* ptrPub,vector<OBB> obstacleVector){
    static visualization_msgs::Marker msg;
    for(int index = 0; index<obstacleVector.size(); index++){
        // Initialize marker message
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.ns = "obstacles";
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.orientation.w = 1.0;
        msg.id = index;
        msg.type = visualization_msgs::Marker::LINE_LIST;
        msg.scale.x = 0.1;	// msg/LINE_LIST markers use only the x component of scale, for the line width

        // Line strip is red
        msg.color.r = 1.0;
        msg.color.a = 1.0;
        msg.lifetime = ros::Duration();
        
        geometry_msgs::Point p;// int i = 0;
        p.x = obstacleVector[index].vertices.back().x;
        p.y = obstacleVector[index].vertices.back().y;
        p.z = 0;
        msg.points.push_back(p);
        for(int i = 0; i<obstacleVector[index].vertices.size(); i++){
            p.x = obstacleVector[index].vertices[i].x;
            p.y = obstacleVector[index].vertices[i].y;
            p.z = 0;
            msg.points.push_back(p);
            msg.points.push_back(p);
        }
        msg.points.erase(msg.points.end());
        //msg.points.insert(msg.points.begin(); obstacleVector[index].vertices.back());
        ptrPub->publish(msg);
    }
}


bool checkCollision(ros::Publisher* ptrPub,StateArray T){
    //return false; // Override collision check
    // Quickly check if trajectory exceeds the domain
    ROS_WARN_ONCE("TODO: In collision, implement domain check!");
    // for(int index = 0; index!=T.size(); index++){
    //     double xmax{25}, xmin{-25}, ymax{25}, ymin{-25};
    //     if(T[index][0]>xmax | T[index][0]<xmin | T[index][1]>ymax | T[index][1]<ymin){
    //         return true;
    //     }
    // }
    // Check if the trajectory2d obb separating axis theorem2d obb separating axis theorem collides with obstacles
    vector<OBB> obstacleVector = getObstacleVector(ptrPub);
    for(int index = 0; index !=T.size(); index++){
        Vector2D vPos(T[index][0]+1.424*cos(T[index][2]),T[index][1]+1.424*sin(T[index][2]));
        OBB vOBB(vPos,2,4.848,T[index][2]); // Create vehicle OBB
        for(int j = 0; j !=obstacleVector.size(); j++){
            if (intersects(vOBB,obstacleVector[j])){
                return true;
            }
        }
    }
    // No collisions
    return false;
}



void OBB::setVertices(){
    vertices.push_back(Vector2D(pos.x+cos(o)*(h/2)-sin(o)*(w/2),pos.y+sin(o)*(h/2)+cos(o)*(w/2)));      // FL
    vertices.push_back(Vector2D(pos.x+cos(o)*(h/2)-sin(o)*(-w/2),pos.y+sin(o)*(h/2)+cos(o)*(-w/2)));    // FR
    vertices.push_back(Vector2D(pos.x+cos(o)*(-h/2)-sin(o)*(-w/2),pos.y+sin(o)*(-h/2)+cos(o)*(-w/2)));  // RR
    vertices.push_back(Vector2D(pos.x+cos(o)*(-h/2)-sin(o)*(w/2),pos.y+sin(o)*(-h/2)+cos(o)*(w/2)));    // RL
    assert(vertices.size()==4);
}

void OBB::setNorms(){
    int i = 0;
    while (i<(vertices.size()-1)){
        norms.push_back(Vector2D(vertices[i+1].y-vertices[i].y,-(vertices[i+1].x-vertices[i].x)));
        i++;
    }
    norms.push_back(Vector2D(vertices[0].y-vertices[i].y,-(vertices[0].x-vertices[i].x)));
    assert(norms.size()==vertices.size());
}

vector<double> OBB::findMaxMin(Vector2D axis){
    //cout<<"start max min calculation..."<<endl;
    vector<double> maxMin; // [0] is max, [1] is cosmin
    // First vertice as baseline
    maxMin.push_back(myDot(vertices[0], axis));
    maxMin.push_back(maxMin[0]);
    //maxMin[1] = maxMin[0];
    // Iterate through the remaining
    for(int i = 1; i < vertices.size(); i++){
        double proj = myDot(vertices[i], axis);
        // if new maximum
        if (proj > maxMin[0]){
            maxMin[0] = proj;
        }
        // if new minimum
        else if (proj < maxMin[1]){
            maxMin[1] = proj;
        }
    }
    return maxMin;
}

// Collison checking two rectangles OBB using the Separating Axis Theorem
bool intersects(OBB a, OBB b){
    // http://blog.marcher.co/sat1/
    // https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169
    //SWRI_PROFILE("intersects");
    // circle collision optimization
    Vector2D d(a.pos.x-b.pos.x, a.pos.y-b.pos.y); // change this operation
    float distSq = myDot(d,d);
    float r = max(a.w, a.h) + max(b.w, b.h);
    if (distSq > r*r){
        //cout<<"passed circle test! \n \n"<<endl;
        return false;
    }
    // SAT
   // Combine vectors not real function
    vector<Vector2D> axes = a.norms;
    for(int i = 0; i<b.norms.size(); i++){
        axes.push_back(b.norms[i]);
    }
    for(int i =0; i < axes.size(); i++){
        Vector2D axis = axes[i];
        vector<double> aProj = a.findMaxMin(axis);

        vector<double> bProj = b.findMaxMin(axis);

        // Check is separating axis condition is true
        if(aProj[0] < bProj[1] || bProj[0] < aProj[1]){
            return false;
        }

    }
    return true;
}