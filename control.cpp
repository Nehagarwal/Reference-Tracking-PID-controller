#include <algorithm>
#include <bitset>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>
constexpr double kPi = 3.1415926;
constexpr double kVelocity = 1;
constexpr double kWheelbase = 2.0;
struct CarPose {
  double x;
  double y;
  double theta;
};
/**
 * Returns the new pose of the car.
 *
 * Integrate the current pose forwards by timestep seconds when applying a given
 * steering angle to the car at a given velocity using the first order forward
 * Euler method.
 *
 * Kinematics model:
 * dot(x) = v0*cos(theta)
 * dot(y) = v0*sin(theta)
 * dot(theta) = v0*tan(u)/L
 *
 * where
 * x: car's x position
 * y: car's y position
 * theta: heading angle theta == 0,  car in x direction
 *                      theta == PI/2, car in y direction
 * L: wheelbase ==  2m,
 * v0: velocity == 1m/s.
 *
 * @param current_pose[in] The current car's pose
 * @param steering_angle[in] The applied steering angle to the car in radians
 * @param velocity[in] The car's velocity
 * @param timestep[in] The timestep to integrate by
 * @return Car's pose at timestep seconds
 */
CarPose Step(CarPose& current_pose, double steering_angle,
             double velocity, double timestep) {
  // Fill in this function
    CarPose nextstep;
    double dotx = velocity*cos(current_pose.theta);
    double doty = velocity*sin(current_pose.theta);
    double dottheta = (kVelocity*tan(steering_angle))/kWheelbase;
    double xnew = current_pose.x + timestep*dotx;
    double ynew = current_pose.y + timestep*doty;
    double thetanew = current_pose.theta + timestep*dottheta;
    nextstep.x = xnew;
    nextstep.y = ynew;
    nextstep.theta = thetanew;
    return nextstep;
}

double cte(double& x1, double& y1, double& x2, double& y2, double& pointX, double& pointY)
{
    double diffX = x2 - x1;
    float diffY = y2 - y1;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = pointX - x1;
        diffY = pointY - y1;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = pointX - x1;
        diffY = pointY - y1;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = pointX - x2;
        diffY = pointY - y2;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = pointX - (x1 + t * diffX);
        diffY = pointY - (y1 + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}
/**
 * Simulates the car tracking the reference path by setting the car's
 * steering_angles.
 *
 * Steering angles are limited to [-0.5, 0.5].
 *
 * @param reference[in] The reference path to track
 * @param initial_pose[in] The initial pose of the car
 * @param path[out] The path that the car drives
 * @param steering_angle[out] The steering angles applied to the car
 * @return Root mean squared error of trajectory
 steer=sum(cross([TestTrack.cline(:,tp+1);0]-[TestTrack.cline(:,tp);0],[z_temp(end,1);z_temp(end,3);0]-[TestTrack.cline(:,tp);0])./norm(cross([TestTrack.cline(:,tp+1);0]-[TestTrack.cline(:,tp);0],[z_temp(end,1);z_temp(end,3);0]-[TestTrack.cline(:,tp);0])));
*/
void Simulate(std::vector<CarPose>& reference,
                CarPose& initial_pose, std::vector<CarPose>* path,
                std::vector<double>* steering_angles) {
  // Fill in this function
  // Implement a controller to track the reference trajectory. You
  // can update the car's pose using the previous function.
  double cte1;
  double kp=0.5;
  double kd = 3;
  double ki = 0.004;
  double cte_prev=0;
  double diff_cte;
  double int_diff = 0;
  //int_diff = int_diff + cte1;
  int i;
  for(i=0;i<200;i++)
  {
      cte1 = cte(reference[i].x,reference[i].y,reference[i+1].x,reference[i+1].y,initial_pose.x,initial_pose.y)l;
      diff_cte = cte1 - cte_prev;
      int_diff = int_diff + cte1;
      steering_angles->push_back(-kp*cte1 -kd*diff_cte -ki*int_diff);
      path->push_back(Step(initial_pose,steering_angles->back(),1,0.02));
      
      initial_pose = path->back();
      }
      
}
int main() {
  constexpr int kTimesteps = 200;
  constexpr double kDt = 0.02;
  constexpr double kSteeringAngle = 0.2;
  CarPose initial_pose = {0, 0, kPi / 2.0};
  std::vector<CarPose> reference_path(kTimesteps);
  reference_path[0] = CarPose{0.2, 0.2, kPi / 2.0};
  for (int i = 1; i < kTimesteps; ++i) {
    CarPose new_pose =
        Step(reference_path[i-1], kSteeringAngle, kVelocity, kDt);
    reference_path[i] = new_pose;
  }
  std::vector<double> u;
  std::vector<CarPose> path;
  //std::cout << "RMSE: " << 
  Simulate(reference_path, initial_pose, &path, &u);
//            << std::endl;
  return 0;
}


