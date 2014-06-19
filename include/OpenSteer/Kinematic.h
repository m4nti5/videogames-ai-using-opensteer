#ifndef KINEMATIC_H
#define KINEMATIC_H
	#include <float.h>
	#include <sstream>
	#include "Vec3.h"
	#include "Polygon.h"
	#include "Compare.h"
	#include "CustomPath.h"
	#include "SteeringOutput.h"
	#include "Draw.h"
	#include "OpenSteerDemo.h"
	
	namespace OpenSteer {
		// Handles the phisics information
		class Kinematic{
			public:
				// Maximum velocity for ALL vehicles
				float maxSpeed;
				// Maximum turn speed in radians/s
				float maxRotation;
				// Maximum possible acceleration in m/s^2
				float maxAcceleration;
				// Maximum possible angular acceleration in radians/s^2
				float maxAngularAcceleration;
				// vehicle position
				Vec3 position;
				// radian value of orientation, measured from the z coordinate counter-clockwise
				float orientation;
				// Speed vector for linear velocity
				Vec3 velocity;
				// Angular speed Value(radian per seconds, to avoid 3rd Newtons law violations)
				float rotation;
				// used for keyboard checks during updates, the value has a 1 if the key was pressed
				// every slot of the array maps to a, b, c, d, e,..., z in english character set
				char keymap[28];
				// Holds the behaviour name
				std::string seekerStateString;
				// Needed for wander behaviour
				float wanderOrientation;
				// Needed for Follow Path behaviour
				Vec3 lastPathPoint;
				// Saves the last polygon the agent was
				Polygon lastNode;
				// For FollowPath Steering
				CustomPath path;
				Vec3 lastCustomPathPoint;
		
				static const Kinematic zero;
		
				Kinematic(void): maxSpeed(2.0f), maxRotation(M_PI), maxAcceleration(13.0f), maxAngularAcceleration(20.0f), position(), orientation(0.0f), velocity(), rotation (0.0f), seekerStateString("") {}
		
				Kinematic(float s, float r, float a, float aa, Vec3 p, float o, Vec3 v, float ro, std::string str): maxSpeed(s), maxRotation(r), maxAcceleration(a), maxAngularAcceleration(aa), position(p), orientation(o), velocity(v), rotation (ro), seekerStateString(str) {}
		
				~Kinematic(){
			
				}

				void setPosition(const OpenSteer::Vec3& p){
					position = p;
				}

				void setSpeed(const OpenSteer::Vec3& v){
					velocity = v;
				}

				void update(const SteeringOutput& s,float time){
					position += velocity * time;
					orientation += rotation * time;
					// s brings the new acceleration so, it is used to update the velocity and rotation
					velocity += s.linear * time;
					rotation += s.angular * time;
					if(velocity.length() > maxSpeed){
						velocity = velocity.normalize();
						velocity *= maxSpeed;
					}
				}

				void setBehaviourName(std::string stateStr){
					this->seekerStateString = stateStr;
				}

				static double getNewOrientation(double currentOrientation, const OpenSteer::Vec3& direction){
					if(direction.length()> 0)
						return atan2(direction.x,direction.z);
					else
						return currentOrientation;
				}

				void printBehaviour(){
					// annote seeker with its state as text
					const OpenSteer::Vec3 textOrigin = position + OpenSteer::Vec3 (0, 0.25, 0);
					std::ostringstream annote;
					annote << seekerStateString << std::endl;
					OpenSteer::draw2dTextAt3dLocation (annote, textOrigin, OpenSteer::gWhite, OpenSteer::drawGetWindowWidth(), OpenSteer::drawGetWindowHeight());
				}

				Kinematic operator= (const Kinematic& k) {
					maxSpeed=k.maxSpeed, maxRotation=k.maxRotation, maxAcceleration=k.maxAcceleration, maxAngularAcceleration=k.maxAngularAcceleration, position=k.position, orientation=k.orientation, velocity=k.velocity, rotation=k.rotation, lastPathPoint = k.lastPathPoint, lastNode = k.lastNode;
					seekerStateString = k.seekerStateString;
					return *this;
				}

				bool operator== (const Kinematic& k) const {return Compare::nearlyEqual(maxSpeed, k.maxSpeed, FLT_EPSILON ) < 0.001f && Compare::nearlyEqual(maxRotation, k.maxRotation, FLT_EPSILON ) && Compare::nearlyEqual(maxAcceleration, k.maxAcceleration, FLT_EPSILON ) && Compare::nearlyEqual(maxAngularAcceleration, k.maxAngularAcceleration, FLT_EPSILON ) && position == k.position && Compare::nearlyEqual(orientation, k.orientation, FLT_EPSILON ) && velocity == k.velocity && Compare::nearlyEqual(rotation, k.rotation, FLT_EPSILON );}

				static float randomBinomial(){
					return (random01()) - (random01());
				}

				static void orientationAsAVector(float orientation, OpenSteer::Vec3& output){
					output.x = sin(orientation);
					output.z = cos(orientation);
				}

			private:
				static float random01(){
					double i = 0, d = 0;
					i = rand() % 4000 - 2000; // Gives a number between -2000 and +2000;
					d = i / 10000; // Reduces this number to the range you want.
					return d;
				}
		};
	inline std::ostream& operator<< (std::ostream& o, const Kinematic& k){
		return o << "position: " << k.position << std::endl << "orientation: " << k.orientation << std::endl << "velocity: " << k.velocity << std::endl << "rotation: " << k.rotation << std::endl;
	}
	}
	
	
#endif

