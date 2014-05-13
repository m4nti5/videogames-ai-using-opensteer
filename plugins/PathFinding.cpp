#include <stdio.h>
#include <cmath> 
#include <iomanip>
#include <string>
#include <GL/gl.h>
#include <GL/glu.h>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <iterator>
#include <limits.h>
#include <limits>
#include <float.h>
#include <algorithm>
#include "OpenSteer/Annotation.h"
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Color.h"


namespace{
    using namespace OpenSteer;
    
    // For float comparision
    class Compare{
		public:
		
			static bool nearlyEqual(float a, float b, float epsilon) {
				float absA = abs(a);
				float absB = abs(b);
				float diff = abs(a - b);

				if (a == b) { // shortcut, handles infinities
					return true;
				} else if (a == 0 || b == 0 || diff < FLT_MIN) {
					// a or b is zero or both are extremely close to it
					// relative error is less meaningful here
					return diff < (epsilon * FLT_MIN);
				} else { // use relative error
					return diff / (absA + absB) < FLT_EPSILON;
				}
			}
	};
	
    // Update for proyectiles!
        
    const Vec3 GRAVITY (0, -9.81, 0);
	class SteeringOutput{
		public:
			Vec3 linear;
			float angular;
		SteeringOutput(void): linear(),angular(0.0f) {}
		SteeringOutput(const Vec3& v, float a): linear(v),angular(a) {}
	};

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
			
			Kinematic(void): maxSpeed(4.0f), maxRotation(M_PI), maxAcceleration(10.0f), maxAngularAcceleration(20.0f), position(), orientation(0.0f), velocity(), rotation (0.0f), seekerStateString("") {}
			
			Kinematic(float s, float r, float a, float aa, Vec3 p, float o, Vec3 v, float ro, std::string str): maxSpeed(s), maxRotation(r), maxAcceleration(a), maxAngularAcceleration(aa), position(p), orientation(o), velocity(v), rotation (ro), seekerStateString(str) {}
			
			~Kinematic(){
				
			}

			void setPosition(const Vec3& p){
				position = p;
			}
			
			void setSpeed(const Vec3& v){
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
			
			static double getNewOrientation(double currentOrientation, const Vec3& direction){
				if(direction.length()> 0)
					return atan2(direction.x,direction.z);
				else
					return currentOrientation;
			}
			
			void printBehaviour(){
				// annote seeker with its state as text
				const Vec3 textOrigin = position + Vec3 (0, 0.25, 0);
				std::ostringstream annote;
				annote << seekerStateString << std::endl;
				draw2dTextAt3dLocation (annote, textOrigin, gWhite, drawGetWindowWidth(), drawGetWindowHeight());
			}
			
	        Kinematic operator= (const Kinematic& k) {
	        	maxSpeed=k.maxSpeed, maxRotation=k.maxRotation, maxAcceleration=k.maxAcceleration, maxAngularAcceleration=k.maxAngularAcceleration, position=k.position, orientation=k.orientation, velocity=k.velocity, rotation=k.rotation;
	        	seekerStateString = k.seekerStateString;
	        	return *this;
	        }
	        
			bool operator== (const Kinematic& k) const {return Compare::nearlyEqual(maxSpeed, k.maxSpeed, FLT_EPSILON ) < 0.001f && Compare::nearlyEqual(maxRotation, k.maxRotation, FLT_EPSILON ) && Compare::nearlyEqual(maxAcceleration, k.maxAcceleration, FLT_EPSILON ) && Compare::nearlyEqual(maxAngularAcceleration, k.maxAngularAcceleration, FLT_EPSILON ) && position == k.position && Compare::nearlyEqual(orientation, k.orientation, FLT_EPSILON ) && velocity == k.velocity && Compare::nearlyEqual(rotation, k.rotation, FLT_EPSILON );}
	        
			static float randomBinomial(){
				return (random01()) - (random01());
			}
			
			static void orientationAsAVector(float orientation, Vec3& output){
				output.x = sin(orientation);
				output.z = cos(orientation);
			}
			
			static const Kinematic zero;
	        private:
				static float random01(){
					double i = 0, d = 0;
					i = rand() % 4000 - 2000; // Gives a number between -2000 and +2000;
					d = i / 10000; // Reduces this number to the range you want.
					return d;
				}
	};
	
	const Kinematic Kinematic::zero (0.0f, 0.0f, 0.0f, 0.0f, Vec3::zero, 0.0f, Vec3::zero, 0.0f, "");
	
	// Used for behaviours changes with F1/F2
	typedef  void *steerFunc(const Kinematic& , Kinematic& , SteeringOutput&);
	std::vector<void (*)(const Kinematic& , Kinematic& , SteeringOutput&)> steerFunctions;
	static int steerFuncNum = 0;
	
	// BASIC STEERING BEHAVIORS
	
	class SteeringSeek{
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				steering.linear = target.position - character.position;
				steering.linear = steering.linear.normalize();
				steering.linear *= character.maxAcceleration;
				steering.angular = 0;
				
				character.setBehaviourName("STEERINGSEEK");
			}
	};
	
	class SteeringFlee{
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				steering.linear = character.position - target.position;
				steering.linear = steering.linear.normalize();
				steering.linear *= character.maxAcceleration;
				steering.angular = 0;
				
				character.setBehaviourName("STEERINGFLEE");
			}
	};
	
	class SteeringArrive{
		private:
			static const float targetRadius;
			static const float slowRadius;
			static const float timeToTarget;
			
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 direction = target.position - character.position,targetVelocity;
				float distance = direction.length(),targetSpeed;
				if(distance < targetRadius){
					targetSpeed = 0.0f;
				}else{
					if(distance > slowRadius)
						targetSpeed = character.maxSpeed;
					else
						targetSpeed = (character.maxSpeed * distance) / slowRadius;
				}
				targetVelocity = direction;
				targetVelocity = targetVelocity.normalize();
				targetVelocity *= targetSpeed;
				steering.linear = targetVelocity - character.velocity;
				steering.linear /= timeToTarget;
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				steering.angular = 0;
				
				character.setBehaviourName("STEERINGARRIVE");
			} 
	};
	
	const float SteeringArrive::targetRadius = 1.0f;
	const float SteeringArrive::slowRadius = 5.0f;
	const float SteeringArrive::timeToTarget = 0.1f;
	
	class SteeringAlign{
		private:
			static const float targetRadius;
			static const float slowRadius;
			static const float timeToTarget;
			static const float M_2PI;
			
			inline static float mapToRange(float a){
				a = fmod(a + M_PI, M_2PI) - M_PI;
				return a;
			}
			
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				float rotation = target.orientation - character.orientation;
				rotation = mapToRange(rotation);
				float rotationSize = abs(rotation),targetRotation;
				
				if(rotationSize < targetRadius){
					targetRotation = 0.0f;
				}else{
					if(rotationSize > slowRadius)
						targetRotation = character.maxRotation;
					else
						targetRotation = character.maxRotation * rotationSize / slowRadius;
				}
				targetRotation *= rotation / rotationSize;
				steering.angular = targetRotation - character.rotation;
				steering.angular /= timeToTarget;
				float angularAcceleration = abs(steering.angular);
				if(angularAcceleration > character.maxAngularAcceleration){
					steering.angular /= angularAcceleration;
					steering.angular *= character.maxAngularAcceleration;
				}
				steering.linear = Vec3::zero;
				
				character.setBehaviourName("STEERINALIGN");
			}
		
	};
	
	const float SteeringAlign::targetRadius = 0.1f;
	const float SteeringAlign::slowRadius = 0.5f;
	const float SteeringAlign::timeToTarget = 0.1f;
	const float SteeringAlign::M_2PI = 2 * M_PI;
	
	class SteeringVelocityMatch{
		private:
			static const float timeToTarget;
			
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				steering.linear = target.velocity - character.velocity;
				steering.linear /= timeToTarget;
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				steering.angular = 0;
			
				character.setBehaviourName("STEERINGVELOCITYMATCH");
			}
	};
	
	const float SteeringVelocityMatch::timeToTarget = 0.1f;
	
	// DELEGATED BEHAVIORS

	class SteeringFace{
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 direction = target.position - character.position;
				Kinematic alignTarget = target;
				alignTarget.orientation = Kinematic::getNewOrientation(character.orientation, direction);
				SteeringAlign::getSteering(alignTarget,character,steering);
				
				character.setBehaviourName("STEERINGFACE");
			}
	};
	
	class SteeringLookWhereYoureGoing{
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Kinematic alignTarget = character;
				alignTarget.orientation = Kinematic::getNewOrientation(character.orientation, character.velocity);
				SteeringAlign::getSteering(alignTarget,character,steering);
				
				character.setBehaviourName("STEERINGLOOKWHEREYOUREGOING");
			}
	};
	
	class SteeringWander{
		private:
			// Distance from the character position where the wander circulus will be positioned for the wander algorithm
			static const float wanderOffset;
			static const float wanderRadius;
			static const float wanderRate;
		
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 orientationVector;
				float r = Kinematic::randomBinomial();
				character.wanderOrientation += r * wanderRate;
				float targetOrientation = character.wanderOrientation + character.orientation;
				Kinematic::orientationAsAVector(character.orientation,orientationVector);
				Kinematic faceTarget = target;
				faceTarget.position = character.position + (wanderOffset * orientationVector);
				Vec3 targetOrientationVector;
				Kinematic::orientationAsAVector(targetOrientation,targetOrientationVector);
				
				// Draw circle in center
				OpenSteer::drawXZCircleOrDisk(wanderRadius, faceTarget.position, gRed, 200, false);
				
				faceTarget.position += wanderRadius * targetOrientationVector;
				SteeringFace::getSteering(faceTarget, character, steering);
				steering.linear = character.maxAcceleration * orientationVector;
				
				// Draw line to point
				OpenSteer::drawLine(character.position, faceTarget.position, gYellow);
				
				character.setBehaviourName("STEERINGWANDER");
			}
	
	};

	const float SteeringWander::wanderOffset = 5.0f;
	const float SteeringWander::wanderRadius = 1.0f;
	const float SteeringWander::wanderRate = 1.0f;
	
	class SteeringPursue{
		public:
			static const float maxPrediction;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 direction = target.position - character.position;
				float prediction;
				float distance = direction.length();
				float speed = character.velocity.length();
				if(speed <= distance / maxPrediction){
					prediction = maxPrediction;
				}else{
					prediction = distance /speed;
				}
				Kinematic seekTarget = target;
				seekTarget.position += target.velocity * prediction;
				SteeringSeek::getSteering(seekTarget, character, steering);
				
				character.setBehaviourName("STEERINGPURSUE");
			}
	};
	
	const float SteeringPursue::maxPrediction = 3.0f;
	
	class SteeringEvade{
		public:
			static const float maxPrediction;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 direction = target.position - character.position;
				float prediction;
				float distance = direction.length();
				float speed = character.velocity.length();
				if(speed <= distance / maxPrediction){
					prediction = maxPrediction;
				}else{
					prediction = distance /speed;
				}
				Kinematic seekTarget = target;
				seekTarget.position += target.velocity * prediction;
				SteeringFlee::getSteering(seekTarget, character, steering);
				
				character.setBehaviourName("STEERINGEVADE");
			}
	};
	
	const float SteeringEvade::maxPrediction = 3.0f;
	
	class SteeringSeparation{
		public:
			static const float threshold;
			static const float decayCoefficient;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 direction = character.position - target.position;
				float distance = direction.length();
				if(distance < threshold){
					float strength = min(decayCoefficient / (distance * distance),character.maxAcceleration);
					direction = direction.normalize();
					steering.linear += strength * direction;
				}
				
				character.setBehaviourName("STEERINGSEPARATION");
			}
			
		private:
			inline static float min(float a, float b){
				if(a < b)
					return a;
				return b;
			}
	};
	
	const float SteeringSeparation::threshold = 3.0f;
	const float SteeringSeparation::decayCoefficient = 3.0f;
	
	class Path{
		public:
			std::vector<Vec3> points;
			Path(){}
			
			void addPoint(Vec3 &point){
				points.push_back(point);
			}
			
			void draw(){
				int i,n = points.size();
				for(i = 0;i < n - 1; i++){
					Vec3 a = points[i];
					Vec3 b = points[i + 1];
					OpenSteer::drawLine(a, b, gWhite);				
				}
			}
			
			void clearPath(){
				points.clear();
			}
			
			void getNormalPoint(const Vec3& p, const Vec3& a, const Vec3& b, Vec3 &normal){
				Vec3 ap = p - a;
				Vec3 ab = b - a;
				ab = ab.normalize();
				Vec3 proy = ab * ap.dot(ab);
				normal = a + proy;
			}
			
			bool isInSegment(const Vec3& a, const Vec3& b, const Vec3& p){
				float distA, distB ,distSegment;
				// distance to a
				distA = (p - a).length();
				// distance to b
				distB = (p - b).length();
				distSegment = (b - a).length();
				if(distA < distSegment && distB < distSegment){
					return true;
				}
				return false;
			}
			
			void getClosestPoint(const Vec3& p, Vec3& closestPoint, int& segment){
				int i,n = points.size();
				float min = std::numeric_limits<float>::infinity(), distToCandidate, distToPoint;
				Vec3 candidatePoint, a, b;
				for(i = 0;i < n - 1; i++){
					a = points[i];
					b = points[i + 1];
					getNormalPoint(p, a, b, candidatePoint);
					
					if(isInSegment(a, b, candidatePoint)){
						distToCandidate = (candidatePoint - p).length();
					}else{
						distToCandidate = (p - b).length();
						distToPoint = (p - a).length();
						candidatePoint = b;
						if(distToPoint < distToCandidate){
							distToCandidate = distToPoint;
							candidatePoint = a;
						}
					}
					if(distToCandidate < min){
						min = distToCandidate;
						closestPoint = candidatePoint;
						segment = i;
					}
				}
				OpenSteer::drawLine(p, closestPoint, gYellow);	
			}
			
			bool isNearEnd(const Vec3& p){
				Vec3 lastPoint = points[points.size() - 1];
				if((p - lastPoint).length() < 0.1f)
					return true;
				return false;
			}
			
			bool isNearBeginning(const Vec3& p){
				Vec3 firstPoint = points[0];
				if((p - firstPoint).length() < 0.1f)
					return true;
				return false;
			}
			
			void getPosition(Vec3 &point, int segment, float pathOffset, Vec3& resultPoint){
				Vec3 a,b;
				int n = points.size();
				a = points[segment];
				b = points[segment + 1];
				resultPoint = point + (b - a).normalize() * pathOffset;
				//std::cout << "resultPoint: " << resultPoint << ", point: " << point << ", direction: " << (b-a).normalize() << ", pathOffset: " << pathOffset << "(" << a << ", " << b << ")" << std::endl;
				if(!isInSegment(a, b, resultPoint)){
					if(pathOffset > 0.0f){
						a = b;
						b = points[segment + 2];
						resultPoint = a + (b - a).normalize() * pathOffset;
					}else{
						b = points[segment - 1];
						resultPoint = a + (a - b).normalize() * pathOffset;
					}
				}
			}
			
			void reverse(){
				std::reverse(points.begin(), points.end());
			}
			
	};
    
    Path path;
    
    void initPath(){
    	Vec3 point(0.0f,0.0f,0.0f);
    	path.addPoint(point);
    	point.x = 10.0f;
    	path.addPoint(point);
    	point.x = 0.0f;
    	point.z = 10.0f;
    	path.addPoint(point);
    	point.x = 10.0f;
    	path.addPoint(point);
    }
    
    void endPath(){
    	path.clearPath();
    }
    
	class SteeringFollowPath{
		public:
			static float pathOffset;
			static const float predictTime;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 futurePos = character.position + character.velocity * predictTime;
				Vec3 currentParam;
				int segment = 0;
				path.getClosestPoint(futurePos, currentParam, segment);
				
				Vec3 targetParam;
				path.getPosition(currentParam, segment, pathOffset, targetParam);
				
				Kinematic seekTarget;
				seekTarget.position = targetParam;
				
				character.lastPathPoint = targetParam;
				OpenSteer::drawLine(character.position, targetParam, gRed);		
				
				SteeringSeek::getSteering(seekTarget, character, steering);
				character.setBehaviourName("STEERINGFOLLOWPATH");
				
			}
	};
	
	float SteeringFollowPath::pathOffset = 0.5f;
	const float SteeringFollowPath::predictTime = 0.01f;
	
	
	class SteeringCollisionAvoidance{
		public:
			// collision radius
			static const float radius;
			
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				int shortestTime = INT_MAX;
				bool targetSet = false;
				float firstDistance, firstMinSeparation, minSeparation, distance, relativeSpeed, timeToCollision;
				Vec3 relativePos, firstRelativePos, firstRelativeVel, relativeVel;
				Kinematic firstTarget = Kinematic::zero;
				relativePos = target.position - character.position;
				relativeVel = target.velocity - character.velocity;
				relativeSpeed = relativeVel.length();
				timeToCollision = -(relativePos.dot(relativeVel) / (relativeSpeed * relativeSpeed));
				distance = relativePos.length();
				minSeparation = distance - (relativeSpeed * timeToCollision);
				
				if(minSeparation > 2 * radius)
					return;
					
				if(timeToCollision > 0.0f && timeToCollision < shortestTime){
					shortestTime = timeToCollision;
					firstTarget = character;
					firstMinSeparation = minSeparation;
					firstDistance = distance;
					firstRelativePos = relativePos;
					firstRelativeVel = relativeVel;
					targetSet = true;
				}
				
				if(!targetSet)
					return;
				
				if(firstMinSeparation <= 0 || distance < 2 * radius){
					relativePos = firstTarget.position - character.position;
				}else{
					relativePos = firstRelativePos + (firstRelativeVel * shortestTime);
				}
				relativePos = relativePos.normalize();
				steering.linear = (-relativePos) * character.maxAcceleration;
				
				character.setBehaviourName("STEERINGCOLLISIONAVOIDANCE");
			}
	};
	
	const float SteeringCollisionAvoidance::radius = 0.5f;

	// This is for blended behaviour
	class BehaviourAndWeight{
		public:
			steerFunc *sFunc;
			float weight;
			
			BehaviourAndWeight(void): sFunc(NULL),weight(0.0f) {}
	};
	
	std::vector<BehaviourAndWeight *> behaviours;
	
	void initBlendedBehaviours(){
		BehaviourAndWeight *b = new BehaviourAndWeight();
		b->weight = 1.0f;
		b->sFunc = SteeringArrive::getSteering;
		behaviours.push_back(b);
		b = new BehaviourAndWeight();
		b->weight = 1.0f;
		b->sFunc = SteeringFace::getSteering;
		behaviours.push_back(b);
	}
	
	void endBlendedBehaviours(){
		for(std::vector<BehaviourAndWeight *>::iterator it = behaviours.begin(); it != behaviours.end(); ++it)
			delete(*it);
		behaviours.clear();
	}
	
	// Blending Behaviours with weigths
	class SteeringBlended{
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				for(std::vector<BehaviourAndWeight *>::iterator it = behaviours.begin(); it != behaviours.end(); ++it){
					SteeringOutput localSteering;
					(*((*it)->sFunc))(target,character,localSteering);
					steering.linear += (*it)->weight * localSteering.linear;
					steering.angular += (*it)->weight * localSteering.angular;
				}
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				if(steering.angular > character.maxRotation)
					steering.angular = character.maxRotation;
				
				character.setBehaviourName("STEERINGBLENDED");
			}
	};
	
	std::vector< std::vector<BehaviourAndWeight *> *> groups;
	
	void initPriorityGroups(){
	
		// Collision Avoidance
		std::vector<BehaviourAndWeight *> *behaviour = new std::vector<BehaviourAndWeight *>();
		BehaviourAndWeight *b = new BehaviourAndWeight();
		b->weight = 1.0f;
		b->sFunc = SteeringCollisionAvoidance::getSteering;
		behaviour->push_back(b);
		
		groups.push_back(behaviour);
		
		// Wander and LookWhereYoureGoing
		behaviour = new std::vector<BehaviourAndWeight *>();
		b = new BehaviourAndWeight();
		b->weight = 1.0f;
		b->sFunc = SteeringWander::getSteering;
		behaviour->push_back(b);
		b = new BehaviourAndWeight();
		b->weight = 1.0f;
		b->sFunc = SteeringLookWhereYoureGoing::getSteering;
		behaviour->push_back(b);
		
		groups.push_back(behaviour);
	}
	
	void endPriorityGroups(){
		for(std::vector< std::vector<BehaviourAndWeight *> *>::iterator group = groups.begin(); group != groups.end(); ++group){
			for(std::vector<BehaviourAndWeight *>::iterator it = (*group)->begin(); it != (*group)->end(); ++it){
				delete(*it);
			}
			delete(*group);
		}
		groups.clear();
	
	}
	
	// Priority Steering
	class SteeringPriority{
		public:
			
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				int i = 0;
				for(std::vector< std::vector<BehaviourAndWeight *> *>::iterator group = groups.begin(); group != groups.end(); ++group){
					getGroupSteering(**group, target, character, steering);
					if(steering.linear.length() > FLT_EPSILON  || abs(steering.angular) > FLT_EPSILON ){
						character.setBehaviourName("STEERINGPRIORITY");
						return;
					}
					++i;
				}
				
				character.setBehaviourName("STEERINGPRIORITY");
			}
		
		private:	
			static void getGroupSteering(std::vector<BehaviourAndWeight *>& behaviours, const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				for(std::vector<BehaviourAndWeight *>::iterator it = behaviours.begin(); it != behaviours.end(); ++it){
					SteeringOutput localSteering;
					(*((*it)->sFunc))(target,character,localSteering);
					steering.linear += (*it)->weight * localSteering.linear;
					steering.angular += (*it)->weight * localSteering.angular;
				}
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				if(steering.angular > character.maxRotation)
					steering.angular = character.maxRotation;
			}
	};
	
	
	inline std::ostream& operator<< (std::ostream& o, const SteeringOutput& s){
		return o << "linear vel: " << s.linear << std::endl << "angular vel: " << s.angular << std::endl;
	}
	
	inline std::ostream& operator<< (std::ostream& o, const Kinematic& k){
		return o << "position: " << k.position << std::endl << "orientation: " << k.orientation << std::endl << "velocity: " << k.velocity << std::endl << "rotation: " << k.rotation << std::endl;
	}
	
	class Connection{
		public:
			int toNode;
			int fromNode;
			float cost;
			static const Connection None;
		
			Connection(int from, int to,float cost){
				fromNode = from;
				toNode = to;
				this->cost = cost;
			}
		
			int getToNode(){
				return toNode;
			}
		
			float getCost(){
				return cost;
			}

			Connection operator= (const Connection& c) {
				this->toNode = c.toNode,this->fromNode = c.fromNode,this->cost=c.cost;
				return *this;
			}
			
			bool operator== (const Connection& n) const {return this->toNode == n.toNode && this->fromNode == n.fromNode && this->cost == n.cost;}
	};
	
	const Connection Connection::None (-1,-1,0.0f);
	
	class Node{
		public:
			int id;
			std::vector<Connection> adyacents;
			Vec3 position;
			static const Node None;
			
			Node(void): position(Vec3::zero){}
			
			~Node(){
				adyacents.clear();
			}
			
			Node(int id, const Vec3& position){
				this->position = position;
				this->id = id;
			}
			
			void addConnection(int b, float cost){
				Connection c (this->id, b, cost);
				adyacents.push_back(c);
			}
			
			inline Node operator= (const Node& n) {
				this->position = n.position;
				this->id = n.id;
				this->adyacents = n.adyacents;
				return *this;
			}
					
			inline bool operator== (const Node& n) const {return this->id == n.id;}
			
			inline bool operator!= (const Node& n) const {return (!(*this == n));}
	
	};
	
	const Node Node::None (0,Vec3::zero);
	
	inline std::ostream& operator<< (std::ostream& o, const Connection& c){
		return o << c.toNode << "(" << c.cost << ")";
	}
	
	inline std::ostream& operator<< (std::ostream& o, const Node& n){
		o << n.id << "(" << n.adyacents.size() << ")" << ": ";
		copy(n.adyacents.begin(), n.adyacents.end(), std::ostream_iterator<Connection>(o, " "));
		return o << std::endl;
	}
	
	class Graph{
		public:
			std::vector<Node> nodes;
			int lastid;
			
			Graph(void){lastid = 0;}
			
			int addNode(const Vec3& position){
				int node_id = lastid++;
				Node n (node_id, position);
				nodes.push_back(n);
				return node_id;
			}
			
			void addConnection(int a, int b, float cost){
				for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end();++it){
					if((*it).id == a)
						(*it).addConnection(b, cost);
				}
			}
			
			void getConnections(const Node &node, std::vector<Connection>& connections){
				for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it){
					if(*it == node){
						for(std::vector<Connection>::iterator c_it = (*it).adyacents.begin(); c_it != (*it).adyacents.end(); ++c_it){
							connections.push_back(*c_it);
						}
						break;
					}
				}
			}
			
			void clear(){
				nodes.clear();
			}
			
			void draw(){
				for(std::vector<Node>::iterator n = nodes.begin();n != nodes.end();++n){
					Vec3 p1 = (*n).position;
					drawCircleOrDisk (0.25, Vec3 (0.0f,1.0f,0.0f), p1, gOrange, 30, true, true);
					std::ostringstream node_id;
					node_id << (*n).id << std::endl;
					draw2dTextAt3dLocation (node_id, p1, gWhite, drawGetWindowWidth(), drawGetWindowHeight());

					for(std::vector<Connection>::iterator c = (*n).adyacents.begin(); c!= (*n).adyacents.end();++c){
						Vec3 p2 = nodes[(*c).toNode].position;
						OpenSteer::drawLine(p1, p2, gOrange);
						
						Vec3 tpos = p2 - p1;
						float l = tpos.length();
						const Vec3 textOrigin = p1 + tpos.normalize() * l/2 + Vec3 (0, 0.25, 0);
						std::ostringstream annote;
						annote << (*c).cost << std::endl;
						draw2dTextAt3dLocation (annote, textOrigin, gOrange, drawGetWindowWidth(), drawGetWindowHeight());
					}
				}
			}
	};
	
	inline std::ostream& operator<< (std::ostream& o, const Graph& s){
		copy(s.nodes.begin(), s.nodes.end(), std::ostream_iterator<Node>(o, ""));
		return o;
	}
	
	Graph g;
	
	void initGraph(){
		/*
		Vec3 p1 (0.0f, 0.0f, 0.0f);
		Vec3 p2 (5.0f, 0.0f, 0.0f);
		Vec3 p3 (10.0f, 0.0f, 5.0f);
		Vec3 p4 (13.0f, 0.0f, 10.0f);
		int n0 = g.addNode(p1);
		int n1 = g.addNode(p2);
		int n2 = g.addNode(p3);
		int n3 = g.addNode(p4);
		
		std::cout << "NODES ADDED: " << std::endl << g << std::endl;
		g.addConnection(n0, n1, (p0 - p1).length());
		g.addConnection(n0, n2, (p0 - p2).length());
		g.addConnection(n1, n2, (p1 - p2).length());
		g.addConnection(n2, n3, (p2 - p3).length());
		g.addConnection(n1, n3, (p1 - p3).length());
		std::cout << g << std::endl;
		*/
		/*
		Vec3 p0 (0.0f, 0.0f, 0.0f);
		Vec3 p1 (5.0f, 0.0f, 0.0f);
		Vec3 p2 (10.0f, 0.0f, 0.0f);
		Vec3 p3 (2.0f, 0.0f, 5.0f);
		Vec3 p4 (6.0f, 0.0f, 5.0f);
		Vec3 p5 (5.0f, 0.0f, 10.0f);
		Vec3 p6 (10.0f, 0.0f, 10.0f);
		
		int n0 = g.addNode(p0);
		int n1 = g.addNode(p1);
		int n2 = g.addNode(p2);
		int n3 = g.addNode(p3);
		int n4 = g.addNode(p4);
		int n5 = g.addNode(p5);
		int n6 = g.addNode(p6);
		
		g.addConnection(n0, n1, (p0 - p1).length());
		g.addConnection(n1, n2, (p1 - p2).length());
		g.addConnection(n0, n3, (p0 - p3).length());
		g.addConnection(n1, n4, (p1 - p4).length());
		g.addConnection(n3, n4, (p3 - p4).length());
		g.addConnection(n3, n5, (p3 - p5).length());
		g.addConnection(n4, n6, (p4 - p6).length());
		*/
		Vec3 p0 (0.0f, 0.0f, 0.0f);
		Vec3 p1 (5.0f, 0.0f, -2.0f);
		Vec3 p2 (10.0f, 0.0f, -5.0f);
		Vec3 p3 (2.0f, 0.0f, 10.0f);
		Vec3 p4 (5.0f, 0.0f, 5.0f);
		Vec3 p5 (10.0f, 0.0f, 0.0f);
		Vec3 p6 (10.0f, 0.0f, 10.0f);
		
		int n0 = g.addNode(p0);
		int n1 = g.addNode(p1);
		int n2 = g.addNode(p2);
		int n3 = g.addNode(p3);
		int n4 = g.addNode(p4);
		int n5 = g.addNode(p5);
		int n6 = g.addNode(p6);
		
		g.addConnection(n0, n1, (p0 - p1).length());
		g.addConnection(n1, n2, (p1 - p2).length());
		g.addConnection(n0, n3, (p0 - p3).length());
		g.addConnection(n0, n4, (p0 - p4).length() + 10.0f);
		g.addConnection(n1, n5, (p1 - p5).length());
		g.addConnection(n3, n4, (p3 - p4).length());
		g.addConnection(n4, n6, (p4 - p6).length());
		g.addConnection(n5, n6, (p5 - p6).length() + 5.0f);
	}
	
	
	void endGraph(){
		g.clear();
	}

	class NodeRecord{
		public:
			Node node;
			Connection connection;
			float costSoFar;
			float estimatedTotalCost;
			static const NodeRecord None;
			
			NodeRecord(void): node(),connection(Connection::None), costSoFar(0.0f), estimatedTotalCost(0.0f){}
			
			NodeRecord(const Node &n, const Connection &c, float cost, float estimated): node(n),connection(c),costSoFar(cost),estimatedTotalCost(estimated){}
			
			NodeRecord operator= (const NodeRecord& n) {
				this->node = n.node, this->connection = n.connection,this->costSoFar = n.costSoFar,
				this->estimatedTotalCost = n.estimatedTotalCost;
				return *this;
			}
    
			bool operator== (const NodeRecord& n) const {
				return this->node == n.node && this->connection == n.connection && Compare::nearlyEqual(this->costSoFar,n.costSoFar,FLT_EPSILON) && 	 Compare::nearlyEqual(this->estimatedTotalCost,n.estimatedTotalCost,FLT_EPSILON);
			}
			
			bool operator!= (const NodeRecord& n) const {
				return !(*this == n);
			}
			
	};
	
	struct compareNodeRecord{
		static bool compare(const NodeRecord& n1, const NodeRecord& n2){
			return n1.estimatedTotalCost > n2.estimatedTotalCost;
		}
	};
		
	
	const NodeRecord NodeRecord::None (Node::None,Connection::None,0.0f,0.0f);
	
	inline std::ostream& operator<< (std::ostream& o, const NodeRecord& s){
		return o << "Nodo: " << s.node << ", Coneccion: " << s.connection << ", soFar: " << s.costSoFar << ", estimated: " << s.estimatedTotalCost;
	}
	
	class Heuristic{
		public:
			static float estimate(Node &goal,Node &end){
				return (end.position - goal.position).length();
			}
	};
	
	class PathFindingList{
		public:
			PathFindingList(){}
			
			bool contains(const Node &node, NodeRecord& result){
				for(std::vector<NodeRecord>::iterator it = list.begin(); it != list.end(); ++it){
					if((*it).node == node){
						result = *it;
						return true;
					}
				}
				return false;
			}
			
			bool remove(const NodeRecord &node){
				for(std::vector<NodeRecord>::iterator it = list.begin(); it != list.end(); ++it){
					if(*it == node){
						list.erase(it);
						break;
					}
				}
			}
			
			static NodeRecord searchForNode(Node& node, PathFindingList& list1, PathFindingList& list2){
				NodeRecord res = NodeRecord::None;
				if(!list1.contains(node, res))
					list2.contains(node, res);
				return res;
			}
			
			void push(const NodeRecord& n){
				list.push_back(n);
			}
			
			int size(){
				return list.size();
			}
			
			bool empty(){
				return list.size() == 0;
			}
			
			bool not_empty(){
				return !empty();
			}
			
			NodeRecord findSmallest(){
				std::vector<NodeRecord>::iterator it = list.begin();
				NodeRecord smallest = *it;
				for(it +=  1; it != list.end(); ++it){
					if((*it).estimatedTotalCost < smallest.estimatedTotalCost)
						smallest = *it;
				}
				return smallest;
			}
			
			NodeRecord findSmallestDijkstra(){
				std::vector<NodeRecord>::iterator it = list.begin();
				NodeRecord smallest = *it;
				for(it +=  1; it != list.end(); ++it){
					if((*it).costSoFar < smallest.costSoFar)
						smallest = *it;
				}
				return smallest;
			}
			
		private:
			std::vector<NodeRecord> list;
	
	};
	
	class PathFinding{
		public:
			
			static bool pathFindAStar(Graph& graph, Node& start, Node& goal, Path& path){
				NodeRecord startRecord;
				startRecord.node = start;
				startRecord.connection = Connection::None;
				startRecord.costSoFar = 0.0f;
				startRecord.estimatedTotalCost = Heuristic::estimate(start,goal);
				
				PathFindingList open;
				open.push(startRecord);
				PathFindingList closed;
				
				NodeRecord current = NodeRecord::None;
				while(open.not_empty()){
					current = open.findSmallest();
					if(current.node == goal)
						break;
					std::vector<Connection> connections;
					graph.getConnections(current.node, connections);
					float endNodeHeuristic;
					float endNodeCost;
					for(std::vector<Connection>::iterator it = connections.begin(); it != connections.end(); ++it){
						Connection connection = *it;
						Node endNode = graph.nodes[connection.toNode];
						endNodeCost = current.costSoFar + connection.getCost();
						NodeRecord endNodeRecord = NodeRecord::None;
						if(closed.contains(endNode, endNodeRecord)){
							if(endNodeRecord.costSoFar <= endNodeCost)
								continue;
							closed.remove(endNodeRecord);
							endNodeHeuristic = endNodeRecord.estimatedTotalCost - endNodeRecord.costSoFar;
						}else if(open.contains(endNode, endNodeRecord)){
							if(endNodeRecord.costSoFar <= endNodeCost)
								continue;
							open.remove(endNodeRecord);
							endNodeHeuristic = endNodeRecord.estimatedTotalCost - endNodeRecord.costSoFar;
						}else{
							endNodeRecord.node = endNode;
							endNodeHeuristic = Heuristic::estimate(endNode, goal);
						}
						endNodeRecord.costSoFar = endNodeCost;
						endNodeRecord.connection = connection;
						endNodeRecord.estimatedTotalCost = endNodeCost + endNodeHeuristic;
						NodeRecord check;
						if(!open.contains(endNode, check)){
							open.push(endNodeRecord);
						}
					}
					open.remove(current);
					closed.push(current);
				}
				
				if(current.node != goal)
					return false;
				while(current.node != start){
					path.addPoint(current.node.position);
					Node prev = graph.nodes[current.connection.fromNode];
					current = PathFindingList::searchForNode(prev, open, closed);
				}
				path.addPoint(start.position);
				path.reverse();
				return true;
			}
			
			static bool pathFindDijkstra(Graph& graph, Node& start, Node& goal, Path& path){
				NodeRecord startRecord;
				startRecord.node = start;
				startRecord.connection = Connection::None;
				startRecord.costSoFar = 0;
				
				PathFindingList open;
				open.push(startRecord);
				PathFindingList closed;
				
				NodeRecord current;
				
				while(open.not_empty()){
					current = open.findSmallestDijkstra();
					if(current.node == goal)
						break;
					std::vector<Connection> connections;
					graph.getConnections(current.node, connections);
					for(std::vector<Connection>::iterator it = connections.begin(); it != connections.end(); ++it){
						Connection connection = *it;
						Node endNode = graph.nodes[connection.toNode];
						float endNodeCost = current.costSoFar + connection.getCost();
						NodeRecord endNodeRecord, check;
						if(closed.contains(endNode, check)){
							continue;
						}else if(open.contains(endNode, endNodeRecord)){
							if(endNodeRecord.costSoFar <= endNodeCost){
								continue;
							}else{
								open.remove(endNodeRecord);
							}
						}else{
							endNodeRecord.node = endNode;
						}
						endNodeRecord.costSoFar = endNodeCost;
						endNodeRecord.connection = connection;
						if(!open.contains(endNode, check))
							open.push(endNodeRecord);
					}
					open.remove(current);
					closed.push(current);
				}
				if(current.node != goal)
					return false;
				
				while(current.node != start){
					path.addPoint(current.node.position);
					Node prev = graph.nodes[current.connection.fromNode];
					current = PathFindingList::searchForNode(prev, open, closed);
				}
				path.addPoint(start.position);
				path.reverse();
				return true;
			}
	};

    // ----------------------------------------------------------------------------
    // This PlugIn uses tree vehicle types: CtfAgent, CtfPlayer and CtfProyectile.  They have a
    // common base class: CtfBase which is a specialization of SimpleVehicle.


    class CtfBase : public SimpleVehicle
    {
		public:
			
		    // constructor
		    CtfBase () {reset ();}

		    // reset state
		    void reset (void);

		    // draw this character/vehicle into the scene
		    void draw (void);

		    void randomizeStartingPositionAndHeading (void);
		     
		    // for draw method
		    Color bodyColor;
		    
		    // for steering algorithms
			Kinematic k;
    };

    class CtfAgent : public CtfBase
    {
		public:

		    // constructor
		    CtfAgent () {reset ();}

		    // reset state
		    void reset (void);

		    // per frame simulation update
		    void update (const float currentTime, const float elapsedTime);
	
			// draw
		    void draw (void);
    };

	class JumpPoint{
		public:
			Vec3 jumpLocation;
			Vec3 landingLocation;
			Vec3 deltaPosition;
			
			JumpPoint() {}
	};
	
    class CtfPlayer : public CtfBase
    {
		public:

		    // constructor
		    CtfPlayer () {reset ();}

		    // reset state
		    void reset (void);

		    // per frame simulation update
		    void update (const float currentTime, const float elapsedTime);
		    
		    // draw
		    void draw (void);
		    
		    // if true then the player have to jump
		    bool jump;
		    
		    // Holds information of the jump
		    JumpPoint jpoint;
		    
    };

	class CtfProyectile : public CtfBase
	{
		public:
			
			// constructor
			CtfProyectile () {reset ();}
			
			CtfProyectile (const float currentTime, Vec3 init, Vec3 d, const float mu): initialTime(currentTime), initialPosition(init), direction(d), muzzleVelocity(mu) {reset ();}
			
			~CtfProyectile (){}
			
			// reset state
			void reset (void);
			
			// per frame simulation update
			void update (const float currentTime, const float elapsedTime);
			
			// draw
			void draw (void);
			
			// Initial position
			Vec3 initialPosition;
			
			// proyectile direction
			Vec3 direction;
			
			// Time of fire
			float initialTime;
			
			// muzzle velocity used to launch this proyectile
			float muzzleVelocity;
			
			// inclination angle from xz plane.
			float angle;
			
			// Tells if this element should be erased
			bool shouldErase();
	};
	
    // ----------------------------------------------------------------------------
    // globals
    // (perhaps these should be member variables of a Vehicle or PlugIn class)


    const Vec3 gHomeBaseCenter (0, 0, 0);
    const float gHomeBaseRadius = 1.5;

    const float gMinStartRadius = 30;
    const float gMaxStartRadius = 40;

    const float gBrakingRate = 0.75;

    const Color evadeColor     (0.6f, 0.6f, 0.3f); // annotation
    const Color seekColor      (0.3f, 0.6f, 0.6f); // annotation
    const Color clearPathColor (0.3f, 0.6f, 0.3f); // annotation

    // count the number of times the simulation has reset (e.g. for overnight runs)
    int resetCount = 0;


    CtfAgent* ctfAgent;
    CtfPlayer* ctfPlayer;


    // ----------------------------------------------------------------------------
    // reset state


    void CtfBase::reset (void)
    {
        SimpleVehicle::reset ();  // reset the vehicle 

        randomizeStartingPositionAndHeading ();  // new starting position

        clearTrailHistory ();     // prevent long streaks due to teleportation
    }


    void CtfAgent::reset (void)
    {
        CtfBase::reset ();
        bodyColor.set (0.4f, 0.4f, 0.6f); // blueish
    }


    void CtfPlayer::reset (void)
    {
        CtfBase::reset ();
        jump = false;
        jpoint.jumpLocation = Vec3::zero;
        bodyColor.set (0.6f, 0.4f, 0.4f); // redish
    }

	void CtfProyectile::reset (void)
	{
		CtfBase::reset ();
		bodyColor.set (0.4f, 0.6f, 0.4f); // greenish
	}
	

    // ----------------------------------------------------------------------------


    void CtfBase::randomizeStartingPositionAndHeading (void)
    {
        // randomize position on a ring between inner and outer radii
        // centered around the home base
        const float rRadius = frandom2 (gMinStartRadius, gMaxStartRadius);
        const Vec3 randomOnRing = RandomUnitVectorOnXZPlane () * rRadius;
        setPosition (gHomeBaseCenter + randomOnRing);
        // randomize 2D heading
        randomizeHeadingOnXZPlane ();
    }

    void CtfPlayer::update (const float currentTime, const float elapsedTime){
    	SteeringOutput s;
    	// Vec3 direction = k.velocity;
    	Vec3 direction;
    	Kinematic::orientationAsAVector(k.orientation, direction);
    	Vec3 left (direction.z,0.0f,-direction.x);
    	if(direction == Vec3::zero)
    		left.x = -1.0f,left.z = 0.0f;
    	// Move forward
		if (k.keymap['w' - 'a']){
			s.linear = (s.linear + direction) * k.maxAcceleration;
			k.keymap['w' - 'a'] = 0;
		}
		// Move backward
		if (k.keymap['s' - 'a']){
			s.linear = (s.linear - direction) * k.maxAcceleration;
			k.keymap['s' - 'a'] = 0;
		}
		// Strafe right
		if (k.keymap['d' - 'a']){
			s.linear = (s.linear - left) * k.maxAcceleration;
			k.keymap['d' - 'a'] = 0;
		}
		// Strafe left
		if (k.keymap['a' - 'a']){
			s.linear = (s.linear + left) * k.maxAcceleration;
			k.keymap['a' - 'a'] = 0;
		}
		
		if(jump){
			if(jpoint.jumpLocation == Vec3::zero){
				jpoint.jumpLocation = k.position;
				k.velocity = direction;
				k.velocity.y = 1.0f;
				k.velocity = k.velocity.normalize();
				k.velocity *= k.maxSpeed;
				float time = (-2 * k.velocity.y) / GRAVITY.y;
				jpoint.landingLocation = jpoint.jumpLocation + (k.velocity * time) + (0.5 * GRAVITY * time * time);
			}
			float dToLanding = (jpoint.landingLocation - k.position).length();
			if(k.position.y <= 0.1f  && dToLanding <= 0.1f){
				k.position.y = 0.0f;
				k.velocity.y = 0.0f;
				k.velocity = k.velocity.normalize();
				k.velocity *= k.maxSpeed;
				jump = false;
				jpoint.jumpLocation = Vec3::zero;
			}else{
				s.linear = GRAVITY;
			}
		}else{
			s.linear = s.linear.normalize();
			s.linear = s.linear * k.maxAcceleration;
		}
		k.update(s,elapsedTime);
		
		// Fix orientation
		k.orientation = Kinematic::getNewOrientation(k.orientation,k.velocity);
    	if(k.velocity != Vec3::zero)
			regenerateOrthonormalBasisUF(k.velocity.normalize());
		recordTrailVertex (currentTime, k.position);
    }

    
    // ----------------------------------------------------------------------------
    // update method for goal seeker


    void CtfAgent::update (const float currentTime, const float elapsedTime)
    {
        SteeringOutput s;
        steerFunc *getSteering = (steerFunc *)steerFunctions[steerFuncNum];
        (*getSteering)(ctfPlayer->k,k,s);
		k.update(s,elapsedTime);
		
		// Fix orientation
    	Vec3 orientation (sin(k.orientation),0.0f,cos(k.orientation));
    	regenerateOrthonormalBasisUF(orientation);
    	
		// annotation
		recordTrailVertex (currentTime, k.position);
    }
    
    // Update for proyectiles!
        
    void CtfProyectile::update (const float currentTime, const float elapsedTime)
    {
    	float time = currentTime - initialTime;
    	Vec3 fterm = direction * muzzleVelocity * time;
    	Vec3 sterm = 0.5 * GRAVITY * time * time;
    	Vec3 oldPos = k.position;
		k.position = initialPosition + fterm + sterm;
		Vec3 dir = k.position - oldPos;
		dir = dir.normalize();
		regenerateOrthonormalBasisUF(dir);
		recordTrailVertex (currentTime, k.position);
    }
    
    bool CtfProyectile::shouldErase(){
    	// Erase when is about to hit the floor
    	if(k.position.y < 0.0)
    		return true;
    	return false;
    }
    
    // ----------------------------------------------------------------------------
    // PlugIn for OpenSteerDemo

	bool drawGraph = true;
	bool runAStar = true;
	
    class CtfPlugIn : public PlugIn
    {
		public:

		    const char* name (void) {return "Steering Seek";}

		    float selectionOrderSortKey (void) {return 0.01f;}

		    virtual ~CtfPlugIn() {} // be more "nice" to avoid a compiler warning

		    void open (void)
		    {
		    	steerFunctions.push_back(SteeringArrive::getSteering);
		    	/*
		    	steerFunctions.push_back(SteeringFollowPath::getSteering);
		    	steerFunctions.push_back(SteeringPriority::getSteering);
		    	steerFunctions.push_back(SteeringCollisionAvoidance::getSteering);
		    	steerFunctions.push_back(SteeringSeparation::getSteering);
		    	steerFunctions.push_back(SteeringPursue::getSteering);
		    	steerFunctions.push_back(SteeringEvade::getSteering);
		    	steerFunctions.push_back(SteeringBlended::getSteering);
		    	steerFunctions.push_back(SteeringWander::getSteering);
		    	steerFunctions.push_back(SteeringSeek::getSteering);
		    	steerFunctions.push_back(SteeringFlee::getSteering);
		    	steerFunctions.push_back(SteeringAlign::getSteering);
		    	steerFunctions.push_back(SteeringVelocityMatch::getSteering);
		    	steerFunctions.push_back(SteeringFace::getSteering);
		    	steerFunctions.push_back(SteeringLookWhereYoureGoing::getSteering);
		    	*/
		    	initPath();
		    	initGraph();
		    	
		    	initBlendedBehaviours();
		    	initPriorityGroups();
				srand(time(0));
		        // create the seeker ("hero"/"attacker")
		        ctfAgent = new CtfAgent;
		        Vec3 pos1 (1,0,1);
		        Vec3 pos2 (6,0,6);
		        ctfAgent->setPosition(pos1);
		        ctfAgent->k.setPosition(pos1);
		        all.push_back (ctfAgent);
				ctfPlayer = new CtfPlayer;
				ctfPlayer->setPosition(pos2);
				ctfPlayer->k.setPosition(pos2);
				all.push_back (ctfPlayer);

		        // initialize camera
		        OpenSteerDemo::init2dCamera (*ctfPlayer);
		        OpenSteerDemo::camera.mode = Camera::cmFixedDistanceOffset;
		        OpenSteerDemo::camera.fixedTarget.set (15, 0, 0);
		        OpenSteerDemo::camera.fixedPosition.set (80, 60, 0);

		    }

		    void update (const float currentTime, const float elapsedTime)
		    {
		        // update the seeker
		        ctfAgent->update (currentTime, elapsedTime);
		      	ctfPlayer->update (currentTime, elapsedTime);
		      	if(ctfPlayer->k.keymap['f' - 'a'] == 1){
		      		fireVehicle(currentTime, *ctfPlayer, M_PI / 4, 10.0);
		      		ctfPlayer->k.keymap['f' - 'a'] = 0;
		      	}
		      	// update proyectiles
		      	for(std::vector<CtfProyectile *>::iterator it = proyectiles.begin() ; it != proyectiles.end(); ++it)
					(*it)->update (currentTime, elapsedTime);
				
				// Draw path
				// path.draw();
				
				// Draw graph
				if(drawGraph)
					g.draw();
				
				// Find A* path
				Path p;
				if(runAStar)
					PathFinding::pathFindDijkstra(g, g.nodes[0], g.nodes[g.nodes.size() - 1], p);
					//PathFinding::pathFindAStar(g, g.nodes[0], g.nodes[g.nodes.size() - 1], p);
				p.draw();
				
				if(path.isNearEnd(ctfAgent->k.position) || path.isNearBeginning(ctfAgent->k.position))
					SteeringFollowPath::pathOffset *= -1.0f;
				
		    }

		    void redraw (const float currentTime, const float elapsedTime)
		    {
		        // selected vehicle (user can mouse click to select another)
		        AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

		        // vehicle nearest mouse (to be highlighted)
		        AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();

		        // update camera
		        OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

		        // draw "ground plane" centered between base and selected vehicle
		        const Vec3 goalOffset = gHomeBaseCenter-OpenSteerDemo::camera.position();
		        const Vec3 goalDirection = goalOffset.normalize ();
		        const Vec3 cameraForward = OpenSteerDemo::camera.xxxls().forward();
		        const float goalDot = cameraForward.dot (goalDirection);
		        const float blend = remapIntervalClip (goalDot, 1, 0, 0.5, 0);
		        const Vec3 gridCenter = interpolate (blend,
		                                             selected.position(),
		                                             gHomeBaseCenter);
		        OpenSteerDemo::gridUtility (gridCenter);

		        // draw the all the objects and home base
		        ctfAgent->draw();

				ctfPlayer->draw();
				
				int i,n = proyectiles.size();
				for(i = 0;i < n;++i){
					if(proyectiles[i]->shouldErase()){
						delete(*(proyectiles.begin() + i));
						proyectiles.erase(proyectiles.begin() + i);
						n--;
					}else{
						proyectiles[i]-> draw();
					}
				}

		        // highlight vehicle nearest mouse
		        OpenSteerDemo::highlightVehicleUtility (nearMouse);
		    }

		    void close (void)
		    {
		        // delete seeker
		        delete (ctfAgent);
		        ctfAgent = NULL;
	
				delete (ctfPlayer);
				ctfPlayer = NULL;
				
				endBlendedBehaviours();
				endPriorityGroups();
				endPath();
				endGraph();

		        // clear the group of all vehicles
		        all.clear();
		        
		        // clear the proyectiles
		        proyectiles.clear();
		    }

		    void reset (void)
		    {
		        // count resets
		        resetCount++;

		        // reset the seeker ("hero"/"attacker") and enemies
		        ctfAgent->reset ();
		        ctfPlayer->reset ();

		        // reset camera position
		        OpenSteerDemo::position2dCamera (*ctfAgent);

		        // make camera jump immediately to new position
		        OpenSteerDemo::camera.doNotSmoothNextMove ();
		    }
			
			void fireVehicle (const float currentTime, CtfBase& base, float angle, float mv){
				std::cout << "VEHICLE FIRED FROM " << base.k.position << std::endl;
				Vec3 direction;
				Kinematic::orientationAsAVector(base.k.orientation,direction);
				direction.y = cos(angle);
				direction = direction.normalize();
				CtfProyectile *p = new CtfProyectile(currentTime,base.k.position,direction,mv);
				proyectiles.push_back(p);
			}
			
		    void handleFunctionKeys (int keyNumber)
		    {
		    	int n = steerFunctions.size();
		    	switch(keyNumber){
		    		case 1:
		    			steerFuncNum = (steerFuncNum - 1) % n;
		    			if(steerFuncNum < 0)
		    				steerFuncNum = n - 1;
		    			break;
		    		case 2:
		    			steerFuncNum = (steerFuncNum + 1) % n;
		    			break;
		    		case 3:
		    			drawGraph = !drawGraph;
		    			break;
		    		case 4:
		    			runAStar = !runAStar;
		    			break;
		    	}
		    }

			void handleKeyboardKeys(unsigned char key, int /*x*/, int /*y*/){
				if(key == ' '){
					ctfPlayer->jump = true;
				}else{
					ctfPlayer->k.keymap[key - 'a'] = 1;			
				}
			}
		
		    void printMiniHelpForFunctionKeys (void)
		    {
		        std::ostringstream message;
		        message << "Function keys handled by ";
		        message << '"' << name() << '"' << ':' << std::ends;
		        OpenSteerDemo::printMessage (message);
		        OpenSteerDemo::printMessage ("");
		    }

		    const AVGroup& allVehicles (void) {return (const AVGroup&) all;}

		    // a group (STL vector) of all vehicles in the PlugIn
		    std::vector<CtfBase*> all;
		    
		    std::vector<CtfProyectile*> proyectiles;
		};

		CtfPlugIn gCtfPlugIn;

    // ----------------------------------------------------------------------------

    // ----------------------------------------------------------------------------
    // draw this character/vehicle into the scene


    void CtfBase::draw (void)
    {
    	setPosition(k.position);
    	setSpeed(k.velocity.length());
        drawBasic2dCircularVehicle (*this, bodyColor);
        drawTrail ();
    }

    void CtfPlayer::draw (void)
    {
        CtfBase::draw();
    }
    
    void CtfProyectile::draw (void)
    {
		CtfBase::draw();
    }
    
    void CtfAgent::draw (void){
        // first call the draw method in the base class
        CtfBase::draw();
        k.printBehaviour();

        // display status in the upper left corner of the window
        std::ostringstream status;
        status << resetCount << " restarts" << std::ends;
        const float h = drawGetWindowHeight ();
        const Vec3 screenLocation (10, h-50, 0);
        draw2dTextAt2dLocation (status, screenLocation, gGray80, drawGetWindowWidth(), drawGetWindowHeight());
    }

} // anonymous namespace

