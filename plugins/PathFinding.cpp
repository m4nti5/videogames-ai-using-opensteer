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
#include <fstream>
#include <string>
#include <iomanip>
#include <limits.h>
#include <time.h>
#include <limits>
#include <algorithm>
#include "OpenSteer/Annotation.h"
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Color.h"
#include "OpenSteer/cpphop.hpp"
#include "OpenSteer/VantagePoints.h"
#include "OpenSteer/Compare.h"
#include "OpenSteer/Polygon.h"
#include "OpenSteer/SteeringOutput.h"
#include "OpenSteer/Kinematic.h"
#include "OpenSteer/CustomPath.h"

#define WORLD_FILE	"../files/world2.obj"
#define MESH_FILE	"../files/mesh2.obj"
#define INITIAL_AGENT_STATUS 100.0f
#define INITIAL_PLAYER_STATUS 100.0f
#define DAMAGE_RADIUS 5.0f
#define PROYECTILE_DAMAGE 20.0f
#define PLANTHRESHOLD 3

namespace{
    using namespace OpenSteer;
	float SAFE_DISTANCE = 15.0f;
	
    // Update for proyectiles!
        
    const Vec3 GRAVITY (0, -9.81, 0);
    const Vec3 playerFlagPosition (10.0, 0.0, 25.0);
    const Vec3 computerFlagPosition (90.0, 0.0, 25.0);

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
			static const float slowRadius;
			static const float timeToTarget;
			
		public:
			static const float targetRadius;
			
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
    
    CustomPath path;
    
    void initCustomPath(){
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
    
    void endCustomPath(){
    	path.clearCustomPath();
    }
    
	class SteeringFollowCustomPath{
		public:
			static float pathOffset;
			static const float predictTime;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 futurePos = character.position + character.velocity * predictTime;
				Vec3 currentParam;
				int segment = 0;
				character.path.getClosestPoint(futurePos, currentParam, segment);
				
				Vec3 targetParam;
				character.path.getPosition(currentParam, segment, pathOffset, targetParam);
				
				Kinematic seekTarget;
				seekTarget.position = targetParam;
				
				character.lastCustomPathPoint = targetParam;
				OpenSteer::drawLine(character.position, targetParam, gRed);		
				
				SteeringSeek::getSteering(seekTarget, character, steering);
				character.setBehaviourName("STEERINGFOLLOWPATH");
			}
			
			static void getCustomPathSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering, CustomPath &path){
				Vec3 futurePos = character.position + character.velocity * predictTime;
				Vec3 currentParam;
				int segment = 0;
				path.getClosestPoint(futurePos, currentParam, segment);
				
				Vec3 targetParam;
				path.getPosition(currentParam, segment, pathOffset, targetParam);
				
				Kinematic seekTarget;
				seekTarget.position = targetParam;
				
				character.lastCustomPathPoint = targetParam;
				OpenSteer::drawLine(character.position, targetParam, gRed);		
				
				SteeringSeek::getSteering(seekTarget, character, steering);
				character.setBehaviourName("STEERINGPATH");
			}
	};
	
	float SteeringFollowCustomPath::pathOffset = 0.5f;
	const float SteeringFollowCustomPath::predictTime = 0.01f;
	
	
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
	
	// Reading files for Walls and for nodes
	template <class T>
	class tuple{
		public:
			T first;
			T second;
			T third;
			tuple (T f, T s, T t):first(f), second(s), third(t){}
			tuple(){}
	};
	
	bool loadMap(const char* path, std::vector< Vec3 >& vertex, std::vector< tuple<int> >& triangles, VantagePoints &vpoints){
		std::ifstream file;
		std::string line;
		int tn = 0;
		char op, newline;
		tuple <int> t;
		Vec3 v;
		file.open(path, std::ios::in);
		if(!file.is_open())
			return false;
		while(file >> op){
			switch(op){
				case 'v':
					file >> std::setprecision(6) >> std::fixed >> v.x >> v.y >> v.z;
					vertex.push_back(v);
					break;
				case 'f':
					file >> t.first >> t.second >> t.third;
					triangles.push_back(t);
					break;
				case 'h':
					file >> std::setprecision(6) >> std::fixed >> v.x >> v.y >> v.z;
					vpoints.add(v);
					break;
				case '\n':
					break;
				default:
					getline(file, line);
					break;
			}
		}
		file.close();
		return false;
	}
	
	bool loadMesh(const char* path, std::vector< Vec3 >& vertex, std::vector< tuple<int> >& triangles){
		std::ifstream file;
		std::string line;
		int tn = 0;
		char op, newline;
		tuple <int> t;
		Vec3 v;
		file.open(path, std::ios::in);
		if(!file.is_open())
			return false;
		while(file >> op){
			switch(op){
				case 'v':
					file >> std::setprecision(6) >> std::fixed >> v.x >> v.y >> v.z;
					vertex.push_back(v);
					break;
				case 'f':
					file >> t.first >> t.second >> t.third;
					triangles.push_back(t);
					break;
				case '\n':
					break;
				default:
					getline(file, line);
					break;
			}
		}
		file.close();
		return false;
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
	
	class CustomPathFindingList{
		public:
			CustomPathFindingList(){}
			
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
			
			static NodeRecord searchForNode(Node& node, CustomPathFindingList& list1, CustomPathFindingList& list2){
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
	
	class CustomPathFinding{
		public:
			
			static bool pathFindAStar(Graph& graph, Node& start, Node& goal, CustomPath& path){
				NodeRecord startRecord;
				startRecord.node = start;
				startRecord.connection = Connection::None;
				startRecord.costSoFar = 0.0f;
				startRecord.estimatedTotalCost = Heuristic::estimate(start,goal);
				
				CustomPathFindingList open;
				open.push(startRecord);
				CustomPathFindingList closed;
				
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
					current = CustomPathFindingList::searchForNode(prev, open, closed);
				}
				path.addPoint(start.position);
				path.reverse();
				return true;
			}
			
			static bool pathFindDijkstra(Graph& graph, Node& start, Node& goal, CustomPath& path){
				NodeRecord startRecord;
				startRecord.node = start;
				startRecord.connection = Connection::None;
				startRecord.costSoFar = 0;
				
				CustomPathFindingList open;
				open.push(startRecord);
				CustomPathFindingList closed;
				
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
					current = CustomPathFindingList::searchForNode(prev, open, closed);
				}
				path.addPoint(start.position);
				path.reverse();
				return true;
			}
	};

	// for patrol
	
	class Patrol{
		public:
			static CustomPath path;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				if(path.isNearEnd(character.position))
					path.reverse();
				
				SteeringOutput localSteering;
				SteeringLookWhereYoureGoing::getSteering(target, character, localSteering);
				steering.linear = localSteering.linear;
				steering.angular = localSteering.angular;
				SteeringFollowCustomPath::getCustomPathSteering(target, character, localSteering, path);
				steering.linear += localSteering.linear;
				steering.angular += localSteering.angular;
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				if(steering.angular > character.maxRotation)
					steering.angular = character.maxRotation;
				
				character.setBehaviourName("PATROL");
			}
	};
	
	static CustomPath Patrol::path;
	
	void initComputerPatrol(){
		Patrol::path.addPoint(85.0f, 0.0f, 15.0f);
		Patrol::path.addPoint(85.0f, 0.0f, 35.0f);
	}
	
	void clearComputerPatrol(){
		Patrol::path.clearCustomPath();
	}
	
	class GoToPlayerFlag{
		public:
			static CustomPath path;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				SteeringOutput localSteering;
				if(path.size() == 0){
					int initNode = character.g.nodeIndex(character.position);
					int endNode = character.g.nodeIndex(playerFlagPosition);
					std::cout << initNode << ", " << endNode << std::endl;
					if(initNode != -1 && endNode != -1){
						CustomPathFinding::pathFindAStar(character.g, character.g.nodes[initNode], character.g.nodes[endNode], GoToPlayerFlag::path);
					}
					GoToPlayerFlag::path.addPoint(playerFlagPosition);
				}
				if(path.isNearEnd(character.position)){
					GoToPlayerFlag::path.clearCustomPath();
					steering.completed = true;
					return;
				}
				path.draw();
				SteeringLookWhereYoureGoing::getSteering(target, character, localSteering);
				steering.linear = localSteering.linear;
				steering.angular = localSteering.angular;
				SteeringFollowCustomPath::getCustomPathSteering(target, character, localSteering, GoToPlayerFlag::path);
				steering.linear += localSteering.linear;
				steering.angular += localSteering.angular;
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				if(steering.angular > character.maxRotation)
					steering.angular = character.maxRotation;
				
				character.setBehaviourName("GOTOPLAYERFLAG");
			}
	};
	
	CustomPath GoToPlayerFlag::path;
	
	class GoToBaseFlag{
		public:
			static CustomPath path;
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				SteeringOutput localSteering;
				if(path.size() == 0){
					int initNode = character.g.nodeIndex(character.position);
					int endNode = character.g.nodeIndex(computerFlagPosition);
					std::cout << initNode << ", " << endNode << std::endl;
					if(initNode != -1 && endNode != -1){
						CustomPathFinding::pathFindAStar(character.g, character.g.nodes[initNode], character.g.nodes[endNode], GoToBaseFlag::path);
					}
					(GoToBaseFlag::path).addPoint(computerFlagPosition);
				}
				if(path.isNearEnd(character.position)){
					GoToBaseFlag::path.clearCustomPath();
					steering.completed = true;
					std::cout << "ARRIVED" << std::endl;
					return;
				}
				path.draw();
				SteeringLookWhereYoureGoing::getSteering(target, character, localSteering);
				steering.linear = localSteering.linear;
				steering.angular = localSteering.angular;
				SteeringFollowCustomPath::getCustomPathSteering(target, character, localSteering, GoToBaseFlag::path);
				steering.linear += localSteering.linear;
				steering.angular += localSteering.angular;
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				if(steering.angular > character.maxRotation)
					steering.angular = character.maxRotation;
				
				character.setBehaviourName("GOTOBASEFLAG");
			}
	};
	
	CustomPath GoToBaseFlag::path;
	
	VantagePoints vantage_points;
	
	class Camp{
		public:
			static CustomPath path;
		
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				SteeringOutput localSteering;
				if(path.size() == 0){
					Vec3 destination = vantage_points.getClosest(character.position);
					int initNode = character.g.nodeIndex(character.position);
					int endNode = character.g.nodeIndex(destination);
					if(initNode != -1 && endNode != -1){
						CustomPathFinding::pathFindAStar(character.g, character.g.nodes[initNode], character.g.nodes[endNode], Camp::path);
						std::cout << initNode << ", " << endNode << ", path: " << path << std::endl;
					}
					(Camp::path).addPoint(destination);
				}
				if(path.isNearEnd(character.position)){
					Camp::path.clearCustomPath();
					steering.completed = true;
					std::cout << "ARRIVED" << std::endl;
					steering.linear = Vec3::zero;
					steering.angular = 0.0;
					return;
				}
				path.draw();
				SteeringLookWhereYoureGoing::getSteering(target, character, localSteering);
				steering.linear = localSteering.linear;
				steering.angular = localSteering.angular;
				SteeringFollowCustomPath::getCustomPathSteering(target, character, localSteering, Camp::path);
				steering.linear += localSteering.linear;
				steering.angular += localSteering.angular;
				if(steering.linear.length() > character.maxAcceleration){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxAcceleration;
				}
				if(steering.angular > character.maxRotation)
					steering.angular = character.maxRotation;
			
				character.setBehaviourName("CAMP");
			}
	
	};
	CustomPath Camp::path;
	
	
	class DoNothing{
		static Vec3 pos;
		public:
			static void getSteering(const Kinematic& target, Kinematic& character, SteeringOutput& steering){
				character.velocity = Vec3::zero;
				character.rotation = 0.0f;
			}
	};
	
    // ----------------------------------------------------------------------------
    // This PlugIn uses tree vehicle types: CtfAgent, CtfPlayer and CtfProyectile.  They have a
    // common base class: CtfBase which is a specialization of SimpleVehicle.


    class CtfBase : public SimpleVehicle
    {
		public:
			float hp;
			
		    // constructor
		    CtfBase () {reset ();}

		    // reset state
		    void reset (void);

		    // draw this character/vehicle into the scene
		    void draw (void);

		    void randomizeStartingPositionAndHeading (void);
			
			// Permits overwrite per vehicle to show damage in vehicle
			void damageReceived(float damage);
		     
		    // for draw method
		    Color bodyColor;
		    
		    // for steering algorithms
			Kinematic k;
    };

    class CtfAgent : public CtfBase
    {
		public:
			std::vector<void (*)(const Kinematic& , Kinematic& , SteeringOutput&)> plan;
			std::vector<cpphophtn::task> current_plan;
			time_t lastPlan;
			Graph g;
			float hp;
			
		    // constructor
		    CtfAgent (): hp(INITIAL_AGENT_STATUS){reset ();}

		    // reset state
		    void reset (void);

		    // per frame simulation update
		    void update (const float currentTime, const float elapsedTime);
	
			// draw
		    void draw (void);
		    
		    // Calculates a new plan for the agent
		    void plan_behaviour (void);
		    
		    void damageReceived(float damage);
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
			float hp;
			
		    // constructor
		    CtfPlayer (): hp(INITIAL_PLAYER_STATUS) {reset ();}

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
		    
		    void damageReceived(float damage);
		    
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
	
	class Flag{
		public:
			Vec3 position;
			void draw();
	};
	
	void Flag::draw(){
		Vec3 p2 = position;
		p2.y += 3.0f;
		Vec3 p3 = position;
		p2.y = 1.5f;
		p3.x += 1.0f;
		OpenSteer::drawTriangle(position, p2, p3, gBlack);
	}
	
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
    const Color clearCustomPathColor (0.3f, 0.6f, 0.3f); // annotation

    // count the number of times the simulation has reset (e.g. for overnight runs)
    int resetCount = 0;


    CtfAgent* ctfAgent;
    CtfPlayer* ctfPlayer;
    Flag* computerFlag;
    Flag* playerFlag;
	
	std::vector<Polygon> walls;
	
	Graph g;
	
	void initGraph(){
		std::vector< Vec3 > vertex;
		std::vector< tuple<int> > triangles;
		loadMesh(MESH_FILE, vertex, triangles);
		std::cout << "Cargado los meshes, " << triangles.size() << " poligonos!!!" << std::endl;
		for(std::vector< tuple<int> >::iterator it = triangles.begin(); it != triangles.end(); ++it){
			Polygon p(vertex[(*it).first - 1], vertex[(*it).second - 1], vertex[(*it).third - 1]);
			g.addPolygon(walls, p);
		}
	}
	
	void endGraph(){
		g.clear();
	}
	
	// loadWorld
	
	void loadWorld(){
		std::vector< Vec3 > vertex;
		std::vector< tuple<int> > triangles;
		loadMap(WORLD_FILE, vertex, triangles, vantage_points);
		std::cout << "Cargado el mundo, " << triangles.size() << " poligonos!!!" << std::endl;
		for(std::vector< tuple<int> >::iterator it = triangles.begin(); it != triangles.end(); ++it){
			Polygon p(vertex[(*it).first - 1], vertex[(*it).second - 1], vertex[(*it).third - 1]);
			walls.push_back(p);
		}
	}
	
	// clear world
	void clearWorld(){
		walls.clear();
	}
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
		setPosition(computerFlagPosition);
		k.setPosition(computerFlagPosition);
		hp = INITIAL_AGENT_STATUS;
    }


    void CtfPlayer::reset (void)
    {
        CtfBase::reset ();
        jump = false;
        jpoint.jumpLocation = Vec3::zero;
        bodyColor.set (0.6f, 0.4f, 0.4f); // redish
		setPosition(playerFlagPosition);
		k.setPosition(playerFlagPosition);
		hp = INITIAL_PLAYER_STATUS;
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
        time_t t = time(NULL);
        if(plan.size() && t - lastPlan < PLANTHRESHOLD){
	        plan[0](ctfPlayer->k, k, s);
	        if(s.completed){
	        	std::cout << "COMPLETADO PLAN" << std::endl;
	        	plan.erase(plan.begin());
	        }
	    }else{
		    plan_behaviour();
		    lastPlan = t;
	    }
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
    
    // damage received!!!
    
    void CtfBase::damageReceived(float damage){
    	std::cout << "Base called!!!" << std::endl;
    
    }
    
	void CtfAgent::damageReceived(float damage){
		CtfBase::damageReceived(damage);
		hp -= damage;
		std::cout << "damage: " << damage << std::endl;
		if(hp <= 0.0f){
			g.incrementCosts(k.position);
			reset();
		}
	}
    
	void CtfPlayer::damageReceived(float damage){
		CtfBase::damageReceived(damage);
		std::cout << "damage: " << damage << std::endl;
		hp -= damage;
		if(hp <= 0.0f){
			reset();
		}
	}
    
	// a group (STL vector) of all vehicles in the PlugIn
	std::vector<CtfBase*> all;
	std::vector<CtfAgent*> agents;
	
    bool CtfProyectile::shouldErase(){
    	// Erase when is about to hit the floor
    	if(k.position.y < 0.0){
    		for(std::vector<CtfAgent*>::iterator it = agents.begin(); it != agents.end();++it){
    			float distanceToImpact = (k.position - (*it)->k.position).length();
    			if(distanceToImpact < DAMAGE_RADIUS){
	    			std::cout << "d: " << distanceToImpact << ", r: " << DAMAGE_RADIUS << std::endl;
    				(*it)->damageReceived(distanceToImpact <= 1.0f ? PROYECTILE_DAMAGE : PROYECTILE_DAMAGE / distanceToImpact);
    			}
    		}
    		return true;
    	}
    	return false;
    }
    
    // ----------------------------------------------------------------------------
    // PlugIn for OpenSteerDemo

	bool drawGraph = false;
	
    class CtfPlugIn : public PlugIn
    {
		public:

		    const char* name (void) {return "Steering Seek";}

		    float selectionOrderSortKey (void) {return 0.01f;}

		    virtual ~CtfPlugIn() {} // be more "nice" to avoid a compiler warning

		    void open (void)
		    {
		    
				loadWorld();
		    	initCustomPath();
		    	initGraph();
		    	
		    	initBlendedBehaviours();
		    	initPriorityGroups();
				srand(time(0));
		        // create the seeker ("hero"/"attacker")
		        ctfAgent = new CtfAgent;
		        ctfAgent->setPosition(computerFlagPosition);
		        ctfAgent->k.setPosition(computerFlagPosition);
		        ctfAgent->k.g = g;
		        all.push_back (ctfAgent);
		        agents.push_back(ctfAgent);
				ctfPlayer = new CtfPlayer;
				ctfPlayer->setPosition(computerFlagPosition);
				ctfPlayer->k.setPosition(computerFlagPosition);
				all.push_back (ctfPlayer);

				playerFlag = new Flag;
				playerFlag->position = playerFlagPosition;
				
				computerFlag = new Flag;
				computerFlag->position = computerFlagPosition;
				
		        // initialize camera
		        OpenSteerDemo::init2dCamera (*ctfPlayer);
		        OpenSteerDemo::camera.mode = Camera::cmFixedDistanceOffset;
		        OpenSteerDemo::camera.fixedTarget.set (15, 0, 0);
		        OpenSteerDemo::camera.fixedPosition.set (80, 60, 0);
		        
				initComputerPatrol();
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
				
		    }

		    void redraw (const float currentTime, const float elapsedTime)
		    {
		        // selected vehicle (user can mouse click to select another)
		        AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

		        // vehicle nearest mouse (to be highlighted)
		        AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();

		        // update camera
		        OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

				// Draw graph
				if(drawGraph)
					g.draw();
				
		      	// draw walls
		      	for(std::vector<Polygon>::iterator it = walls.begin() ; it != walls.end(); ++it)
					(*it).draw();

		        // draw the all the objects and home base
		        ctfAgent->draw();

				ctfPlayer->draw();
				
				computerFlag -> draw();
				playerFlag -> draw();
				
				vantage_points.draw();
				
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
		    }

		    void close (void)
		    {
		        // delete seeker
		        delete (ctfAgent);
		        ctfAgent = NULL;
	
				delete (ctfPlayer);
				ctfPlayer = NULL;
				
				delete (computerFlag);
				computerFlag = NULL;
				
				delete (playerFlag);
				playerFlag = NULL;
				
				endBlendedBehaviours();
				endPriorityGroups();
				endCustomPath();
				endGraph();

		        // clear the group of all vehicles
		        all.clear();
		        
		        // clear the proyectiles
		        proyectiles.clear();
		        
		        // clear all the world
		        clearWorld();
		        
		        clearComputerPatrol();
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
		    		case 3:
		    			drawGraph = !drawGraph;
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
        status << resetCount << " restarts" << std::endl << "hp: " << hp << std::ends;
        const float h = drawGetWindowHeight ();
        const Vec3 screenLocation (10, h-50, 0);
        draw2dTextAt2dLocation (status, screenLocation, gGray80, drawGetWindowWidth(), drawGetWindowHeight());
    }

	// Definitions for planner
	typedef enum flag_status_t{
		FLAG_SAFE,
		FLAG_UNSAFE
	}flag_status_t;
	
	typedef enum agent_status_t{
		AGENT_HURT,
		AGENT_HEALTHY
	}agent_status_t;
	
	// Operators
	bool action_patrol(cpphophtn::state &init_state, std::map<std::string, boost::any>& parameters, cpphophtn::state &final_state){
		final_state = init_state;
		std::map<std::string, flag_status_t> flags_status = boost::any_cast<std::map<std::string, flag_status_t> >(final_state.variables["flags_status"]);
		if(flags_status["computer_flag"] == FLAG_UNSAFE){
			flags_status["computer_flag"] = FLAG_SAFE;
			return true;
		}
		return false;
	}
	
	bool action_steal_flag(cpphophtn::state &init_state, std::map<std::string, boost::any>& parameters, cpphophtn::state &final_state){
		final_state = init_state;
		std::map<std::string, flag_status_t> flags_status = boost::any_cast<std::map<std::string, flag_status_t> >(final_state.variables["flags_status"]);
		if(flags_status["player_flag"] == FLAG_SAFE)
			flags_status["player_flag"] = FLAG_UNSAFE;
		return true;
	}
	
	bool action_camp(cpphophtn::state &init_state, std::map<std::string, boost::any>& parameters, cpphophtn::state &final_state){
		final_state = init_state;
		std::map<std::string, flag_status_t> flags_status = boost::any_cast<std::map<std::string, flag_status_t> >(final_state.variables["flags_status"]);
		if(flags_status["computer_flag"] == FLAG_UNSAFE){
			flags_status["computer_flag"] = FLAG_SAFE;
			return true;
		}
		return false;
		
	}
	
	// Methods
	bool method_defend(cpphophtn::state &init_state, std::map<std::string, boost::any>& parameters, std::vector<cpphophtn::task>& tasks){
		std::map<std::string, flag_status_t> flags_status = boost::any_cast<std::map<std::string, flag_status_t> >(init_state.variables["flags_status"]);
		agent_status_t agent_status = boost::any_cast<agent_status_t>(init_state.variables["agent_status"]);
		if(flags_status["computer_flag"] == FLAG_UNSAFE){
			cpphophtn::task t;
			switch(agent_status){
				case AGENT_HEALTHY:
					t.name = "patrol";
					tasks.push_back(t);
					break;
				case AGENT_HURT:
					t.name = "camp";
					tasks.push_back(t);
					break;
			}
			return true;
		}
		return false;
	}
	
	bool method_attack(cpphophtn::state &init_state, std::map<std::string, boost::any>& parameters, std::vector<cpphophtn::task>& tasks){
		std::map<std::string, flag_status_t> flags_status = boost::any_cast<std::map<std::string, flag_status_t> >(init_state.variables["flags_status"]);
		if(flags_status["player_flag"] == FLAG_UNSAFE){
			cpphophtn::task t;
			t.name = "steal_flag";
			tasks.push_back(t);
			return true;
		}
		return false;
	}
	
	void CtfAgent::plan_behaviour (void){
		std::cout << "PLANIFICANDO" << std::endl;
		cpphophtn::state s;
		std::vector<cpphophtn::task> plan_str;
		cpphophtn::htn_operator w;
		s.name = "worldstate";
	
		std::map<std::string, flag_status_t> flags_status;
		flags_status["computer_flag"] = (ctfPlayer->k.position - computerFlagPosition).length() < SAFE_DISTANCE ? FLAG_UNSAFE : FLAG_SAFE;
		flags_status["player_flag"] = (ctfPlayer->k.position - playerFlagPosition).length() > SAFE_DISTANCE ? FLAG_UNSAFE : FLAG_SAFE;
		s.variables["flags_status"] = flags_status;
		
		agent_status_t agent_status = hp < 40.0f ? AGENT_HURT : AGENT_HEALTHY;
		
		s.variables["agent_status"] = agent_status;
		
		cpphophtn::cpphop htn;
		w.name = "patrol";
		w.action = action_patrol;
		htn.declare_operator("patrol", w);
		
		w.name = "steal_flag";
		w.action = action_steal_flag;
		htn.declare_operator("steal_flag", w);
		
		w.name = "camp";
		w.action = action_camp;
		htn.declare_operator("camp", w);
		
		cpphophtn::method m;
		std::vector<cpphophtn::method> mtds;
		m.name = "defend";
		m.method_exe = method_defend;
		mtds.push_back(m);
		m.name = "attack";
		m.method_exe = method_attack;
		mtds.push_back(m);
		htn.declare_methods("behave",  mtds);
	
		std::vector<cpphophtn::task> tasks;
		cpphophtn::task t;
	
		t.name = "behave";
		tasks.push_back(t);
		
		htn.plan(s, tasks, plan_str, 3);
		std::cout << "***PLAN: " << plan_str << std::endl;
		
		if(plan_str.size() == 0){
			plan.push_back(GoToBaseFlag::getSteering);
		}else{
			if(!cpphophtn::compare_plans(current_plan, plan_str)){
				plan.clear();
				for(std::vector<cpphophtn::task>::iterator it = plan_str.begin(); it != plan_str.end(); it++){
					if((*it).name == "patrol"){
						plan.push_back(GoToBaseFlag::getSteering);
						plan.push_back(Patrol::getSteering);
					}else if((*it).name == "steal_flag"){
						plan.push_back(GoToPlayerFlag::getSteering);
						plan.push_back(GoToBaseFlag::getSteering);
					}else if((*it).name == "camp"){
						plan.push_back(Camp::getSteering);
						plan.push_back(DoNothing::getSteering);
					}
				}
				current_plan.clear();
				for(std::vector<cpphophtn::task>::iterator it = plan_str.begin(); it != plan_str.end();++it){
					current_plan.push_back(*it);
				}
				plan_str.clear();
			}
		}
	}
} // anonymous namespace

