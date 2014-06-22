#ifndef CUSTOM_PATH_H
#define CUSTOM_PATH_H
	#include <vector>
	#include <algorithm>
	#include "Vec3.h"
	#include "Color.h"
	#include "Draw.h"

	// CustomPath class
	namespace OpenSteer {
		class CustomPath{
			public:
				std::vector<Vec3> points;
				float totalCost;
				
				CustomPath():totalCost(0.0f){}
		
				void addPoint(const Vec3 &point){
					points.push_back(point);
				}
		
				void addPoint(float x, float y, float z){
					Vec3 p(x,y,z);
					points.push_back(p);
				}
		
				void draw(){
					int i,n = points.size();
					for(i = 0;i < n - 1; i++){
						Vec3 a = points[i];
						Vec3 b = points[i + 1];
						drawLine(a, b, gWhite);				
					}
				}
		
				void clearCustomPath(){
					points.clear();
					totalCost = 0.0f;
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
					drawLine(p, closestPoint, gYellow);	
				}
		
				bool isNearEnd(const Vec3& p){
					if(points.size() == 0)
						return true;
					Vec3 lastPoint = points[points.size() - 1];
					if((p - lastPoint).length() < 0.5f)
						return true;
					return false;
				}
		
				bool isNearBeginning(const Vec3& p){
					Vec3 firstPoint = points[0];
					if((p - firstPoint).length() < 0.5f)
						return true;
					return false;
				}
		
				void getPosition(Vec3 &point, int segment, float pathOffset, Vec3& resultPoint){
					Vec3 a,b;
					int n = points.size();
					a = points[segment];
					b = points[segment + 1];
					resultPoint = point + (b - a).normalize() * pathOffset;
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
		
				int size(){
					return points.size();
				}
		
				std::ostream& print(std::ostream &o) const{
					for(std::vector<Vec3>::const_iterator it = points.begin(); it != points.end(); ++it)
						o << (*it) << ", ";
					o << std::endl;
					return o;
				}
		
				inline Vec3& operator[] (std::size_t idx){
					return points[idx];
				}
				
				inline CustomPath operator=(const CustomPath &p){
					totalCost = p.totalCost;
					points.clear();
					for(std::vector<Vec3>::const_iterator it = p.points.begin(); it != p.points.end(); ++it){
						points.push_back(*it);
					}
					return *this;
				}
		
		};
		inline std::ostream& operator<< (std::ostream& o, const CustomPath& p){
			return p.print(o);
		}
	}
	
	
#endif

