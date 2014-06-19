#include "Vec3.h"
#include "Color.h"
	
class VantagePoints{
	public:
		std::vector<OpenSteer::Vec3> vpoints;
		VantagePoints(){}
		
		void add(const OpenSteer::Vec3 &v){
			std::cout << "Agregando punto de ventaja " << v << std::endl;
			vpoints.push_back(v);
		}
		
		void clear(){
			vpoints.clear();
		}
		
		void draw(){
			for(std::vector<OpenSteer::Vec3>::iterator it = vpoints.begin(); it != vpoints.end();++it){
			    drawXZCircle (0.3, *it, OpenSteer::gBlue, 10);
            }
		}
		
		OpenSteer::Vec3 getClosest(const OpenSteer::Vec3 &src){
			OpenSteer::Vec3 closest = vpoints[0];
			for(std::vector<OpenSteer::Vec3>::iterator it = vpoints.begin() + 1; it != vpoints.end();++it){
				float dist = (src - (*it)).length();
				if(dist < (src - closest).length()){
					closest = *it;
				}
            }
            return closest;
		}
};

