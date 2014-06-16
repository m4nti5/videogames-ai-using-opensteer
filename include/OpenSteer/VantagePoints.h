#include "Vec3.h"
#include "Color.h"
	
class VantagePoints{
	public:
		std::vector<OpenSteer::Vec3> vpoints;
		VantagePoints(){}
		
		void add(const OpenSteer::Vec3 &v){
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
};

