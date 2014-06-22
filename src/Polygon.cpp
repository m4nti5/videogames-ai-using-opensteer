#include "OpenSteer/Polygon.h"

bool OpenSteer::notClipped(std::vector<OpenSteer::Polygon>& walls,OpenSteer::Polygon &p1, OpenSteer::Polygon &p2){
	OpenSteer::Vec3 c1,c2;
	p1.getCenter(c1);
	p2.getCenter(c2);
	for(std::vector<OpenSteer::Polygon>::iterator it = walls.begin(); it != walls.end();++it){
		OpenSteer::Polygon &p = (*it);
		if(p.normal.y > 0.1f)
			continue;
		OpenSteer::Vec3 u = c2 - c1;
		OpenSteer::Vec3 w = c1 - p.v0;
		
		float D = p.normal.dot(u);
		float N = -p.normal.dot(w);
		if(fabs(D) < FLT_EPSILON)
			continue;
		float sI = N/D;
		if(sI < FLT_EPSILON || sI > 1.0f)
			continue;
		OpenSteer::Vec3 intersect = c1 + sI * u;
		if(p.insidePolygonxy(intersect) && p.insidePolygon(intersect) && p.insidePolygonyz(intersect)){
			std::cout << "c1: " << c1 << ", c2: " << c2 << ", interseccion: " << intersect << ", p.v0: " << p.v0 << ", p.v1: " << p.v1 << ", p.v2: " << p.v2 << std::endl;
			return false;
		}
	}
	return true;
}

