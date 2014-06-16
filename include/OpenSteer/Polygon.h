#ifndef POLYGON_H
#define POLYGON_H
	#include "Color.h"
	#include "Vec3.h"
	#include "Draw.h"
	// Represents a polygon in the game
	class Polygon{
		public:
			OpenSteer::Vec3 v0, v1, v2, normal;
			Polygon(){}
			Polygon(const OpenSteer::Vec3& v_0, const OpenSteer::Vec3& v_1, const OpenSteer::Vec3& v_2):v0(v_0), v1(v_1), v2(v_2){
				calculateNormal();
			}
	
			void setVertex(const OpenSteer::Vec3& v_0, const OpenSteer::Vec3& v_1, const OpenSteer::Vec3& v_2){
				v0 = v_0, v1 = v_1, v2 = v_2;
				calculateNormal();
			}
		
			void draw(){
				OpenSteer::Color color;
				if(normal.y > 0.1f)
					color.setR(.5f), color.setG(.5f), color.setB(.5f);
				else
					color.setR(0.6f), color.setG(0.6f), color.setB(0.0f);
				
				OpenSteer::Vec3 v_0(v0),v_1(v1),v_2(v2);
				v_0.y - 1.0f;
				v_1.y - 1.0f;
				v_2.y - 1.0f;
				OpenSteer::drawTriangle(v_0, v_1, v_2, color);
			}
		
			void drawMesh(){
				OpenSteer::Vec3 v_0(v0),v_1(v1),v_2(v2);
				v_0.y - 0.2;
				v_1.y - 0.2;
				v_2.y - 0.2;
				OpenSteer::drawLine(v_0, v_1, OpenSteer::gOrange);
				OpenSteer::drawLine(v_1, v_2, OpenSteer::gOrange);
				OpenSteer::drawLine(v_2, v_0, OpenSteer::gOrange);
			}
		
			bool segmentCheck(const OpenSteer::Vec3 &v0, const OpenSteer::Vec3 &v1, const Polygon &p){
				if((p.v0 == v0 && p.v1 == v1) || (p.v0 == v1 && p.v1 == v0))
					return true;
				if((p.v1 == v0 && p.v2 == v1) || (p.v1 == v1 && p.v2 == v0))
					return true;
				if((p.v2 == v0 && p.v0 == v1) || (p.v2 == v1 && p.v0 == v0))
					return true;
				return false;
			}
		
			bool isConnected(const Polygon& p){
				if(p.v0 == v0 || p.v0 == v1 || p.v0 == v2)
					return true;
				if(p.v1 == v0 || p.v1 == v1 || p.v1 == v2)
					return true;
				if(p.v2 == v0 || p.v2 == v1 || p.v2 == v2)
					return true;
				
				/*
				if(segmentCheck(v0, v1, p))
					return true;
				if(segmentCheck(v1, v2, p))
					return true;
				if(segmentCheck(v2, v0, p))
					return true;
			
				*/
				return false;
			}
		
			void getCenter(OpenSteer::Vec3& center){
				center.x = (v0.x + v1.x + v2.x) / 3;
				center.y = (v0.y + v1.y + v2.y) / 3;
				center.z = (v0.z + v1.z + v2.z) / 3;
			}
		
			inline Polygon operator= (const Polygon& p) {
				this->v0 = p.v0;
				this->v1 = p.v1;
				this->v2 = p.v2;
				this->normal = p.normal;
				return *this;
			}
		
			bool insidePolygon(const OpenSteer::Vec3 &pt){
				bool b1, b2, b3;

				b1 = sign(pt, v0, v1) < 0.0f;
				b2 = sign(pt, v1, v2) < 0.0f;
				b3 = sign(pt, v2, v0) < 0.0f;

				return ((b1 == b2) && (b2 == b3));
			}
		
			bool insidePolygonxy(const OpenSteer::Vec3 &pt){
				bool b1, b2, b3;

				b1 = signxy(pt, v0, v1) < 0.0f;
				b2 = signxy(pt, v1, v2) < 0.0f;
				b3 = signxy(pt, v2, v0) < 0.0f;

				return ((b1 == b2) && (b2 == b3));
			}
		
			bool insidePolygonyz(const OpenSteer::Vec3 &pt){
				bool b1, b2, b3;

				b1 = signyz(pt, v0, v1) < 0.0f;
				b2 = signyz(pt, v1, v2) < 0.0f;
				b3 = signyz(pt, v2, v0) < 0.0f;

				return ((b1 == b2) && (b2 == b3));
		
			}
		private:
			void calculateNormal(){
				OpenSteer::Vec3 e0 = v1 - v0;
				OpenSteer::Vec3 e1 = v2 - v0;
				normal = crossProduct(e0, e1);
				normal = normal.normalize();
			}
		
			float sign(const OpenSteer::Vec3 &p1, const OpenSteer::Vec3 &p2, const OpenSteer::Vec3 &p3){
				return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
			}
		
			float signxy(const OpenSteer::Vec3 &p1, const OpenSteer::Vec3 &p2, const OpenSteer::Vec3 &p3){
				return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
			}
		
			float signyz(const OpenSteer::Vec3 &p1, const OpenSteer::Vec3 &p2, const OpenSteer::Vec3 &p3){
				return (p1.y - p3.y) * (p2.z - p3.z) - (p2.y - p3.y) * (p1.z - p3.z);
			}
	};
#endif

