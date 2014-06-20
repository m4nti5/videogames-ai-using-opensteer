#ifndef NODE_H
#define NODE_H
#include <iostream>
#include <iterator>
#include "Polygon.h"
#include "Connection.h"

namespace OpenSteer{
	class Node{
		public:
			int id;
			Polygon p;
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
				this->p = n.p;
				return *this;
			}
			
			bool insideNode(const Vec3& point){
				return p.insidePolygon(point);
			}
			
			inline bool operator== (const Node& n) const {return this->id == n.id;}
			
			inline bool operator!= (const Node& n) const {return (!(*this == n));}
	
	};
	
	inline std::ostream& operator<< (std::ostream& o, const Node& n){
		o << n.id << "(" << n.adyacents.size() << ")" << ": ";
		copy(n.adyacents.begin(), n.adyacents.end(), std::ostream_iterator<Connection>(o, " "));
		return o << std::endl;
	}
}
#endif
