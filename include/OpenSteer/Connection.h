#ifndef CONNECTION_H
#define CONNECTION_H
#include <iostream>
#include <iterator>

namespace OpenSteer {
	
	// Connection class (to connect nodes)
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
	
	
	inline std::ostream& operator<< (std::ostream& o, const Connection& c){
		return o << c.toNode << "(" << c.cost << ")";
	}
}
#endif
