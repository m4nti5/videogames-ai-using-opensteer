#ifndef GRAPH_H
#define GRAPH_H
#include <iostream>
#include <iterator>
#include "Vec3.h"
#include "Node.h"

namespace OpenSteer {
	class Graph{
		public:
			std::vector<Node> nodes;
			int lastid;
			float maxIncrement;
			float damageIncrement;
			
			Graph(void):lastid(0),damageIncrement(5.0f),maxIncrement(1000.0f){}
			
			int addNode(const Vec3& position){
				int node_id = lastid++;
				Node n (node_id, position);
				nodes.push_back(n);
				return node_id;
			}
			
			void addPolygon(std::vector<Polygon>& walls, Polygon& p){
				int node_id = lastid++;
				float cost;
				Vec3 position;
				p.getCenter(position);
				Node n (node_id, position);
				for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end();++it){
					if((*it).p.isConnected(p) && notClipped(walls, (*it).p, p)){
					//if((*it).p.isConnected(p)){
						cost = ((*it).position - position).length();
						(*it).addConnection(node_id, cost);
						n.addConnection((*it).id, cost);
					}
				}
				n.p = p;
				nodes.push_back(n);
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
					(*n).p.drawMesh();
					std::ostringstream node_id;
					node_id << (*n).id << std::endl;
					draw2dTextAt3dLocation (node_id, p1, gWhite, drawGetWindowWidth(), drawGetWindowHeight());

					for(std::vector<Connection>::iterator c = (*n).adyacents.begin(); c!= (*n).adyacents.end();++c){
						Vec3 p2 = nodes[(*c).toNode].position;
						OpenSteer::drawLine(p1, p2, gRed);
						
						Vec3 tpos = p2 - p1;
						float l = tpos.length();
						const Vec3 textOrigin = p1 + tpos.normalize() * l/2 + Vec3 (0, 0.25, 0);
						std::ostringstream annote;
						annote << (*c).cost << std::endl;
						draw2dTextAt3dLocation (annote, textOrigin, gOrange, drawGetWindowWidth(), drawGetWindowHeight());
					}
				}
			}
			
			int nodeIndex(const Vec3 &pt){
				int n = nodes.size();
				for(int i = 0; i < n; i++){
					if(nodes[i].p.insidePolygon(pt)){
						return i;
					}
				}
				return -1;
			}
			
			inline Graph operator= (const Graph& g) {
				this->nodes = g.nodes;
				this->lastid = g.lastid;
				return *this;
			}
			
			void incrementCosts(const Vec3& p){
				for(std::vector<Node>::iterator n = nodes.begin();n != nodes.end();++n){
					if((*n).insideNode(p)){
						for(std::vector<Connection>::iterator it = (*n).adyacents.begin(); it != (*n).adyacents.end(); ++it){
							Node &to = nodes[(*it).toNode];
							Node &from = nodes[(*it).fromNode];
							if(((*it).cost - (to.position - from.position).length()) < maxIncrement){
								(*it).cost += damageIncrement;
							}
						}
					}
				}
			}
	};
	
	inline std::ostream& operator<< (std::ostream& o, const Graph& s){
		copy(s.nodes.begin(), s.nodes.end(), std::ostream_iterator<Node>(o, ""));
		return o;
	}
}
#endif
	
