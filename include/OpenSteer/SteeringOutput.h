#ifndef STEERINGOUTPUT_H
#define STEERINGOUTPUT_H
	#include "Vec3.h"
	
	// All steering behaviours will answer with this class
	class SteeringOutput{
		public:
			OpenSteer::Vec3 linear;
			float angular;
			bool completed;
		
			SteeringOutput(void): linear(),angular(0.0f),completed(false) {}
			SteeringOutput(const OpenSteer::Vec3& v, float a): linear(v),angular(a) {}
	};
#endif
