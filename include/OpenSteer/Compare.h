#ifndef COMPARE_H
#define COMPARE_H
	#include <float.h>
	// For float comparision
	class Compare{
		public:
	
			static bool nearlyEqual(float a, float b, float epsilon) {
				float absA = abs(a);
				float absB = abs(b);
				float diff = abs(a - b);

				if (a == b) { // shortcut, handles infinities
					return true;
				} else if (a == 0 || b == 0 || diff < FLT_MIN) {
					// a or b is zero or both are extremely close to it
					// relative error is less meaningful here
					return diff < (epsilon * FLT_MIN);
				} else { // use relative error
					return diff / (absA + absB) < FLT_EPSILON;
				}
			}
	};
#endif

