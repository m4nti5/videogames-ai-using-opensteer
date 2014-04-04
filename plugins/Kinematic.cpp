#include <stdio.h>
#include <cmath> 
#include <iomanip>
#include <string>
#include <GL/gl.h>
#include <GL/glu.h>
#include <sstream>
#include <cstdlib>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Color.h"

namespace{
    using namespace OpenSteer;
    
	class SteeringOutput{
		public:
			Vec3 linear;
			double angular;
		SteeringOutput(void): linear(),angular(0.0f) {}
		SteeringOutput(const Vec3& v, double a): linear(v),angular(a) {}
	};
	
	class Kinematic{
		public:
			// Maximum velocity for ALL vehicles
			const float maxSpeed = 3.0f;
			// Maximum turn for wander steering
			const float maxRotation = 100.0f;
			// vehicle position
			Vec3 position;
			// radian value of orientation, measured from the z coordinate counter-clockwise
			float orientation;
			// Speed vector for linear velocity
			Vec3 velocity;
			// Angular speed Value(radian per seconds, to avoid 3rd Newtons law violations)
			float rotation;
			char keymap[28];
			
			Kinematic(void): position(), orientation(0.0f), velocity(), rotation (0.0f) {}
			
        	Kinematic(const Vec3& pos, double o, const Vec3& v, double r) : position(pos), orientation(o), velocity(v), rotation(r) {}
			
			void setPosition(const Vec3& p){
				position = p;
			}
			
			void setSpeed(const Vec3& v){
				velocity = v;
			}
			
			void update(const SteeringOutput& s,float time){
				Vec3 linear_acc = s.linear;
				linear_acc -= velocity;
				position += (velocity * time) + (0.5 * linear_acc * time * time);
				float angular_acc = s.angular;
				angular_acc -= rotation;
				orientation += (rotation * time) + (0.5 * angular_acc * time * time);
				velocity += linear_acc * time;
				rotation += angular_acc * time;
			}

			void updateNewtonEuler(const SteeringOutput& s,float time){
				position += velocity * time;
				orientation += rotation * time;
				velocity += s.linear * time;
				rotation += s.angular * time;
				if(velocity.length() > maxSpeed)
					velocity = velocity.normalize() * maxSpeed;
			}
			
			static double getNewOrientation(double currentOrientation, const Vec3& velocity){
				if(velocity.length()> 0)
					return atan2(velocity.x,velocity.z);
				else
					return currentOrientation;
			}
			
			static void orientationAsAVector(float orientation, Vec3& output){
				output.x = sin(orientation);
				output.z = cos(orientation);
			}
	};
	
	class KinematicSeek{
		public: 
			static void getSteering(Kinematic& target, Kinematic& character, SteeringOutput& steering){
				steering.linear = target.position - character.position;
				Vec3 normalized;
				normalized = steering.linear.normalize();
				steering.linear = normalized;
				steering.linear *= character.maxSpeed;
				character.orientation = Kinematic::getNewOrientation(character.orientation,steering.linear);
				steering.angular = 0;
				
				// select string describing current seeker state
				std::string seekerStateString("KINEMATICSEEK");
				
				// annote seeker with its state as text
				const Vec3 textOrigin = character.position + Vec3 (0, 0.25, 0);
				std::ostringstream annote;
				annote << seekerStateString << std::endl;
				draw2dTextAt3dLocation (annote, textOrigin, gWhite, drawGetWindowWidth(), drawGetWindowHeight());
			}
	};
	
	class KinematicFlee{
		public: 
			static void getSteering(Kinematic& target, Kinematic& character, SteeringOutput& steering){
				// It's a flee, so, go the other way!
				steering.linear = character.position - target.position;
				Vec3 normalized;
				normalized = steering.linear.normalize();
				steering.linear = normalized;
				steering.linear *= character.maxSpeed;
				character.orientation = Kinematic::getNewOrientation(character.orientation,steering.linear);
				steering.angular = 0;
				
				// select string describing current seeker state
				std::string seekerStateString("KINEMATICFLEE");
				
				// annote seeker with its state as text
				const Vec3 textOrigin = character.position + Vec3 (0, 0.25, 0);
				std::ostringstream annote;
				annote << seekerStateString << std::endl;
				draw2dTextAt3dLocation (annote, textOrigin, gWhite, drawGetWindowWidth(), drawGetWindowHeight());
			}
	};
	
	class KinematicArrive{
		public: 
		
			// Satisfaction radius
			static const float radius;
			
			// Constant time to target
			static const float timeToTarget;
			
			static void getSteering(Kinematic& target, Kinematic& character, SteeringOutput& steering){
				steering.linear = target.position - character.position;
				if(steering.linear.length() < radius){
					steering.linear = Vec3::zero;
					steering.angular = character.rotation;
					return;
				}
				// Magic to appear deacceleration
				steering.linear /= timeToTarget;
				if(steering.linear.length() > character.maxSpeed){
					steering.linear = steering.linear.normalize();
					steering.linear *= character.maxSpeed;
				}
				character.orientation = Kinematic::getNewOrientation(character.orientation,steering.linear);
				steering.angular = 0;
				
				// select string describing current seeker state
				std::string seekerStateString("KINEMATICARRIVE");
				
				// annote seeker with its state as text
				const Vec3 textOrigin = character.position + Vec3 (0, 0.25, 0);
				std::ostringstream annote;
				annote << seekerStateString << std::endl;
				draw2dTextAt3dLocation (annote, textOrigin, gWhite, drawGetWindowWidth(), drawGetWindowHeight());
			}
	};
	
	const float KinematicArrive::radius = 1.0f;
	
	const float KinematicArrive::timeToTarget = 3.0f;
	
	class KinematicWander{
		static int wanderTime;
		static int wanderTimer;
		public: 
			static void getSteering(Kinematic& target, Kinematic& character, SteeringOutput& steering){
				Vec3 vec_orientation (sin(character.orientation),0,cos(character.orientation));
				steering.linear = character.maxSpeed * vec_orientation;
				wanderTimer++;
				int randomTime = 1 + (int)(randomBinomial() * 10);
				if(randomTime % 3 == 0){
					float r = randomBinomial();
					steering.angular = r * character.maxRotation;
					wanderTimer = 0;
				}
				
				// select string describing current seeker state
				std::string seekerStateString("KINEMATICWANDER");
				
				// annote seeker with its state as text
				const Vec3 textOrigin = character.position + Vec3 (0, 0.25, 0);
				std::ostringstream annote;
				annote << seekerStateString << std::endl;
				draw2dTextAt3dLocation (annote, textOrigin, gWhite, drawGetWindowWidth(), drawGetWindowHeight());
			}
			
		private:
			static float random01(){
				double i = 0, d = 0;
				i = rand() % 4000 - 2000; //Gives a number between -2000 and +2000;
				d = i / 10000; //Reduces this number to the range you want.
				return d;
			}
			static float randomBinomial(){
				return (random01()) - (random01());
		}
		
	};
	
	int KinematicWander::wanderTime = 100;
	int KinematicWander::wanderTimer = 0;
	
	inline std::ostream& operator<< (std::ostream& o, const SteeringOutput& s){
		return o << "linear vel: " << s.linear << std::endl << "angular vel: " << s.angular << std::endl;
	}
	
	inline std::ostream& operator<< (std::ostream& o, const Kinematic& k){
		return o << "position: " << k.position << std::endl << "orientation: " << k.orientation << std::endl << "velocity: " << k.velocity << std::endl << "rotation: " << k.rotation << std::endl;
	}
    // ----------------------------------------------------------------------------
    // This PlugIn uses two vehicle types: CtfSeeker and CtfPlayer.  They have a
    // common base class: CtfBase which is a specialization of SimpleVehicle.


    class CtfBase : public SimpleVehicle
    {
    public:
    	
        // constructor
        CtfBase () {reset ();}

        // reset state
        void reset (void);

        // draw this character/vehicle into the scene
        void draw (void);

        void randomizeStartingPositionAndHeading (void);
         
        // for draw method
        Color bodyColor;
        // for Kinematic seek movement
    	Kinematic k;
    };

    class CtfSeeker : public CtfBase
    {
    public:

        // constructor
        CtfSeeker () {reset ();}

        // reset state
        void reset (void);

        // per frame simulation update
        void update (const float currentTime, const float elapsedTime);

        void draw (void);
        float lastRunningTime; // for auto-reset
    };


    class CtfPlayer : public CtfBase
    {
    public:

        // constructor
        CtfPlayer () {reset ();}

        // reset state
        void reset (void);

        // per frame simulation update
        void update (const float currentTime, const float elapsedTime);
        
        // draw
        void draw (void);
    };


    // ----------------------------------------------------------------------------
    // globals
    // (perhaps these should be member variables of a Vehicle or PlugIn class)


    const Vec3 gHomeBaseCenter (0, 0, 0);
    const float gHomeBaseRadius = 1.5;

    const float gMinStartRadius = 30;
    const float gMaxStartRadius = 40;

    const float gBrakingRate = 0.75;

    const Color evadeColor     (0.6f, 0.6f, 0.3f); // annotation
    const Color seekColor      (0.3f, 0.6f, 0.6f); // annotation
    const Color clearPathColor (0.3f, 0.6f, 0.3f); // annotation


    CtfSeeker* gSeeker = NULL;

    // count the number of times the simulation has reset (e.g. for overnight runs)
    int resetCount = 0;


    CtfSeeker* ctfSeeker;
    CtfPlayer* ctfPlayer;


    // ----------------------------------------------------------------------------
    // reset state


    void CtfBase::reset (void)
    {
        SimpleVehicle::reset ();  // reset the vehicle 

        randomizeStartingPositionAndHeading ();  // new starting position

        clearTrailHistory ();     // prevent long streaks due to teleportation
    }


    void CtfSeeker::reset (void)
    {
        CtfBase::reset ();
        bodyColor.set (0.4f, 0.4f, 0.6f); // blueish
        gSeeker = this;
    }


    void CtfPlayer::reset (void)
    {
        CtfBase::reset ();
        bodyColor.set (0.6f, 0.4f, 0.4f); // redish
    }


    // ----------------------------------------------------------------------------
    // draw this character/vehicle into the scene


    void CtfBase::draw (void)
    {
    	setPosition(k.position);
    	setSpeed(k.velocity.length());
        drawBasic2dCircularVehicle (*this, bodyColor);
        drawTrail ();
    }

    void CtfPlayer::draw (void)
    {
    	setPosition(k.position);
    	setSpeed(k.velocity.length());
        drawBasic2dCircularVehicle (*this, bodyColor);
        drawTrail ();
    }
    
    // ----------------------------------------------------------------------------


    void CtfBase::randomizeStartingPositionAndHeading (void)
    {
        // randomize position on a ring between inner and outer radii
        // centered around the home base
        const float rRadius = frandom2 (gMinStartRadius, gMaxStartRadius);
        const Vec3 randomOnRing = RandomUnitVectorOnXZPlane () * rRadius;
        setPosition (gHomeBaseCenter + randomOnRing);
        // randomize 2D heading
        randomizeHeadingOnXZPlane ();
    }

    void CtfPlayer::update (const float currentTime, const float elapsedTime){
    	SteeringOutput s;
    	// Vec3 direction  = k.velocity;
    	
    	Vec3 direction;
    	Kinematic::orientationAsAVector(k.orientation, direction);
    	Vec3 left (direction.z,0.0f,-direction.x);
    	if(direction == Vec3::zero)
    		left.x = -1.0f,left.z = 0.0f;
    	// Move forward
		if (k.keymap['w' - 'a']){
			s.linear = (s.linear + direction) * 10.0f;
			k.keymap['w' - 'a'] = 0;
		}
		// Move backward
		if (k.keymap['s' - 'a']){
			s.linear = (s.linear - direction) * 10.0f;
			k.keymap['s' - 'a'] = 0;
		}
		// Strafe right
		if (k.keymap['d' - 'a']){
			s.linear = (s.linear - left) * 10.0f;
			k.keymap['d' - 'a'] = 0;
		}
		// Strafe left
		if (k.keymap['a' - 'a']){
			s.linear = (s.linear + left) * 10.0f;
			k.keymap['a' - 'a'] = 0;
		}
		
		s.linear = s.linear.normalize();
		s.linear = s.linear * 10.0f;
		k.updateNewtonEuler(s,elapsedTime);
		
		// Fix orientation
		k.orientation = Kinematic::getNewOrientation(k.orientation,k.velocity);
    	if(k.velocity != Vec3::zero)
    		regenerateOrthonormalBasisUF(k.velocity.normalize());
    	
		// annotation
		recordTrailVertex (currentTime, k.position);
    }

    void CtfSeeker::draw (void){
        // first call the draw method in the base class
        CtfBase::draw();

        // display status in the upper left corner of the window
        std::ostringstream status;
        status << resetCount << " restarts" << std::ends;
        const float h = drawGetWindowHeight ();
        const Vec3 screenLocation (10, h-50, 0);
        draw2dTextAt2dLocation (status, screenLocation, gGray80, drawGetWindowWidth(), drawGetWindowHeight());
    }

	// Used for behaviours changes with F1/F2
	typedef  void *steerFunc(Kinematic& , Kinematic& , SteeringOutput&);
	std::vector<void (*)(Kinematic& , Kinematic& , SteeringOutput&)> steerFunctions;
	static int steerFuncNum = 0;

    
    // ----------------------------------------------------------------------------
    // update method for goal seeker
    
    void CtfSeeker::update (const float currentTime, const float elapsedTime)
    {
        SteeringOutput s;
        Vec3 dist = k.position - ctfPlayer->k.position;
        steerFunc *getSteering = (steerFunc *)steerFunctions[steerFuncNum];
        (*getSteering)(ctfPlayer->k,k,s);
        
	    if(s.linear == Vec3::zero){
	    	s.linear = k.velocity;
	    	s.angular = k.rotation;
	    }
		k.update(s,elapsedTime);
		
		// Fix orientation
    	Vec3 orientation (sin(k.orientation),0.0f,cos(k.orientation));
    	regenerateOrthonormalBasisUF(orientation);
    	
		// annotation
		recordTrailVertex (currentTime, k.position);
    }
    
    // ----------------------------------------------------------------------------
    // PlugIn for OpenSteerDemo


    class CtfPlugIn : public PlugIn
    {
    public:

        const char* name (void) {return "Kinematic Seek";}

        float selectionOrderSortKey (void) {return 0.01f;}

        virtual ~CtfPlugIn() {} // be more "nice" to avoid a compiler warning

        void open (void)
        {
			srand(time(0));
            // create the seeker ("hero"/"attacker")
            ctfSeeker = new CtfSeeker;
            Vec3 pos1 (1,0,1);
            Vec3 pos2 (5,0,5);
            ctfSeeker->setPosition(pos1);
            ctfSeeker->k.setPosition(pos1);
            all.push_back (ctfSeeker);
			ctfPlayer = new CtfPlayer;
			ctfPlayer->setPosition(pos2);
			ctfPlayer->k.setPosition(pos2);
			all.push_back (ctfPlayer);
			
			// Add steers to vector
	    	steerFunctions.push_back(KinematicSeek::getSteering);
	    	steerFunctions.push_back(KinematicFlee::getSteering);
	    	steerFunctions.push_back(KinematicArrive::getSteering);
	    	steerFunctions.push_back(KinematicWander::getSteering);

            // initialize camera
            OpenSteerDemo::init2dCamera (*ctfPlayer);
            OpenSteerDemo::camera.mode = Camera::cmFixedDistanceOffset;
            OpenSteerDemo::camera.fixedTarget.set (15, 0, 0);
            OpenSteerDemo::camera.fixedPosition.set (80, 60, 0);

        }

        void update (const float currentTime, const float elapsedTime)
        {
            // update the seeker
            ctfSeeker->update (currentTime, elapsedTime);
          	ctfPlayer->update (currentTime, elapsedTime);
        }

        void redraw (const float currentTime, const float elapsedTime)
        {
            // selected vehicle (user can mouse click to select another)
            AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

            // vehicle nearest mouse (to be highlighted)
            AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();

            // update camera
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

            // draw "ground plane" centered between base and selected vehicle
            const Vec3 goalOffset = gHomeBaseCenter-OpenSteerDemo::camera.position();
            const Vec3 goalDirection = goalOffset.normalize ();
            const Vec3 cameraForward = OpenSteerDemo::camera.xxxls().forward();
            const float goalDot = cameraForward.dot (goalDirection);
            const float blend = remapIntervalClip (goalDot, 1, 0, 0.5, 0);
            const Vec3 gridCenter = interpolate (blend,
                                                 selected.position(),
                                                 gHomeBaseCenter);
            OpenSteerDemo::gridUtility (gridCenter);

            // draw the seeker, obstacles and home base
            ctfSeeker->draw();

			ctfPlayer->draw();

            // highlight vehicle nearest mouse
            OpenSteerDemo::highlightVehicleUtility (nearMouse);
        }

        void close (void)
        {
            // delete seeker
            delete (ctfSeeker);
            ctfSeeker = NULL;
	
			delete (ctfPlayer);
			ctfPlayer = NULL;

            // clear the group of all vehicles
            all.clear();
        }

        void reset (void)
        {
            // count resets
            resetCount++;

            // reset the seeker ("hero"/"attacker") and enemies
            ctfSeeker->reset ();
            ctfPlayer->reset ();

            // reset camera position
            OpenSteerDemo::position2dCamera (*ctfSeeker);

            // make camera jump immediately to new position
            OpenSteerDemo::camera.doNotSmoothNextMove ();
        }

        void handleFunctionKeys (int keyNumber){
        	int n = steerFunctions.size();
	    	switch(keyNumber){
	    		case 1:
	    			steerFuncNum = (steerFuncNum - 1) % n;
	    			if(steerFuncNum < 0)
	    				steerFuncNum = n - 1;
	    			break;
	    		case 2:
	    			steerFuncNum = (steerFuncNum + 1) % n;
	    			break;
	    	}
        }

		void handleKeyboardKeys(unsigned char key, int x, int y){
		    ctfPlayer->k.keymap[key - 'a'] = 1;
		}
		
        void printMiniHelpForFunctionKeys (void)
        {
            std::ostringstream message;
            message << "Function keys handled by ";
            message << '"' << name() << '"' << ':' << std::ends;
            OpenSteerDemo::printMessage (message);
            OpenSteerDemo::printMessage ("");
        }

        const AVGroup& allVehicles (void) {return (const AVGroup&) all;}

        // a group (STL vector) of all vehicles in the PlugIn
        std::vector<CtfBase*> all;
        
    };


    CtfPlugIn gCtfPlugIn;


    // ----------------------------------------------------------------------------


} // anonymous namespace

