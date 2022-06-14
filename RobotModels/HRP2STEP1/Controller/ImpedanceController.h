#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/Sensor>
#include <cnoid/EigenUtil>

#include <cnoid/JointPath>
#include <fstream>
#include <Eigen/Core>


/********************************* DEFINES *********************************************/
#define Vector6d Matrix<double,6,1>

#define ARM_DOF   6
#define STR_LEN 256


#define CHEST_JOINT0 0

#define HEAD_JOINT0 1
#define HEAD_JOINT1 2

#define RARM_JOINT0 3
#define RARM_JOINT1 4
#define RARM_JOINT2 5
#define RARM_JOINT3 6
#define RARM_JOINT4 7
#define RARM_JOINT5 8

#define LARM_JOINT0 9
#define LARM_JOINT1 10
#define LARM_JOINT2 11
#define LARM_JOINT3 12
#define LARM_JOINT4 13
#define LARM_JOINT5 14


using namespace std;
using namespace cnoid;
using namespace Eigen;


//-------------------------------------------------------------------------------------------
class HRP2STEP1Arm
{
  friend class ImpedanceController;

 public:

  /******************************* ENUMERATIONS **************************************/
  enum interpolation
  {
    CONSTANT_VELOCITY,
    QUINTIC_FUNCTION,
    // append new method hereinafter
    END_OF_INTERPOLATION
  };

  /****************************** PATH GENERATION PARAMETERS ***************************/
  struct path_params
  {
    interpolation method;						// Interpolation method. See enumeration above.
    double        	max_jvel;				// Joint velocity limit
    Vector6d 		q_start;				// Starting joint angle configuration
    Vector6d 		q_goal;					// Ending joint angle configuration
    Vector6d 		distance;				// Distance measure
    double 		duration;				// Desired Duration
    Vector6d 		jvel;					// Joint velocity
    //Vector6d duration;
    //double max_duration;
    //std::vector<object_state> route
  };

  /*****************************  Control Mode **************************************/
  enum cmode
  {
    NOCONTROL,
    VEL_CONTROL,
    IMP_CONTROL
  };

  enum
  {
    X,
    Y,
    Z
  };

  /********************************************** METHODS *********************************************/

//  HRP2STEP1Arm(std::string  name_in, 							// String variable indicating right or left arm
//	  BodyPtr      body_in, 					// Body (link objects) Pointer
//	  unsigned int num_q0_in, 					// Number of Starting joint
//	  double 		 period_in, 				// Sampling Time
//	  float 		 ang_limits_in[6][5], 			// Angle Limits
//	  Vector3 	 ePh_in, 					// Tranlation from wrist to end effector
//	  Matrix3 	 eRh_in, 					// Rotation   from wrist to end effector
//	  Vector3 	 hPfs_in);					//
  HRP2STEP1Arm(std::string  name_in, 							// String variable indicating right or left arm
	  BodyPtr      body_in, 					// Body (link objects) Pointer
	  unsigned int num_q0_in, 					// Number of Starting joint
	  double 		 period_in, 				// Sampling Time
	  float 		 ang_limits_in[6][5], 			// Angle Limits
	  Vector3 	 ePh_in, 					// Tranlation from wrist to end effector
	  Matrix3 	 eRh_in, 					// Rotation   from wrist to end effector
	  Vector3 	 hPfs_in,					//
	  Matrix3 	 hRfs_in);					//	FUKUMOTO added (2014.12.24)

  virtual ~HRP2STEP1Arm();
#if 0
  void init();
  int init(Vector3 pos, Matrix3 rot, double CurAngles[15]);	// Initialize
  virtual void savedata();									// Save Data

  // Data retrieval methods
  std::string get_name();
#endif
  // Cartesian position and orientation and Current Joint Angles
  void update_currposdata();							// Update the base2endeffector position
  void wrist2EndEffXform(/*in*/Vector3 rPe, /*in*/Matrix3 rRe, /*out*/Vector3& rPh, /*out*/Matrix3& rRh);
  void EndEff2wristXform(/*in*/Vector3 rPh, /*in*/Matrix3 rRh, /*out*/Vector3& rPe);

  // Joint Position
  Vector6d get_qref();							// Get joint angle references
//  Vector6d get_qcur();							// Get current joint angles

  // Forces
  //void set_FSptr(nittaFS *fs_in, int NO_fs_in);	                       // Set force sensor pointer from nittaFs clas
  virtual void update_currforcedata();
//virtual bool get_forces(Vector3 &rFh_gc_out, 				                                // Force
//			  Vector3 &rMh_gc_out);		                                                                // Moment
#if 0
  // Hand position
  void get_curr_handpos(Vector3 &rPh_out, 					                // Get current base2endeffector position
			Matrix3 &rRh_out);		                                                                // Rotation

  void get_ref_handpos(Vector3 &rPh_ref_out, 					                // Get desired base2endeffector position
		       Matrix3 &rRh_ref_out);		                                                                // Rotation

  void set_ref_handpos(Vector3 rPh_ref_in, 					                // Set desired base2endeffector position
		       Matrix3 rRh_ref_in);		                                                                // Rotation

  // Time
  unsigned long get_Iteration();
  void set_Iteration();

  // Control Methods
  //void set_sampling_period(double period_in);
  bool velocity_control(Vector3 rdP, Vector3 rW, bool f_new);
  bool impedance_control();
#endif
  bool impedance_control2( Vector3 rXd_des, Vector3 rRd_des, Vector3 rFd_des, Vector3 rMd_des, int control_step, bool stop_flag, bool init_impedance );

  // Gravity Compensation
	void setGC(Vector3 grav_accel, double M, Vector3 cm);
//  void reset_gravity_comp();
//  int gravity_comp();
//  int moveto_q_goal(Vector6d q_goal_in);
	// Sensor resolution
	void setResolution(double force_resolution, double moment_resolution);

#if 0
  // Pivot Approach and Control Bais Approach
  int PivotApproach(double 	cur_time,
		    Vector3 	pos,
		    Matrix3 	rot,
		    Vector6d 	currForces,
		    Vector6d& JointAngleUpdate,
		    Vector6d& CurrAngles);
  // Reset the original EndEff pos and rpy
  void set_OrgPosRot(Vector3& pos, Vector3& RPY);

	// Move arm
	void setDesTime(double move_time);
	void setDestinationPos(Vector3 org_pos,Matrix3 org_rot,Vector3 des_pos,Matrix3 des_rot);
	bool moveArm(double cur_time);

	void closeLogfiles();
	
  // Files
  char TrajState1[STR_LEN];
  char TrajState2[STR_LEN];
  char Angles[STR_LEN];
  char CartPos[STR_LEN];
  char Forces[STR_LEN];
  char State[STR_LEN];
  char manipTest[STR_LEN];
#endif

#ifdef SIMULATION
  double raw_forces[6];
#endif

  /*********************************************************** MEMBERS ******************************************************/
 protected:

	// Asembly Strategy - Pivot Approach object
//  AssemblyStrategy* PA;						// Pivot Approach strategy pointer

	//static const int ARM_DOF = 6;
	cmode controlmode;						// Control Mode Flag

	std::string name;						// String name for right or left arm
	FILE *fp1,*fp2, *fp3;						// File streams

	BodyPtr      body;						// Body (link objects) pointer
	JointPathPtr m_path;						// Path variable of OpenHRP model
	//	OpenHRP::JointPath* path;

	// Angles
	Vector6d 	 q;						// Joint angles(radians)
	float 		 ang_limits[ARM_DOF][5];		               	// Angle limits
	unsigned int NUM_q0;						// Number of start joint of arm

	// Sensors
	DeviceList<ForceSensor> forceSensors;
	DeviceList<RateGyroSensor> rateGyroSensors;
	DeviceList<AccelSensor> accelSensors;

	// Force
	//nittaFS 	*fs;						// Force Sensor Pointer
	int 		NO_fs;						// number of force sensor (0 or 1)
	double 	DEL_T;						// sampling period
	double 	ikGains[4], ikRot[4], ikTrans[4];		// Gains

	// Gravity compensation
	Vector3 gc_gravaccel;
	double gc_M;
	Vector3 gc_cm;
	// Sensor resolution
	double force_resolution;
	double moment_resolution;
	// Forces
	Vector3 fsF_raw, fsM_raw;				        // Raw force and moment values of FS
	Vector3 rFfs_gc, rMfs_gc;				        // gravity compensated force-moment value of FS in robot coord for the arm
  int frame_cnt;
  Vector3 fsF_accum;
  Vector3 fsM_accum;
	Vector3 fsF_prev[4];
	Vector3 fsM_prev[4];
	Vector3 rFh_gc, rMh_gc;						// gravity compensated force-moment value of FS in robot coord for the hand
  Vector3 rFh_lpf, rMh_lpf;
	//Vector3 fsF_offset, fsM_offset;		                // Initial force-monment offset of FS
	//Vector3 fsF, fsM;						// Initialized force-moment value.
  // angular velocity
  Vector3 omega;
  Vector3 rpy0;
  // previous position,velocity,acceleration
  Vector3 x0;
  Vector3 dx0;
  Vector3 ddx0;
  //
  Vector3 Dsection;
  Vector3 Ksection;
  Vector3 Fsection;
  bool IK_result;
  bool IKfast_result;
  bool prev_stop;

	/************************************  Position Parameters ******************************************************************/
	// r stands for the base of the robot
	// e stands for wrist
	// h stands for endeffector
	// fs stands for force sensor

	/*********** Arm's Current State ***************/
	Vector3  ePh;								// Translation from wrist to end effector
	Matrix3	eRh;								// Rotation from wrist to end effector
	Vector3  hPfs;								// Translation from end effector to force sensor

	Matrix3	hRfs;								// Rotation from end effector to force sensor (FUKUMOTO added)

	Vector3  rPh;								// Translation from base to end effector
	Matrix3 rRh;								// Rotation from base to end effector

	Vector3  rPe;								// Translation from base to wrist
	Matrix3 rRe;								// Rotation from base to wrist

	/*********** Arm's Reference State ***************/
	Vector3  rPh_ref;							// Desired position (Translation from base to end effector)
	Matrix3 rRh_ref;							// Desired Rotation (Rotation from base to end effector)

	Vector3  rPe_ref;							// Desired position (Translation from base to wrist)
	Matrix3 rRe_ref;							// Desired Rotation (Rotation from base to wrist)

	Vector6d q_ref;								// Desired joint angle(rad)

	// Arm motion
	bool f_moving, f_reached;					// Motion flag variables: is moving? has reached goal?
	unsigned long m_time;						// amount of time that motion has taken place
	path_params p_param;						// structure containing info about the motion composed of: interpolation method, Max. Angular Joint Vel, starting goal joint position, ending goal joint position, Distance, and duration.

	const double MAX_JVEL;					// Maximum limit for joint velocity, i.e. 0.5 (rad/s)??
	const double TIME_LIMIT;					// Time limit
	const double MIN_PERIOD;					// Min Period
	const double TOLERANCE;					// Tolerance

	// Impedance control
	Matrix3 Kt, Kr, Mt, Mr, Dt, Dr, Ft, Fr;					//
	Vector3 rXd,rdXd,rddXd;						// Desired pos, dpos, ddpos
	Vector3 rRd,rdRd,rddRd;						// Desired rot, drot, ddrot

	// Gravity compensation
	bool		 f_gc, f_gc_init;				// f_gc: ; f_gc_init: have we initialize grav. comp. variables?
	int 		 step_gc, step_fs;				// step_gc: has gravity compensation been executed
	double 		 mass;
	const int 	 max_step_fs, wait_step;			// Max = 500, Wait=200
	const double GACC;						// Gravity acceleration (9.8 m/s^2)

	Vector3  rG_vec;						// Unit gravity vector in global frame (robot frame)
	Vector3  fsPgc;							// Centroid in hand frame
	Vector3  fsF_offset, fsM_offset;				// Force offset in force sensor frame
	Vector3  fsF_tmp[5], fsM_tmp[5];
	Vector6d q_gc_ref[5];
	Matrix3 fsRr_gc[5];

	// Others
	Vector3  rPh_initial;
	Matrix3 rRh_initial;
	Vector3  fsF_initial, fsM_initial;

	//#ifdef IMPEDANCE
	std::ifstream fin_lpos;						// left position data
	std::ifstream fin_rpos;						// right position data
	std::ifstream fin_lfc;						// left force data
	std::ifstream fin_rfc;						// right force data
	//#endif

	// Flags
	bool initFlag;

	/************************************************** Functions **********************************************/
	// Motion
//	bool is_moving();
//	bool is_reached();
//	bool is_activated();

	// Path Functions
//	bool init_path_params(Vector6d q_start_in, Vector6d q_goal_in);
//	bool set_path_params(path_params param_in);
//	bool set_path_method(interpolation method_in);
//	bool set_path_max_jvel(double max_jvel_in);
//	bool calc_duration_jvel(Vector6d distance_in, double max_jvel_in);
	bool calc_qref(Vector3 rPh_ref_in, Matrix3 rRh_ref_in, Vector6d &q_out);
//	bool moveto(Vector6d q_start, Vector6d q_goal);

	// Gravity Compensation
//	bool calc_gravity_param();
//	bool calc_gravity_param_shimizu();

	// Force Computations
	bool get_raw_forces();
//	bool get_raw_forces(double f_out[6]);
	bool calc_gc_forces();
	void calc_gc_forces_at_hand();
//	virtual Matrix3 calc_hRfs(Vector6d q_in);

	// Mathematical
//	void SkewToVector(Matrix3 skew, Vector3 &vec);
//	Matrix3 get_rot33(int dir, double rad);
	void calc_rPe_rRe(Vector3 rPh_in, Matrix3 rRh_in, Vector3 &rPe_out, Matrix3 &rRe_out); //base2wrist position and rotation matrices


};


//-------------------------------------------------------------------------------------------------
template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6)
{
  using namespace Eigen;
  typedef JacobiSVD<t_matrix> TSVD;
  unsigned int svd_opt(ComputeThinU | ComputeThinV);
  if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
  svd_opt= ComputeFullU | ComputeFullV;
  TSVD svd(m, svd_opt);
  const typename TSVD::SingularValuesType &sigma(svd.singularValues());
  typename TSVD::SingularValuesType sigma_inv(sigma.size());
  for(long i=0; i<sigma.size(); ++i)
  {
    if(sigma(i) > tolerance)
      sigma_inv(i)= 1.0/sigma(i);
    else
      sigma_inv(i)= 0.0;
  }
  return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
}

