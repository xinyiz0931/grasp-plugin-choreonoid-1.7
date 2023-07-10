/*
   @author Xinyi Zhang (Osaka Univ.)
*/

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/VectorMath.h"
#include "../PRM/TrajectoryPlanner.h"
// #include "../MotionFile/MotionFileControl.h"
#include "../../graspPlugin/Grasp/exportdef.h"
#include "../PRM/mpkInterface.h"
#include "../PRM/RRTPathPlanner.h"
#include "../PRM/RRT.h"
#include "../PRM/RRTStar.h"
#include "../PRM/ParamDialog.h"



#include "../Grasp/GraspController.h"
#include "../Grasp/AssemblyObject.h"
#include "../Grasp/ObjectBase.h"
#include "../Grasp/ObjectManager.h"


#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

using namespace cnoid;

	namespace grasp {

		//class EXCADE_API BinPickingControl : public cnoid::ToolBar, public boost::signals::trackable
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		class EXCADE_API BinPickingControl : public boost::signals::trackable
#else
		class EXCADE_API BinPickingControl
#endif
		{
			public:

			static BinPickingControl* instance();

			virtual ~BinPickingControl();

			BinPickingControl();

			protected:
			ArmPtr arm, arm_;
			Vector3 tmpPos, tmpRPY;

			private:
			MessageView& mes;
			std::ostream& os;

			public:
			PlanBase *gc;
			Vector3		InitBasePos, InitBaseRPY;
			VectorXd	InitJnt;
			std::vector<VectorXd> InitArmJnt;
			std::vector<double> _InitJnt;
			
			MotionState tmpState;
			enum handState {HAND_OPENING=3, HAND_CLOSING=4};

			// for trajectory planning
			std::vector<MotionState> motionSeq;
			cnoid::PoseSeqItem* poseSeqItemRobot;

			int			init;
			int 		initRegrasp;

			int numJnt;

            void SaveJnt(std::string savepath);

			void initJointSeq();
			void clearJointSeq(VectorXd jnt);

			void calcIK(int LR_flag, Vector3d pos, Vector3d rpy, VectorXd &jnt);
			void calcArmJointSeq(VectorXd& armJointseq, ArmPtr& arm);
			void calJointSeq(Vector3d pos, Vector3d rpy);
			void calcJointSeqTwoPoses(int LR_flag, Vector3d startpos, Vector3d startrpy, Vector3d endpos, Vector3d endrpy);
			// void calcJointSeqMoveTo(int LR_flag, Vector3d endpos, Vector3d endrpy);
			void calcJointSeqMoveTo(int LR_flag, vector<VectorXd> poses, vector<double> tms);
			void calcJointSeqRegrasp(Vector3d initGraspPos, Vector3d initGraspRPY, Vector3d terminatePos, Vector3d terminateRPY);
			void calcJointSeqPick(int LR_flag, Vector3d pos, Vector3d rpy);
			void calcJointSeqTransport(int LR_flag, Vector3d pos, Vector3d rpy);
			void calcJointSeqSwing(int LR_flag, Vector3d pos, Vector3d rpy, Vector3d angles, double T=0.5, int repeat=1, bool bilateral=false, Vector3d endpos={});
			void calcJointSeqSpin(int LR_flag, double velocity);
			void calcJointSeqPut(int LR_flag, Vector3d pos, Vector3d rpy);
			// for demo 20230217
			void calcJointSeqWaveHand();

			void calcJointSeqShake(int LR_Flag, Vector3d startpos, Vector3d startrpy, Vector3d endpos, int n=8);

			void setMotionSeqDual(int LR_flag, double T, Vector3d pos, Vector3d rpy, bool is_fixwaist=true);
			void setMotionSeq(int LR_flag, double T, Vector3d pos, Vector3d rpy);
			void setHandStateSeq(int LR_flag, int handState);
			void setArmJointSeq(int LR_flag, double T, VectorXd armJnt);
			void setJointSeq(double T, VectorXd jnt);
			
			//new function
			void setRelJoint(double T, string jntName, double jntAngle);

			bool trajectoryPlanning();
			void doBinPickingPlanning(int LR_flag, Vector3d pickP, Vector3d pickR, Vector3d placeP);


		};
   }


