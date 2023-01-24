/*
   @author Takashi Kitagawa (AIST)
*/

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/GraspController.h"
#include "../PRM/TrajectoryPlanner.h"
#include "../MotionFile/MotionFileControl.h"
#include "../../graspPlugin/Grasp/exportdef.h"

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

			private:
			// added by xinyi

			MessageView& mes;
			std::ostream& os;

			public:

            void SaveJnt(std::string savepath);
		};
   }


