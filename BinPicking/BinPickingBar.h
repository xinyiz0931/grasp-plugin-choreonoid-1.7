/**
   @author Takashi Kitagawa (AIST)
   copied by xinyi 2022/12/1
*/

#ifndef _MOTION_FILE_BAR_H_INCLUDED
#define _MOTION_FILE_BAR_H_INCLUDED

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../PRM/TrajectoryPlanner.h"
#include "../MotionFile/MotionFileControl.h"
#include "../../graspPlugin/Grasp/exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#endif

using namespace cnoid;

	namespace grasp {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		class EXCADE_API BinPickingBar : public cnoid::ToolBar, public boost::signals::trackable
#else
		class EXCADE_API BinPickingBar : public cnoid::ToolBar
#endif
		{
			public:

			static BinPickingBar* instance();

			virtual ~BinPickingBar();

			std::string objectBasePath;

			protected:

			private:

			BinPickingBar();

			MessageView& mes;
			std::ostream& os;

			void onPlanButtonClicked();
			void onPlanDualArmButtonClicked();
			void onClearButtonClicked();
			void onSaveButtonClicked();

			void doBinPickingFunc();
			void doCablePickingFunc();
			void doNewFunc();

			void doMove();
			void doMoveTHK();
			void doIniPose();
        };
   }

#endif


