#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>
#include "BinPickingBar.h"

#include <string>
#include <cnoid/Button>
#include <cnoid/ToolBar>
#include "../Grasp/GraspBar.h"


using namespace std;
using namespace cnoid;
using namespace grasp;


namespace {
    class BinPickingPlugin : public Plugin
    {
    public:
        BinPickingPlugin() : Plugin("BinPicking") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        	depend("Grasp");
		depend("Trajectory");
#else
		require("Grasp");
		require("Trajectory");
#endif
        }

        bool initialize() {

		addToolBar(grasp::BinPickingBar::instance());

// #if defined(CNOID_10_11_12_13) || defined(CNOID_14)
// 		addView(new MotionEdit());
// #else
// 		MotionEdit::initializeClass(this);
// #endif
		return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(BinPickingPlugin);

