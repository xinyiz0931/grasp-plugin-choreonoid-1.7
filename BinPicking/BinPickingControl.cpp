/**
   @author Takashi Kitagawa (AIST)
   copied by xinyi 2022/12/1
*/

#include "BinPickingControl.h"
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/ExecutablePath>
#include <cnoid/MessageView>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <iostream>
#include <fstream>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/ViewManager>
#endif

#define ARM_NONE			0								// 腕無しロボット
#define ONE_ARM			1								// 片腕ロボット
#define DOUBLE_ARM		2								// 双腕ロボット
#define RIGHT_FLAG		0								// 右腕フラグ
#define LEFT_FLAG			1								// 左腕フラグ
#define RAD_CONV			(3.141592/180.0)				// radian変換
#define RadConv(x)		((x)*(3.141592)/(180.0))
#define DEG_CONV			(180.0/3.141592)				// degree変換
#define DegConv(x)		((x)*(180.0)/(3.141592))
// #define DEBUG_MOTIONFILE_INPUT 1

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

BinPickingControl* BinPickingControl::instance()
{
    static BinPickingControl* instance = new BinPickingControl();
    return instance;
}

BinPickingControl::BinPickingControl()
	: mes(*MessageView::mainInstance()),
	os (MessageView::mainInstance()->cout() )
{

}
BinPickingControl::~BinPickingControl()
{

}

void BinPickingControl::SaveJnt(string savepath)
{
    cout << "[*] Saved current pose in " << savepath << endl;
	PlanBase *tc = PlanBase::instance();

	ofstream gout(savepath);
	for (int i=0; i<tc->bodyItemRobot()->body()->numJoints(); i++) {
		gout << DegConv(tc->bodyItemRobot()->body()->joint(i)->q()) << " ";
	}
	gout << endl;
}

