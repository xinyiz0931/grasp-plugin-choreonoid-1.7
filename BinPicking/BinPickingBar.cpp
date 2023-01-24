/**
   @author Takashi Kitagawa (AIST)
   copied by xinyi 2022/12/1
*/


#include "BinPickingBar.h"
#include "BinPickingControl.h"

#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <cnoid/ExecutablePath>
#include <cnoid/Item>
#include <cnoid/ScriptItem>
#include <cnoid/ItemManager>
#include <iostream>
#include <fstream>

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
BinPickingBar* BinPickingBar::instance()
{
	static BinPickingBar* instance = new BinPickingBar();
	return instance;
}

BinPickingBar::BinPickingBar()
	: ToolBar("BinPickingBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	setVisibleByDefault(true);
#endif
	addSeparator();

	addLabel(("=BinPicking="));

	addButton(("Plan"), ("Motion File Play"))->
		sigClicked().connect(bind(&BinPickingBar::onPlanButtonClicked, this));
	addButton(("PlanDualArm"), ("Motion File Play while torso fixed"))->
		sigClicked().connect(bind(&BinPickingBar::onPlanDualArmButtonClicked, this));
	
	addButton(("Clear"), ("Motion File Clear"))->
		sigClicked().connect(bind(&BinPickingBar::onClearButtonClicked, this));

	addButton(("Save"), ("Save All Joints"))->
		sigClicked().connect(bind(&BinPickingBar::onSaveButtonClicked, this));

	addSeparator();

	addButton(("Move"), (""))->
		sigClicked().connect(bind(&BinPickingBar::doMove, this));
	addSeparator();
	
	addButton(("MoveWithFT"), ("Plan fling motion"))->
		sigClicked().connect(bind(&BinPickingBar::doMoveFT, this));
	
	addButton(("Fling"), ("Motion File Play while torso fixed"))->
		sigClicked().connect(bind(&BinPickingBar::onPlanTmpButtonClicked, this));

	addSeparator();
	addButton(("GoInitial"), (""))->
		sigClicked().connect(bind(&BinPickingBar::doIniPose, this));

}


BinPickingBar::~BinPickingBar()
{

}


/* --------------- */
/* Loadボタンの処理 */
/* --------------- */
void BinPickingBar::onPlanButtonClicked()
{
	string motionfilepath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/motion/motion.dat";
	MotionFileControl::instance()->LoadFromMotionFile(motionfilepath, false);
}
void BinPickingBar::onPlanDualArmButtonClicked()
{
	string motionfilepath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/motion/motion.dat";
	MotionFileControl::instance()->LoadFromMotionFile(motionfilepath, true);
}

void BinPickingBar::onPlanTmpButtonClicked()
{
	ItemPtr item;
    item = ItemManager::create("Python", "PythonScriptItem");

    ScriptItemPtr scriptItem = dynamic_pointer_cast<ScriptItem, Item>(item);
	string scriptPath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/script/script_fling.py";
    scriptItem->load(scriptPath, "PYTHON-SCRIPT-FILE");
	scriptItem->execute();

	onPlanButtonClicked();
	onPlanButtonClicked();
	// doMove();

	// string motionfilepath = "/home/hlab/bpbot/data/motion/motion.dat";
	// std::system("python /home/hlab/choreonoid-1.7.0/ext/graspPlugin/BinPicking/project/script_move_ft.py");
}

/* --------------- */
/* Clearボタンの処理 */
/* --------------- */

void BinPickingBar::onClearButtonClicked()
{
	if (!PlanBase::instance()->graspMotionSeq.empty())
		MotionFileControl::instance()->ClearMotionFile();
}

/* --------------- */
/* Saveボタンの処理 */
/* --------------- */
void BinPickingBar::onSaveButtonClicked()
{
	string savepath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/motion/saved.dat";
	BinPickingControl::instance()->SaveJnt(savepath);
}

void BinPickingBar::doMoveFT()
{
	ItemPtr item;
    item = ItemManager::create("Python", "PythonScriptItem");

    ScriptItemPtr scriptItem = dynamic_pointer_cast<ScriptItem, Item>(item);
	string scriptPath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/script/script_move_ft.py";
    scriptItem->load(scriptPath, "PYTHON-SCRIPT-FILE");
    
	scriptItem->execute();
	// string motionfilepath = "/home/hlab/bpbot/data/motion/motion.dat";
	// std::system("python /home/hlab/choreonoid-1.7.0/ext/graspPlugin/BinPicking/project/script_move_ft.py");
}

void BinPickingBar::doMove()
{
	ItemPtr item;
    item = ItemManager::create("Python", "PythonScriptItem");

    ScriptItemPtr scriptItem = dynamic_pointer_cast<ScriptItem, Item>(item);
	string scriptPath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/script/script_move.py";
    scriptItem->load(scriptPath, "PYTHON-SCRIPT-FILE");
    
	scriptItem->execute();
}

void BinPickingBar::doIniPose()
{
	ItemPtr item;
    item = ItemManager::create("Python", "PythonScriptItem");

    ScriptItemPtr scriptItem = dynamic_pointer_cast<ScriptItem, Item>(item);
	string scriptPath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/script/script_goinitial.py";
    scriptItem->load(scriptPath, "PYTHON-SCRIPT-FILE");
    
	scriptItem->execute();
}
/* end */
