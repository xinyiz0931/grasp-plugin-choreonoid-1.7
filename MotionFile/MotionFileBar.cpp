/**
   @author Takashi Kitagawa (AIST)
*/


#include "MotionFileBar.h"
#include "MotionFileControl.h"

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

MotionFileBar* MotionFileBar::instance()
{
	static MotionFileBar* instance = new MotionFileBar();
	return instance;
}

MotionFileBar::MotionFileBar()
	: ToolBar("MotionFileBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	setVisibleByDefault(true);
#endif
	addSeparator();

	addLabel(("=MotionFile="));

	addButton(("Play"), ("Motion File Play"))->
		sigClicked().connect(bind(&MotionFileBar::onLoadButtonClicked, this));

	addButton(("Clear"), ("Motion File Clear"))->
		sigClicked().connect(bind(&MotionFileBar::onClearButtonClicked, this));

	addButton(("SavePosition"), ("Save Position to DB (PrePlanning File)"))->
		sigClicked().connect(bind(&MotionFileBar::onSavePositionButtonClicked, this));
	
	addSeparator();
	addButton(("Move"), (""))->
		sigClicked().connect(bind(&MotionFileBar::doMove, this));
	
}


MotionFileBar::~MotionFileBar()
{

}


/* --------------- */
/* Loadボタンの処理 */
/* --------------- */
void MotionFileBar::onLoadButtonClicked()
{
	string  motionfile, basepath;
	//basepath = cnoid::executableTopDirectory() + "/extplugin/graspPlugin/MotionFile/data/";
	//motionfile = basepath + "motion.dat";    // default
	//FILE *fp;
	//if( (fp = fopen((basepath+"motionfile.dat").c_str(), "r")) != NULL ){
	//	ifstream motionfilePath((basepath + "motionfile.dat").c_str());
	//	string line;
	//	while(getline(motionfilePath, line)){
	//		if(line.empty() || line.compare(0,1,"#")==0) continue;
	//		motionfile = line;
	//		break;
	//	}
	//	fclose(fp);
	//}
	motionfile = cnoid::executableTopDirectory() + "/ext/bpbot/data/motion/motion.dat";
	// MotionFileControl::instance()->LoadFromMotionFile(motionfile, true);
	MotionFileControl::instance()->LoadFromMotionFile(motionfile, false);
}

/* --------------- */
/* Clearボタンの処理 */
/* --------------- */

void MotionFileBar::onClearButtonClicked()
{
	MotionFileControl::instance()->ClearMotionFile();
}

/* --------------- *pedef ref_ptr<Item> ItemPtr;/
/* Save Positionボタンの処理 */
/* --------------- */
void MotionFileBar::onSavePositionButtonClicked()
{
	MotionFileControl::instance()->SavePositionToDB();
}

void MotionFileBar::doMove()
{
	// int rt;
	// rt = std::system("python ./ext/bpbot/example/xinyi.py");
	ItemPtr item;
    item = ItemManager::create("Python", "PythonScriptItem");

    ScriptItemPtr scriptItem = dynamic_pointer_cast<ScriptItem, Item>(item);
	string scriptPath = cnoid::executableTopDirectory() + "/ext/graspPlugin/BinPicking/project/script_move.py";
    scriptItem->load(scriptPath, "PYTHON-SCRIPT-FILE");
    
	scriptItem->execute();
}


/* end */

