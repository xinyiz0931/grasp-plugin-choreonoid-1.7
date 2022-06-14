/**
   @author Shin'ichiro Nakaoka
*/
   
#include <cnoid/SimpleController>
#include <cnoid/ExecutablePath>
#include <cnoid/Link>
#include <cnoid/YAMLReader>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

using namespace cnoid;
using namespace std;

class TiltGearController : public cnoid::SimpleController
{
  Link* joint0;
  vector<double> starttime;
  vector<double> dgain;
  vector<double> angular_velocity;
  int stage;
  int currentFrame;
  double timestep;
  ofstream *damper_log;

public:

  virtual bool initialize() {

    joint0 = ioBody()->link("ROTATE2");

    if(!joint0){
        os() << "rotate joint \"ROTATE2\" cannot be detected." << std::endl;
        return false;
    }
    if(!isImmediateMode()){
        os() << "Controller should be used with the immediate mode." << std::endl;
        return false;
    }
    // read gain
    string filename = executableTopDirectory() + "/extplugin/graspPlugin/RobotModels/HRP2STEP1/Controller/data/gear_gain.dat";
    ifstream gain;//(filename.c_str());
    if(!gain){
      os() << "gear_gain.dat not found." << std::endl;
      return false;
    }
    gain.open(filename.c_str());
    // count line number
    int line_num = 0;
    string line;
    while(getline( gain, line )){
      cout << line << endl;
      ++line_num;
    }
    cout << "line_num: " << line_num;
    starttime.resize(line_num);
    dgain.resize(line_num);
    angular_velocity.resize(line_num);
    gain.clear();
    gain.seekg(0, ios_base::beg);
    for(int i=0; i<line_num; ++i) {
      gain >> starttime[i];
      gain >> angular_velocity[i];
      gain >> dgain[i];
    }
    stage = 0;
    currentFrame = 0;
    timestep = timeStep();
    damper_log = new std::ofstream((executableTopDirectory()+"/extplugin/graspPlugin/MotionFile/data/Gear_log.csv").c_str());
    *damper_log << "#time,angle(deg),torque,dq" << endl;
		// YAML
		YAMLReader parser;
		ValueNode* doc = parser.loadDocument((executableTopDirectory()+"/extplugin/graspPlugin/RobotModels/HRP2STEP1/Controller/data/test.yaml").c_str());
		Mapping* root = doc->toMapping();
		//ValueNode* node = root->find("FingerSetting");
		ValueNode* node = root->find("Prehension");
		const YamlMapping& gSettings = *node->toMapping();
		cout << gSettings["name"].toString() << endl;
		cout << "List? " << node->isListing() << ", Map? " << node->isMapping() << endl;
		ValueNode* node2 = root->findSequence("FingerSetting");
		cout << "List? " << node2->isListing() << ", Map? " << node2->isMapping() << endl;
		if(node->isListing()){
			Listing* list = node->toListing();
			cout << "list size = " << list->size() << endl;
		}
		cout << __FILE__ << ", " << __LINE__ << ", node=" << node << endl;
		cout << "List? " << node->isListing() << ", Map? " << node->isMapping() << endl;
		node = root->findMapping("GRCDES");
		cout << __FILE__ << ", " << __LINE__ << ", node=" << node <<  endl;
		cout << "List? " << node->isListing() << ", Map? " << node->isMapping() << endl;
		node = root->findSequence("printscalar");
		cout << __FILE__ << ", " << __LINE__ << ", node=" << node <<  endl;
		cout << "List? " << node->isListing() << ", Map? " << node->isMapping() << endl;
		node = root->find("notexist");
		cout << __FILE__ << ", " << __LINE__ << ", node=" << node <<  endl;
		cout << "List? " << node->isListing() << ", Map? " << node->isMapping() << endl;
		cout << node->toString() << endl;
		cout << __FILE__ << ", " << __LINE__ << endl;
    return true;
  }

  virtual bool control() {
    double currenttime = currentFrame * timestep;
    if( stage<starttime.size()-1 && currenttime >= starttime[stage+1]){
      cout << "stage " << stage << endl;
      ++stage;
    }
    joint0->u() = - dgain[stage] * (joint0->dq()-angular_velocity[stage]);
    *damper_log << currenttime << " " << joint0->q()*57.29578 << " " << joint0->u() << " " << joint0->dq() << " " << joint0->p()(0) << " " << joint0->p()(1) << " " << joint0->p()(2) << endl;
    ++currentFrame;
    return true;
  }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TiltGearController)
