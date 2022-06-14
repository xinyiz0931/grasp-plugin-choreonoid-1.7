#include <fstream>
#include <vector>
#include <string>
#include <cnoid/SimpleController>
#include <cnoid/ExecutablePath>
#include <cnoid/ItemTreeView>
#include <cnoid/SimulatorItem>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>

using namespace std;
using namespace cnoid;

class SimpleGraspController : public cnoid::SimpleController {
private:
	BodyPtr body;
	MultiValueSeqPtr qseq;
	vector<double> q0;
	MultiValueSeq::Frame oldFrame;
	int currentFrame;
	vector<double> pgain;
	vector<double> dgain;

public:
	virtual bool initialize() {
		body = ioBody();
		// get trajectory path from PoseSeqItem
		ItemList<PoseSeqItem> ps_items = ItemTreeView::mainInstance()->selectedItems<PoseSeqItem>();
		ItemList<SimulatorItem> simu_items = ItemTreeView::mainInstance()->selectedItems<SimulatorItem>();
		if (ps_items.size() < 1) {
			os() << "Please select poseseqitem." << endl;
			return false;
		}
		ItemTreeView::mainInstance()->clearSelection();
		ItemTreeView::mainInstance()->selectItem(simu_items[0]);
		BodyMotionPtr motion =  ps_items[0]->bodyMotionItem()->motion();
		qseq = motion->jointPosSeq();
		if (qseq->numFrames() == 0) {
			os() << "Empty motion data." << endl;
			return false;
		}
		q0.resize(qseq->numParts());
		double timestep = qseq->getTimeStep();
		setTimeStep(timestep);
		for (int i = 0; i < body->numJoints(); ++i) {
			q0[i] = body->joint(i)->q();
		}
		oldFrame = qseq->frame(0);
		currentFrame = 0;

		// read gain
		string filename = executableTopDirectory() + "/extplugin/graspPlugin/RobotModels/HRP2STEP1/Controller/data/gain.dat";
		ifstream gain;
		gain.open(filename.c_str());
		pgain.resize(body->numJoints());
		dgain.resize(body->numJoints());
		for (int i = 0; i < body->numJoints(); ++i) {
			gain >> pgain[i];
			gain >> dgain[i];
		}

		return true;
	}

	virtual bool control() {
		if(currentFrame < qseq->numFrames()) {
			MultiValueSeq::Frame frame = qseq->frame(currentFrame++);
			for (int i = 0; i < body->numJoints(); ++i) {
				Link* joint = body->joint(i);
				double q_ref = frame[i];
				double q = joint->q();

				double dq_ref = (q_ref - oldFrame[i]) / timeStep();
				double dq = (q - q0[i]) / timeStep();
				joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
				q0[i] = q;
			}
			oldFrame = frame;
			return true;
		} else {
			// keep last frame pose
			for (int i = 0; i < body->numJoints(); ++i) {
				Link* joint = body->joint(i);
				double q_ref =oldFrame[i];
				double q = joint->q();

				double dq_ref = 0;
				double dq = (q - q0[i]) / timeStep();
				joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
				q0[i] = q;
			}
		}
		return false;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SimpleGraspController)

