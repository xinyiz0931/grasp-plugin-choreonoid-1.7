#include <fstream>
#include <vector>
#include <string>
#include <cnoid/SimpleController>
#include <cnoid/ExecutablePath>
#include <cnoid/ItemTreeView>
#include <cnoid/SimulatorItem>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>
#include <iostream>
#include "ImpedanceController.h"
#include "../../../MotionFile/MotionFileControl.h"

#define FS_FREQ 0.002

using namespace std;
using namespace cnoid;

vector<Vector3> Mass_param;
vector<Vector3> Damper_param;
vector<Vector3> K_param;
vector<Vector3> F_param;
vector<Vector3> rotMass_param;
vector<Vector3> rotDamper_param;
vector<Vector3> rotK_param;
vector<Vector3> rotF_param;

class ImpedanceController : public cnoid::SimpleController {
private:
	BodyPtr body;
	MultiValueSeqPtr qseq;
	vector<double> q0;
	MultiValueSeq::Frame oldFrame;
  vector<double> dq0;
  vector<double> reference_q;
  vector<double> reference_q0;
	int currentFrame;
  HRP2STEP1Arm *rArm;
  HRP2STEP1Arm *lArm;
  std::ofstream *log;
  std::ofstream *action_log;
	std::ofstream *center_estimate_log;
  std::ofstream *trace;
  Vector3 attachment_head_pos;
  Vector3 Worm_pos1;
  Vector3 Worm_pos2;
  Vector3 Worm_pos3;
  Vector3 Worm_pos4;
  std::ofstream *pos_trace_temp;
  std::ofstream *forcesensor_temp;
  bool output_file;
  double fs_freq;               // 1frameあたりの秒数
  static const double r2d = 57.29578;
  // 制御パラメータ関連
  int control_step;
  vector<double> param_change_time;
  vector<int> control_mode;
  vector<Vector3> targetF;            //右手のみ
  vector<Vector3> targetM;            //右手のみ
  // ゲイン関連
  int gain_step;
  vector<double> gain_change_time;
  vector< vector<double> > pgain;
  vector< vector<double> > dgain;
  // 探索関連
  int search_step;
  vector<double> search_time;
  bool stop_flag;
  vector<double> stop_frame;   // 停止中の関節角
  vector<int> search_mode;
  vector<int> search_type;
  vector<double> found_thres;
  vector<Vector3> search_direction;
	vector<Vector3> reference_position;
	// 円の中心に補正
	int adjust_step;
	vector<double> adjust_time;
	vector<int> submode;
	vector<int> record_idx;
	vector<int> numofrecords;
	vector<int> idxofcenter;
	vector<int> refindex;
	vector<Vector3> adjust_axis;
	vector<Vector3> circumference_sample;
	vector<Vector3> estimated_center;
  Vector3 base_position;
  Vector3 base1;
  Vector3 base2;
  Vector3 base3;
  Vector3 base4;
  Vector3 base_head;
  bool init_impedance;
	Vector3 offset;

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
    fs_freq = 1/qseq->frameRate();
    std::ofstream *qseq_log = new std::ofstream((executableTopDirectory()+"/extplugin/graspPlugin/MotionFile/data/qseq_log.csv").c_str());
    for(int i=0; i<qseq->numFrames(); ++i){
      for(int j=0; j < body->numJoints(); ++j){
        *qseq_log << qseq->frame(i)[j] << ", ";
      }
      *qseq_log << endl;
    }
		q0.resize(qseq->numParts());
    dq0.resize(qseq->numParts());
    reference_q.resize(qseq->numParts());
    reference_q0.resize(qseq->numParts());
		double timestep = qseq->getTimeStep();
		setTimeStep(timestep);
		for (int i = 0; i < body->numJoints(); ++i) {
			q0[i] = body->joint(i)->q();
      dq0[i] = 0;
      reference_q[i] = 0;
      reference_q0[i] = qseq->frame(0)[i];
		}

		oldFrame = qseq->frame(0);
		currentFrame = 0;

    // initialize rArm and lArm
    Link* rWrist = body->link("RARM_JOINT5");
    Link* lWrist = body->link("LARM_JOINT5");
    Link* rHand = body->link("RARM_JOINT5");
    Link* lHand = body->link("LARM_JOINT5");
    DeviceList<ForceSensor> forceSensors = body->devices();
    ForceSensor* rForceSensor = forceSensors.get(0);   // '0': right arm
    ForceSensor* lForceSensor = forceSensors.get(1);   // '1': left arm
    Vector3 r_ePh = rWrist->R().transpose()*(rHand->p() - rWrist->p());
    Matrix3 r_eRh = rWrist->R().transpose()*rHand->R();
    Vector3 r_rPfs = rForceSensor->link()->p() + rForceSensor->p_local();
    Vector3 r_hPfs = rHand->R().transpose()*(r_rPfs - rHand->p());
    Matrix3 r_hRfs = rHand->R().transpose()*rForceSensor->link()->R()*rForceSensor->R_local();
    float r_ang_limit[6][5];
    rArm = new HRP2STEP1Arm("right", body, 3, fs_freq, r_ang_limit, r_ePh, r_eRh, r_hPfs, r_hRfs);
    Vector3 l_ePh = lWrist->R().transpose()*(lHand->p() - lWrist->p());
    Matrix3 l_eRh = lWrist->R().transpose()*lHand->R();
    Vector3 l_rPfs = lForceSensor->link()->p() + lForceSensor->p_local();
    Vector3 l_hPfs = lHand->R().transpose()*(l_rPfs - lHand->p());
    Matrix3 l_hRfs = lHand->R().transpose()*lForceSensor->link()->R()*lForceSensor->R_local();
    float l_ang_limit[6][5];
    lArm = new HRP2STEP1Arm("left", body, 9, fs_freq, l_ang_limit, l_ePh, l_eRh, l_hPfs, l_hRfs);

		action_log = new std::ofstream((executableTopDirectory()+"/extplugin/graspPlugin/MotionFile/data/action_log.csv").c_str(), ios::app);
		*action_log << endl << endl;
		center_estimate_log = new std::ofstream((executableTopDirectory()+"/extplugin/graspPlugin/MotionFile/data/center_estimate_log.csv").c_str(), ios::app);
		*center_estimate_log << endl << endl << "#---------------------" << endl;
    log = new std::ofstream((executableTopDirectory()+"/extplugin/graspPlugin/MotionFile/data/wrist_traj.csv").c_str());
		log_header(log);
#if 0
    *log << "#time,";
    *log << "ref_x,impedance_ref_pos_x,curpos_x,stop_pos_x,basepos_x,";
    *log << "ref_y,impedance_ref_pos_y,curpos_y,stop_pos_y,basepos_y,";
    *log << "ref_z,impedance_ref_pos_z,curpos_z,stop_pos_z,basepos_z,";
    *log << "ref_roll,ref_pitch,ref_yaw,";
    *log << "impedance_roll,impedance_pitch,impedance_yaw,";
    *log << "roll,pitch,yaw,";
    *log << "v_x,v_y,v_z,";
    *log << "f_x,f_y,f_z,";
//    *log << "f_x(LPF),f_y(LPF),f_z(LPF),";
    *log << "accel_x,accel_y,accel_z,";
//    *log << "raw_f_x,raw_f_y,raw_f_z,";
//    *log << "raw_moment_x,raw_moment_y,raw_moment_z,";
    *log << "moment_x,moment_y,moment_z,";
    *log << "control_mode,";
    *log << "stop,";
    *log << "Ph_ref_x,Ph_ref_y,Ph_ref_z,";
    *log << "Ph_x,Ph_y,Ph_z,";
    *log << "Pe_ref_x,Pe_ref_y,Pe_ref_z,";
    *log << "Re_ref_x,Re_ref_y,Re_ref_z,";
    *log << "Pe_x,Pe_y,Pe_z,";
    *log << "Dsection_x,Ksection_x,Fsection_x,";
    *log << "Dsection_y,Ksection_y,Fsection_y,";
    *log << "Dsection_z,Ksection_z,Fsection_z,";
    *log << "rdXd_x,rdXd_y,rdXd_z,";
    *log << "ikresult,ikfast_result,";
    *log << endl;
#endif

    trace = new std::ofstream((executableTopDirectory()+"/extplugin/graspPlugin/MotionFile/data/sim_trace.csv").c_str());
    attachment_head_pos = Vector3(-0.25,0,0);   // 手元からWorm先端への変位
    Worm_pos1 = Vector3(-0.10,0,0);
    Worm_pos2 = Vector3(-0.12,0,0);
    Worm_pos3 = Vector3(-0.14,0,0);
    Worm_pos4 = Vector3(-0.16,0,0);
    pos_trace_temp = new std::ofstream((executableTopDirectory()+"/position_traj_temp.csv").c_str());
    forcesensor_temp = new std::ofstream((executableTopDirectory()+"/forcesensor_temp.csv").c_str());
    output_file = false;

    // read control parameters and gain
    string ConfigBasePath = executableTopDirectory() + "/extplugin/graspPlugin/RobotModels/HRP2STEP1/Controller/data/";
    string config_file_name = ConfigBasePath + "controller_config.dat";    // default
    FILE *fp;
    if( (fp = fopen((ConfigBasePath + "configfile.dat").c_str(), "r")) != NULL ){
      ifstream ccfile((ConfigBasePath+"configfile.dat").c_str());
      string line;
      while(getline(ccfile, line)){
        // 空行と#から始まる行を飛ばす
        if(line.empty() || line.compare(0,1,"#")==0) continue;
        config_file_name = line;
        break;
      }
      fclose(fp);
    }
    cout << "read parameters from " << config_file_name << endl;
    ifstream config;
    config.open(config_file_name.c_str());
    string line;
    double time;
    double val;
    double x_comp, y_comp, z_comp;
    double tF_x,tF_y,tF_z;
    double Fparam_x, Fparam_y, Fparam_z;
    // 前回の読み込み内容を消去
    Mass_param.clear();
    Damper_param.clear();
    K_param.clear();
    F_param.clear();
    rotMass_param.clear();
    rotDamper_param.clear();
    rotK_param.clear();
    rotF_param.clear();
    param_change_time.clear();
    control_mode.clear();
    targetF.clear();
    targetM.clear();
    gain_change_time.clear();
    pgain.clear();
    dgain.clear();
    search_time.clear();
    search_mode.clear();
    search_type.clear();
    found_thres.clear();
    search_direction.clear();
		reference_position.clear();
		adjust_time.clear();
		submode.clear();
		record_idx.clear();
		numofrecords.clear();
		idxofcenter.clear();
		refindex.clear();
		adjust_axis.clear();
		circumference_sample.clear();
		estimated_center.clear();

    while(getline( config, line) ){
      if(line.empty() || line.compare(0,1, "#")==0) continue;
      stringstream ss;
      ss << line;
      ss >> time;
      int mode;
      ss >> mode;
      if( mode==1 ) {
        param_change_time.push_back(time);
        control_mode.push_back(mode);
        targetF.push_back(Vector3(0,0,0));
        Mass_param.push_back(Vector3(0,0,0));
        Damper_param.push_back(Vector3(0,0,0));
        K_param.push_back(Vector3(0,0,0));
        F_param.push_back(Vector3(0,0,0));
        targetM.push_back(Vector3(0,0,0));
        rotMass_param.push_back(Vector3(0,0,0));
        rotDamper_param.push_back(Vector3(0,0,0));
        rotK_param.push_back(Vector3(0,0,0));
        rotF_param.push_back(Vector3(0,0,0));
      }
      else if( mode==2 ) {
        param_change_time.push_back(time);
        control_mode.push_back(mode);
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        targetF.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        Mass_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        Damper_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        K_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        F_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        targetM.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        rotMass_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        rotDamper_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        rotK_param.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        rotF_param.push_back( Vector3(x_comp, y_comp, z_comp) );
      }
      else if( mode==3 ) {
        gain_change_time.push_back(time);
        vector<double> temp;
        for (int i = 0; i < body->numJoints(); ++i) {
          ss >> val;
          temp.push_back(val);
        }
        pgain.push_back(temp);
        temp.clear();
        for (int i = 0; i < body->numJoints(); ++i) {
          ss >> val;
          temp.push_back(val);
        }
        dgain.push_back(temp);
      }
      else if( mode==4 ) {
        search_time.push_back(time);
        ss >> val;
        search_mode.push_back(val);
        ss >> val;
        search_type.push_back(val);
        ss >> val;
        found_thres.push_back(val);
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
        search_direction.push_back( Vector3(x_comp, y_comp, z_comp) );
        ss >> x_comp; ss >> y_comp; ss >> z_comp;
				reference_position.push_back( Vector3(x_comp, y_comp, z_comp) );
      }
			else if( mode==5 ) {
				adjust_time.push_back(time);
				ss >> val;
				submode.push_back(val);
				if( val==0 ){
					ss >> val;
					record_idx.push_back(val);
					numofrecords.push_back(0);
					idxofcenter.push_back(0);
					refindex.push_back(0);
					adjust_axis.push_back( Vector3(0, 0, 0) );
				} else if( val==1) {
					ss >> val;
					numofrecords.push_back(val);
					ss >> val;
					idxofcenter.push_back(val);
					record_idx.push_back(0);
					refindex.push_back(0);
					adjust_axis.push_back( Vector3(0, 0, 0) );
				} else if( val==2) {
					ss >> val;
					refindex.push_back(val);
					ss >> x_comp; ss >> y_comp; ss >> z_comp;
					adjust_axis.push_back( Vector3(x_comp, y_comp, z_comp) );
					record_idx.push_back(0);
					numofrecords.push_back(0);
					idxofcenter.push_back(0);
				}
			}
			else if( mode==6 ) {
				Vector3 grav_accel;
				double M;
				Vector3 centerOfMass;
				// 重力加速度
				ss >> x_comp; ss >> y_comp; ss >> z_comp;
				grav_accel = Vector3(x_comp, y_comp, z_comp);
				// 右手の質量、重心位置
				ss >> val;
				M = val;
				ss >> x_comp; ss >> y_comp; ss >> z_comp;
				centerOfMass = Vector3(x_comp, y_comp, z_comp);
				rArm->setGC(grav_accel, M, centerOfMass);
				// 左手の質量、重心位置
				ss >> val;
				M = val;
				ss >> x_comp; ss >> y_comp; ss >> z_comp;
				centerOfMass = Vector3(x_comp, y_comp, z_comp);
				lArm->setGC(grav_accel, M, centerOfMass);
			}
			else if( mode==7 ){
				double f_resol, m_resol;
				ss >> f_resol;
				ss >> m_resol;
				rArm->setResolution(f_resol, m_resol);
				lArm->setResolution(f_resol, m_resol);
			}
    }
    config.close();
    control_step=0;
    gain_step=0;
    search_step=0;
		adjust_step=0;
    stop_flag = false;
    stop_frame.clear();
    for(int i=0; i < body->numJoints(); ++i){
      stop_frame.push_back(0);
    }
    init_impedance = false;
		offset = Vector3(0, -0.05, 0);
    return true;
  }

  virtual bool control() {
    body->calcForwardKinematics();
    rArm->update_currposdata();
    lArm->update_currposdata();
    Vector6d IK_q;

    const int POSITION_CONTROL = 1;
    const int IMPEDANCE_CONTROL = 2;
    static int r_control_mode = IMPEDANCE_CONTROL;
    static int l_control_mode = IMPEDANCE_CONTROL;

    if(currentFrame < qseq->numFrames()) {
      MultiValueSeq::Frame frame = qseq->frame(currentFrame++);
      vector<double> original_frame(body->numJoints());  // Impedance制御で変更される前のframeを保存
      double curtime = currentFrame * fs_freq;
/*
			if( submode[adjust_step]==2){
        vector<double> current_joint;
        current_joint.resize(body->numJoints());
        for( int i=0; i<body->numJoints(); ++i){
          // 退避
          current_joint[i] = body->joint(i)->q();
          // 目標の関節角
          body->joint(i)->q() = frame[i];
        }
        body->calcForwardKinematics();
				cout << "original pos : " << body->joint(8)->p() << endl;
				//body->joint(8)->p() += offset;
				bool inv_result = rArm->m_path->calcInverseKinematics(body->joint(8)->p()+offset, body->joint(8)->R());
				//cout << "calc Inverse " << inv_result << endl;
				Vector6d rArm_modified_qref = rArm->get_qref();
				cout << frame[3] << ", " <<frame[4] << ", " <<frame[5] << ", " <<frame[6] << ", " <<frame[7] << ", " <<frame[8] << endl;
				frame[3] = rArm_modified_qref(0);
				frame[4] = rArm_modified_qref(1);
				frame[5] = rArm_modified_qref(2);
				frame[6] = rArm_modified_qref(3);
				frame[7] = rArm_modified_qref(4);
				frame[8] = rArm_modified_qref(5);
				cout << frame[3] << ", " <<frame[4] << ", " <<frame[5] << ", " <<frame[6] << ", " <<frame[7] << ", " <<frame[8] << endl;
        for( int i=0; i<body->numJoints(); ++i){
          // 目標の関節角
          body->joint(i)->q() = frame[i];
        }
        body->calcForwardKinematics();
				cout << "modified pos : " << body->joint(8)->p() << endl;
        for (int i=0; i<body->numJoints(); ++i){
          // 元に戻す
          body->joint(i)->q() = current_joint[i];
        }
        body->calcForwardKinematics();
			}
				*/

      for(int i=0; i < body->numJoints(); ++i){
        original_frame[i] = frame[i];
        reference_q[i] = frame[i];
      }
      if( control_step<param_change_time.size()-1 && curtime >= param_change_time[control_step+1]){
        ++control_step;
#ifdef DEBUG_PLUGIN2
        cout << "control_step " << control_step << "(" << param_change_time[control_step] << ") :" << endl;
        cout << "  mode:" <<  (control_mode[control_step]==1 ? "position":"impedance") << endl;
        cout << "  force_ref: (" << targetF[control_step](0) << ", " << targetF[control_step](1) << ", " << targetF[control_step](2) << ")" << endl;
        cout << "  mass: (" << Mass_param[control_step](0) << ", " << Mass_param[control_step](1) << ", " << Mass_param[control_step](2) << ")" << endl;
        cout << "  D: (" << Damper_param[control_step](0) << ", " << Damper_param[control_step](1) << ", " << Damper_param[control_step](2) << ")" << endl;
        cout << "  K: (" << K_param[control_step](0) << ", " << K_param[control_step](1) << ", " << K_param[control_step](2) << ")" << endl;
        cout << "  F: (" << F_param[control_step](0) << ", " << F_param[control_step](1) << ", " << F_param[control_step](2) << ")" << endl;
#endif
      }
      if( gain_step<gain_change_time.size()-1 && curtime >= gain_change_time[gain_step+1]){
        ++gain_step;
      }
      if( !search_time.empty() && search_step<search_time.size()-1 && curtime >= search_time[search_step+1]){
#ifdef DEBUG_PLUGIN2
        cout << "phase +1  at " << curtime << ", search_time[" << search_step+1 << "]=" << search_time[search_step+1] << endl;
#endif
        ++search_step;
        base_position = body->joint(8)->p();
        base1 = body->joint(8)->p() + body->joint(8)->attitude()*Worm_pos1;
        base2 = body->joint(8)->p() + body->joint(8)->attitude()*Worm_pos2;
        base3 = body->joint(8)->p() + body->joint(8)->attitude()*Worm_pos3;
        base4 = body->joint(8)->p() + body->joint(8)->attitude()*Worm_pos4;
        base_head = body->joint(8)->p() + body->joint(8)->attitude()*attachment_head_pos;
      }
			if( !adjust_time.empty() && adjust_step<adjust_time.size()-1 && curtime >= adjust_time[adjust_step+1]){
				++adjust_step;
				// 中心位置の計算
				if( submode[adjust_step]==1) {
					cout << "estimate center " << endl;
					for(vector<Vector3>::iterator itr = circumference_sample.begin(); itr!=circumference_sample.end(); ++itr){
						cout << *itr << endl;
					}
					cout << "numofrecords" << circumference_sample.size() << endl;
					cout << "adjust_step " << adjust_step << endl;
					cout << "numofrecords[adjust_step]" << numofrecords[adjust_step] << endl;
					if( numofrecords[adjust_step]<3){
					}else if( circumference_sample.size()==3 ){
					}else{
						Vector3 estimated2 = estimate_center(circumference_sample[0],circumference_sample[1],circumference_sample[2],circumference_sample[3]);
						estimated_center.push_back(Vector3(0.05,0.05,0));
						//estimated_center[idxofcenter[adjust_step]].push_back(Vector3(0.03,0.03,0));
					}
				}
			}
      r_control_mode = control_mode[control_step];
      l_control_mode = control_mode[control_step];

      // IMPEDANCE CONTROL
      if( r_control_mode==IMPEDANCE_CONTROL || l_control_mode==IMPEDANCE_CONTROL )
      {
        Vector3 r_dP_wrist;      // desired position of right arm wrist
        Matrix3 r_dR_wrist;      // desired attitude of right arm wrist (rotation matrix)
        Vector3 l_dP_wrist;      // desired position of left arm wrist
        Matrix3 l_dR_wrist;      // desired attitude of left arm wrist (rotation matrix)

        // Impedance control need desired position and attitude.
        vector<double> old_q;
        old_q.resize(body->numJoints());
        for( int i=0; i<body->numJoints(); ++i){
          // 退避
          old_q[i] = body->joint(i)->q();
          // 目標の関節角
          if(stop_flag){
            body->joint(i)->q() = stop_frame[i];
          }else{
            body->joint(i)->q() = frame[i];
          }
        }
        body->calcForwardKinematics();
        r_dP_wrist = body->joint(8)->p();
        r_dR_wrist = body->joint(8)->R();
        l_dP_wrist = body->joint(14)->p();
        l_dR_wrist = body->joint(14)->R();
        for (int i=0; i<body->numJoints(); ++i){
          // 元に戻す
          body->joint(i)->q() = old_q[i];
        }
        body->calcForwardKinematics();

        if( r_control_mode == IMPEDANCE_CONTROL )
        {
          Vector3 r_dP;         // desired position of right hand
          Matrix3 r_dR_hand;    // desired attitude of right hand (rotation matrix)
          Vector3 r_dR;         // desired attitude of right hand (rpy)
          Vector3 r_dF;         // desired force
          Vector3 r_dM;         // desired torque
          Vector6d rArm_qref;
          rArm->wrist2EndEffXform(r_dP_wrist, r_dR_wrist, r_dP, r_dR_hand);
          r_dR = cnoid::rpyFromRot(r_dR_hand);
          if(stop_flag){
            r_dF = VectorXd::Zero(3);
            r_dM = VectorXd::Zero(3);
          } else {
            r_dF = targetF[control_step];
            r_dM = targetM[control_step];
          }
          rArm->update_currforcedata();
//          cout << "--------------- Right arm --------------- " << endl;
          rArm->impedance_control2( r_dP, r_dR, r_dF, r_dM, control_step, stop_flag, init_impedance );
          rArm_qref = rArm->get_qref();  // update q_ref by culculated in impedance_control2()
          reference_q[3] = rArm_qref(0);
          reference_q[4] = rArm_qref(1);
          reference_q[5] = rArm_qref(2);
          reference_q[6] = rArm_qref(3);
          reference_q[7] = rArm_qref(4);
          reference_q[8] = rArm_qref(5);
          IK_q = rArm_qref;
        }
        if( l_control_mode == IMPEDANCE_CONTROL )
        {
          Vector3 l_dP;         // desired position of left hand
          Matrix3 l_dR_hand;    // desired attitude of left hand (rotation matrix)
          Vector3 l_dR;         // desired attitude of left hand (rpy)
          Vector3 l_dF;
          Vector3 l_dM;
          Vector6d lArm_qref;
          lArm->wrist2EndEffXform(l_dP_wrist, l_dR_wrist, l_dP, l_dR_hand);
          l_dR = cnoid::rpyFromRot(l_dR_hand);
          l_dF = VectorXd::Zero(3);
          l_dM = VectorXd::Zero(3);
          lArm->update_currforcedata();
//          cout << "--------------- Left arm --------------- " << endl;
          lArm->impedance_control2( l_dP, l_dR, l_dF, l_dM, control_step, stop_flag, init_impedance );
          lArm_qref = lArm->get_qref();  // update q_ref by culculated in impedance_control2()
          reference_q[9] = lArm_qref(0);
          reference_q[10] = lArm_qref(1);
          reference_q[11] = lArm_qref(2);
          reference_q[12] = lArm_qref(3);
          reference_q[13] = lArm_qref(4);
          reference_q[14] = lArm_qref(5);
        }
        init_impedance = false;
      }
      // 位置制御で力覚センサーの値を見たいときはここを有効化する
      // インピーダンス制御のとき二重に呼ぶと速度などが正しく計算されない
      if( r_control_mode == POSITION_CONTROL || l_control_mode == POSITION_CONTROL){
        rArm->update_currforcedata();
        lArm->update_currforcedata();
      }

      Vector3 WormHead = body->joint(8)->p() + body->joint(8)->attitude()*attachment_head_pos;
      Vector3 curpos = body->joint(8)->p();
      Vector3 jointpos = body->joint(8)->p();
      *pos_trace_temp << curpos(0) << " " << curpos(1) << " " << curpos(2) << " ";
      *pos_trace_temp << jointpos(0) << " " << jointpos(1) << " " << jointpos(2) << endl;
      Vector3 currpy = rpyFromRot(body->joint(8)->attitude());
      vector<double> old_q;
      old_q.resize(body->numJoints());
      for( int i=0; i<body->numJoints(); ++i){
        // 退避
        old_q[i] = body->joint(i)->q();
        // 目標の関節角
        body->joint(i)->q() = original_frame[i];
      }
      std::string fs = " ";
      body->calcForwardKinematics();
      Vector3 refpos = body->joint(8)->p();
      Vector3 refheadpos = body->joint(8)->p() + body->joint(8)->attitude()*attachment_head_pos;
      Vector3 refrpy = rpyFromRot(body->joint(8)->attitude());
      for( int i=0; i<body->numJoints(); ++i){
        // インピーダンス制御で補正した関節角(位置制御の場合は目標の関節角)
        body->joint(i)->q() = reference_q[i];
      }
      body->calcForwardKinematics();
      Vector3 impedance_refpos = body->joint(8)->p();
      Vector3 impedance_refrpy = rpyFromRot(body->joint(8)->attitude());
      for( int i=0; i<body->numJoints(); ++i){
        // インピーダンス制御で補正した関節角(位置制御の場合は目標の関節角)
        if(stop_flag){
          body->joint(i)->q() = stop_frame[i];
        }
      }
      body->calcForwardKinematics();
      Vector3 stop_pos = body->joint(8)->p();
      Vector3 stop_rpy = rpyFromRot(body->joint(8)->attitude());
      for (int i=0; i<body->numJoints(); ++i){
        // 元に戻す
        body->joint(i)->q() = old_q[i];
      }
      body->calcForwardKinematics();

      double  log_displacement;
      double  ld1, ld2, ld3, ld4, ld5;
      // 条件を判定し停止させる
      if(!stop_flag && !search_time.empty() && search_mode[search_step]==1){
        bool captured = false;
        // 無条件に停止
        if(search_type[search_step]==0){
          cout << "stop at " << curtime << endl;
          os() << "stop at " << curtime << endl;
          captured = true;
        }
        // 力がしきい値を越えたら停止
        else if(search_type[search_step]==1){
					Vector3 temp_f = body->joint(8)->attitude()*rArm->rFh_lpf;
//					cout << "direction : " << search_direction[search_step].normalized()(0) << ", " << search_direction[search_step].normalized()(1) << ", " << search_direction[search_step].normalized()(2) << endl;
//					cout << "force : " << rArm->rFh_gc(0) << ", " << rArm->rFh_gc(1) << ", " << rArm->rFh_gc(2) << endl;
//					cout << "force(world) : " << temp_f(0) << ", " << temp_f(1) << ", " << temp_f(2) << endl;
//					cout << search_direction[search_step].normalized().dot(rArm->rFh_gc) << " > " << found_thres[search_step] << endl;
          //if( search_direction[search_step].normalized().dot(rArm->rFh_gc) > found_thres[search_step] ) {
          if( search_direction[search_step].normalized().dot(temp_f) > found_thres[search_step] ) {
            cout << "stop(force) at " << curtime << endl;
						os() << "-----------------------------------------------" << endl;
            os() << "stop(force) at " << curtime << endl;
						// 当たるまでに動いた距離
          //Vector3 pos_increment = body->joint(8)->p() - base_position;
          Vector3 pos_increment = body->joint(8)->p() - reference_position[search_step];
	        base_head = body->joint(8)->p() + body->joint(8)->attitude()*attachment_head_pos;
          Vector3 pos_increment_ee = base_head - reference_position[search_step];
          double displacement = search_direction[search_step].normalized().dot(pos_increment);
          double displacement_ee = search_direction[search_step].normalized().dot(pos_increment_ee);
					//os() << "force " << rArm->rFh_gc(0) << ", " << rArm->rFh_gc(1) << ", " << rArm->rFh_gc(2) << endl;
					os() << "force(world) : " << temp_f(0) << ", " << temp_f(1) << ", " << temp_f(2) << endl;
					//os() << "base " << base_position(0) << ", " << base_position(1) << ", " << base_position(2) << endl;
					os() << "base " << reference_position[search_step](0) << ", " << reference_position[search_step](1) << ", " << reference_position[search_step](2) << endl;
					os() << "direction " << search_direction[search_step](0) << ", " << search_direction[search_step](1) << ", " << search_direction[search_step](2) << endl;
					os() << "current(wrist) " << body->joint(8)->p()[0] << ", " << body->joint(8)->p()[1] << ", " << body->joint(8)->p()[2] << endl;
					os() << "current(ee) " << base_head(0) << ", " << base_head(1) << ", " << base_head(2) << endl;
					os() << "rpy " << rArm->rpy0(0)*57.29578 << ", " << rArm->rpy0(1)*57.29578 << ", " << rArm->rpy0(2)*57.29578 << endl;
					os() << "*** displacement(wrist) = " << displacement << endl;
					os() << "*** displacement(ee) = " << displacement_ee << endl;
            captured = true;
						// 中心補正のための記録中
						cout << "adjust_step = " << adjust_step << endl;
						if( submode[adjust_step]==0 ){
							//circumference_sample[record_idx[adjust_step]] = body->joint(8)->p();
							cout << "record" << endl << body->joint(8)->p() << endl;
							circumference_sample.push_back(body->joint(8)->p());
						}
          }
        }
        // 位置の増分がしきい値を越えたら停止
        else if(search_type[search_step]==2){
          //Vector3 pos_increment = body->joint(8)->p() - base_position;
          Vector3 pos_increment = body->joint(8)->p() - reference_position[search_step];
          double displacement = search_direction[search_step].normalized().dot(pos_increment);
	        base_head = body->joint(8)->p() + body->joint(8)->attitude()*attachment_head_pos;
          Vector3 pos_increment_ee = base_head - reference_position[search_step];
          double displacement_ee = search_direction[search_step].normalized().dot(pos_increment_ee);
          log_displacement = displacement;
          // Wormの途中で見る
          pos_increment = curpos + body->joint(8)->attitude()*Worm_pos1 - base1;
          ld1 = search_direction[search_step].normalized().dot(pos_increment);
          pos_increment = curpos + body->joint(8)->attitude()*Worm_pos2 - base2;
          ld2 = search_direction[search_step].normalized().dot(pos_increment);
          pos_increment = curpos + body->joint(8)->attitude()*Worm_pos3 - base3;
          ld3 = search_direction[search_step].normalized().dot(pos_increment);
          pos_increment = curpos + body->joint(8)->attitude()*Worm_pos4 - base4;
          ld4 = search_direction[search_step].normalized().dot(pos_increment);
          pos_increment = curpos + body->joint(8)->attitude()*attachment_head_pos - base_head;
          ld5 = search_direction[search_step].normalized().dot(pos_increment);
#ifdef DEBUG_PLUGIN2
          cout << "time : " << curtime << endl;
          cout << "pos : " << body->joint(8)->p()(0) << ", " << body->joint(8)->p()(1) << ", " << body->joint(8)->p()(2) << endl;
          //cout << "base_pos : " << base_position(0) << ", " << base_position(1) << ", " << base_position(2) << endl;
          cout << "base_pos : " << reference_position[search_step](0) << ", " << reference_position[search_step](1) << ", " << reference_position[search_step](2) << endl;
          cout << "incre : " << pos_increment(0) << ", " << pos_increment(1) << ", " << pos_increment(2) << endl;
          cout << "direction: " << search_direction[search_step](0) << ", " << search_direction[search_step](1) << ", " << search_direction[search_step](2) << endl;
          double dist = search_direction[search_step].normalized().dot(pos_increment);
          cout << "thres : " << found_thres[search_step] << ",  dist : " << dist << endl;
#endif
          // 測る方向への増分を判定。向きあり。
          if( displacement > found_thres[search_step] ) {
            cout << "stop(position) at " << curtime << endl;
            os() << "stop(position) at " << curtime << endl;
						os() << "base " << reference_position[search_step](0) << ", " << reference_position[search_step](1) << ", " << reference_position[search_step](2) << endl;
						os() << "curpos " << body->joint(8)->p()(0) << ", " << body->joint(8)->p()(1) << ", " << body->joint(8)->p()(2) << endl;
						os() << "displacement = " << displacement << endl;
						os() << "curpos(ee) " << base_head(0) << ", " << base_head(1) << ", " << base_head(2) << endl;
						os() << "displacement(ee) = " << displacement_ee << endl;
						os() << "rpy " << rArm->rpy0(0)*57.29578 << ", " << rArm->rpy0(1)*57.29578 << ", " << rArm->rpy0(2)*57.29578 << endl;
            captured = true;
          }
        }
        if(captured){
          stop_flag = true;
					cout << "stop!" << endl << "keep position " << body->joint(8)->p()(0) << ", " << body->joint(8)->p()(1) << ", " << body->joint(8)->p()(2) << endl;
          for(int i=0; i < body->numJoints(); ++i){
            stop_frame[i] = body->joint(i)->q();
          }
        }
      }

      // 条件を判定し停止を解除する
      if(stop_flag && !search_time.empty() && search_mode[search_step]==2) {
        // 無条件に解除
        if(search_type[search_step]==0){
          cout << "released at " << curtime << endl;
          os() << "released at " << curtime << endl;
          stop_flag = false;
          rArm->prev_stop = false;
        }
        // 力がしきい値を越えたら解除
        else if(search_type[search_step]==1){
          if( search_direction[search_step].normalized().dot(rArm->rFh_gc) > found_thres[search_step]) {
            cout << "released(force) at " << curtime << endl;
            os() << "released(force) at " << curtime << endl;
            stop_flag = false;
            rArm->prev_stop = false;
          }
        }
        // 目標位置が現在位置の近くに来たら解除
        else if(search_type[search_step]==2){
          Vector3 pos_diff = refpos - body->joint(8)->p();
          double displacement = search_direction[search_step].normalized().dot(pos_diff);
#ifdef DEBUG_PLUGIN2
          cout << "time : " << curtime << endl;
          Vector3 pos_diff = refpos - body->joint(8)->p();
          cout << "pos : " << body->joint(8)->p()(0) << ", " << body->joint(8)->p()(1) << ", " << body->joint(8)->p()(2) << endl;
          cout << "refpos : " << refpos(0) << ", " << refpos(1) << ", " << refpos(2) << endl;
          cout << "displacement : " << pos_diff(0) << ", " << pos_diff(1) << ", " << pos_diff(2) << endl;
          cout << "direction: " << search_direction[search_step](0) << ", " << search_direction[search_step](1) << ", " << search_direction[search_step](2) << endl;
          cout << "thres : " << found_thres[search_step] << ",  dist : " << dist << endl;
          cout << "displacement(signed) = " << displacement << endl;
#endif
          // 動き出すときの向きが逆にならないようにする(search_directionの向きに動き出させる)
          if( (displacement >= 0) && (displacement < found_thres[search_step])){
            cout << "released(position) at " << curtime << endl;
            os() << "released(position) at " << curtime << endl;
            stop_flag = false;
            rArm->prev_stop = false;
            init_impedance = true;
          }
        }
      }

      // ログ出力
      // 1: time
      *log << curtime << "  ";
      // 各種position
      *log << refpos(0) << fs << impedance_refpos(0) << fs << curpos(0) << fs << stop_pos(0) << fs << reference_position[search_step](0) << "  ";
      *log << refpos(1) << fs << impedance_refpos(1) << fs << curpos(1) << fs << stop_pos(1) << fs << reference_position[search_step](1) << "  ";
      *log << refpos(2) << fs << impedance_refpos(2) << fs << curpos(2) << fs << stop_pos(2) << fs << reference_position[search_step](2) << "  ";
      // 8,9,10:wristのRPY
      *log << refrpy(0)*r2d << fs << refrpy(1)*r2d << fs << refrpy(2)*r2d << "  ";
      *log << impedance_refrpy(0)*r2d << fs << impedance_refrpy(1)*r2d << fs << impedance_refrpy(2)*r2d << "  ";
      *log << rArm->rpy0(0)*57.29578 << fs << rArm->rpy0(1)*57.29578 << fs << rArm->rpy0(2)*57.29578 << "  ";
      // 11,12,13: velocity
      *log << rArm->dx0(0) << fs << rArm->dx0(1) << fs << rArm->dx0(2) << "  ";
//      *log << rArm->rdXd(0) << fs << rArm->rdXd(1) << fs << rArm->rdXd(2) << "  ";
      // 14,15,16:accel
//      *log << rArm->ddx0(0) << fs << rArm->ddx0(1) << fs << rArm->ddx0(2) << "  ";
      // 17,18,19: 角速度
//      *log << rArm->omega(0) << fs << rArm->omega(1) << fs << rArm->omega(2) << "  ";
      // 23,24,25: 重力補償後の力覚センサー値
      Vector3 Fsensor = body->joint(8)->attitude()*rArm->rFh_gc;
      Vector3 Fsensor_lpf = body->joint(8)->attitude()*rArm->rFh_lpf;
      Vector3 Msensor = body->joint(8)->attitude()*rArm->rMh_gc;
      Vector3 Msensor_lpf = body->joint(8)->attitude()*rArm->rMh_lpf;
//      *log << rArm->rFh_gc(0) << fs << rArm->rFh_gc(1) << fs << rArm->rFh_gc(2) << "  ";
      *log << Fsensor(0) << fs << Fsensor(1) << fs << Fsensor(2) << "  ";
      *log << Fsensor_lpf(0) << fs << Fsensor_lpf(1) << fs << Fsensor_lpf(2) << "  ";
      *log << rArm->ddx0(0) << fs << rArm->ddx0(1) << fs << rArm->ddx0(2) << "  ";
      // 29,30,31: 重力補償後のモーメント
      //*log << rArm->rMh_gc(0) << fs << rArm->rMh_gc(1) << fs << rArm->rMh_gc(2) << "  ";
      *log << Msensor(0) << fs << Msensor(1) << fs << Msensor(2) << "  ";
      *log << Msensor_lpf(0) << fs << Msensor_lpf(1) << fs << Msensor_lpf(2) << "  ";
      // 32: 制御(位置制御/インピーダンス制御)
      string control_mode = r_control_mode==POSITION_CONTROL ? "position_control":"impedance_control";
      *log << control_mode << "  ";
      *log << stop_flag << "  ";
      *log << rArm->rPh_ref(0) << fs << rArm->rPh_ref(1) << fs << rArm->rPh_ref(2) << "  ";
      *log << rArm->rPh(0) << fs << rArm->rPh(1) << fs << rArm->rPh(2) << "  ";
      *log << rArm->rPe_ref(0) << fs << rArm->rPe_ref(1) << fs << rArm->rPe_ref(2) << "  ";
      Vector3 rRe_RPY;
      rRe_RPY = rpyFromRot(rArm->rRe_ref);
      *log << rRe_RPY(0)*57.29578 << fs << rRe_RPY(1)*57.29578 << fs << rRe_RPY(2)*57.29578 << "  ";
      *log << rArm->rPe(0) << fs << rArm->rPe(1) << fs << rArm->rPe(2) << "  ";
      *log << rArm->Dsection(0) << fs << rArm->Ksection(0) << fs << rArm->Fsection(0) << "  ";
      *log << rArm->Dsection(1) << fs << rArm->Ksection(1) << fs << rArm->Fsection(1) << "  ";
      *log << rArm->Dsection(2) << fs << rArm->Ksection(2) << fs << rArm->Fsection(2) << "  ";
      *log << rArm->rdXd(0) << fs << rArm->rdXd(1) << fs << rArm->rdXd(2) << "  ";
      *log << rArm->IK_result << fs << rArm->IKfast_result << "  ";
      *log << log_displacement << "  ";
      *log << ld1 << fs << ld2 << fs << ld3 << fs << ld4 << fs << ld5 << "  ";
			*log << WormHead(0) << fs << WormHead(1) << fs << WormHead(2) << "  ";
			*log << base_head(0) << fs << base_head(1) << fs << base_head(2) << "  ";
//			Vector3 action_r = rArm->rFh_gc.cross(rArm->rMh_gc) / (rArm->rFh_gc.norm() * rArm->rFh_gc.norm());
//			*log << action_r(0) << fs << action_r(1) << fs << action_r(2) << "  ";
			Vector3 rel_r;
			rel_r(0) = -0.25;
			//rel_r(1) = (rel_r(0) * rArm->rFh_lpf(1) - rArm->rMh_gc(2))/rArm->rFh_lpf(0);
			//rel_r(2) = (rArm->rMh_gc(1) + rel_r(0)*rArm->rFh_lpf(2))/rArm->rFh_lpf(0);
			rel_r(1) = (rel_r(0) * rArm->rFh_lpf(1) - rArm->rMh_lpf(2))/rArm->rFh_lpf(0);
			rel_r(2) = (rArm->rMh_lpf(1) + rel_r(0)*rArm->rFh_lpf(2))/rArm->rFh_lpf(0);
			*log << rel_r(0) << fs << rel_r(1) << fs << rel_r(2) << "  ";
			Vector3 line_of_action_src;
			line_of_action_src = body->joint(8)->p() + body->joint(8)->attitude()*rel_r;
			*log << line_of_action_src(0) << fs << line_of_action_src(1) << fs << line_of_action_src(2) << "  ";
			Vector3 line_of_action_vect;
			line_of_action_vect = body->joint(8)->attitude()*rel_r;
			*log << line_of_action_vect(0) << fs << line_of_action_vect(1) << fs << line_of_action_vect(2) << "  ";
      *log << endl;
			*action_log << curtime << "  ";
			*action_log << curpos(0) << fs << curpos(1) << fs << curpos(2) << "  ";
			*action_log << line_of_action_vect(0) << fs << line_of_action_vect(1) << fs << line_of_action_vect(2) << "  ";
			*action_log << line_of_action_src(0) << fs << line_of_action_src(1) << fs << line_of_action_src(2) << "  ";
			*action_log << Fsensor_lpf(0) << fs << Fsensor_lpf(1) << fs << Fsensor_lpf(2) << "  ";
      *action_log << Msensor_lpf(0) << fs << Msensor_lpf(1) << fs << Msensor_lpf(2) << "  ";
      *action_log << rArm->rpy0(0)*r2d << fs << rArm->rpy0(1)*r2d << fs << rArm->rpy0(2)*r2d << "  ";
			*action_log << WormHead(0) << fs << WormHead(1) << fs << WormHead(2) << "  ";
			*action_log << endl;
      *forcesensor_temp << Fsensor(0) << " " << Fsensor(1) << " " << Fsensor(2) << endl;

      *trace << "----------------------------------" << endl;
      *trace << "time : " << curtime << " sec" << endl;
      *trace << "r force data(raw) : " << rArm->fsF_raw(0) << ", " << rArm->fsF_raw(1) << ", " << rArm->fsF_raw(2) << endl;
      *trace << "l force data(raw) : " << lArm->fsF_raw(0) << ", " << lArm->fsF_raw(1) << ", " << lArm->fsF_raw(2) << endl;
      *trace << "r torque data(raw) : " << rArm->fsM_raw(0) << ", " << rArm->fsM_raw(1) << ", " << rArm->fsM_raw(2) << endl;
      *trace << "l torque data(raw) : " << lArm->fsM_raw(0) << ", " << lArm->fsM_raw(1) << ", " << lArm->fsM_raw(2) << endl;
//      *trace << "r force data(gc) : " << rArm->rFh_gc(0) << ", " << rArm->rFh_gc(1) << ", " << rArm->rFh_gc(2) << endl;
      *trace << " r force data(gc) : " << Fsensor(0) << ", " << Fsensor(1) << ", " << Fsensor(2) << endl;
      *trace << " r moment data(gc) : " << Msensor(0) << ", " << Msensor(1) << ", " << Msensor(2) << endl;
      if(r_control_mode==1){
        *trace << " mode= Position Control" << endl;
      }else{
        *trace << " mode= Impedance Control" << endl;
        *trace << "--- translation ---" << endl;
        *trace << " M = " << Mass_param[control_step](0) << ", " << Mass_param[control_step](1) << ", " << Mass_param[control_step](2);
        *trace << "   D = " << Damper_param[control_step](0) << ", " << Damper_param[control_step](1) << ", " << Damper_param[control_step](2);
        *trace << "   K = " << K_param[control_step](0) << ", " << K_param[control_step](1) << ", " << K_param[control_step](2);
        *trace << "   F=" << F_param[control_step](0) << ", " << F_param[control_step](1) << ", " << F_param[control_step](2);
        *trace << "   Force_ref = " << targetF[control_step](0) << ", " << targetF[control_step](1) << ", " << targetF[control_step](2) << endl;
        *trace << "--- rotation ---" << endl;
        *trace << " M = " << rotMass_param[control_step](0) << ", " << rotMass_param[control_step](1) << ", " << rotMass_param[control_step](2);
        *trace << "   D = " << rotDamper_param[control_step](0) << ", " << rotDamper_param[control_step](1) << ", " << rotDamper_param[control_step](2);
        *trace << "   K = " << rotK_param[control_step](0) << ", " << rotK_param[control_step](1) << ", " << rotK_param[control_step](2);
        *trace << "   F=" << rotF_param[control_step](0) << ", " << rotF_param[control_step](1) << ", " << rotF_param[control_step](2);
        *trace << "   moment_ref = " << targetM[control_step](0) << ", " << targetM[control_step](1) << ", " << targetM[control_step](2) << endl;
      }
      *trace << "Pgain: ";
      for(int i=0; i<pgain[gain_step].size(); ++i){
        *trace << pgain[gain_step][i] << ",";
      }
      *trace << endl << "Dgain: ";
      for(int i=0; i<dgain[gain_step].size(); ++i){
        *trace << dgain[gain_step][i] << ",";
      }
      *trace << endl;
//      *trace << "r torque data(gc) : " << rArm->rMh_gc(0) << ", " << rArm->rMh_gc(1) << ", " << rArm->rMh_gc(2) << endl;
//      *trace << "l force data(gc) : " << lArm->rFh_gc(0) << ", " << lArm->rFh_gc(1) << ", " << lArm->rFh_gc(2) << endl;
//      *trace << "l torque data(gc) : " << lArm->rMh_gc(0) << ", " << lArm->rMh_gc(1) << ", " << lArm->rMh_gc(2) << endl;

      if(stop_flag) {
  			// keep stopped frame pose
  			for (int i = 0; i < body->numJoints(); ++i) {
  				Link* joint = body->joint(i);
  				double q_ref =stop_frame[i];
  				double q = joint->q();

  				double dq_ref = 0;
  				double dq = (q - q0[i]) / timeStep();
  				joint->u() = (q_ref - q) * pgain[gain_step][i] + (dq_ref - dq) * dgain[gain_step][i];
  				q0[i] = q;
  			}
      }else{
  			for (int i = 0; i < body->numJoints(); ++i) {
  				Link* joint = body->joint(i);
          double q_ref = reference_q[i];
  				double q = joint->q();

          double dq_ref = (q_ref - reference_q0[i]) / timeStep();
  				double dq = (q - q0[i]) / timeStep();
  				joint->u() = (q_ref - q) * pgain[gain_step][i] + (dq_ref - dq) * dgain[gain_step][i];
  				q0[i] = q;
          reference_q0[i] = q_ref;
  			}
  			oldFrame = frame;
#ifdef DEBUG_PLUGIN2
        cout << "----------------------------------------------------------------------------" << endl;
#endif

      }
			return true;
		} else {
      if(!output_file){
        ifstream ifs((executableTopDirectory()+"/position_traj_temp.csv").c_str());
        ofstream ofs((executableTopDirectory()+"/position_traj.csv").c_str(),ios::trunc);
        ofs << ifs.rdbuf();
        remove((executableTopDirectory()+"/position_traj_temp.csv").c_str());
        ifstream ifs2((executableTopDirectory()+"/forcesensor_temp.csv").c_str());
        ofstream ofs2((executableTopDirectory()+"/forcesensor.csv").c_str(),ios::trunc);
        ofs2 << ifs2.rdbuf();
        remove((executableTopDirectory()+"/forcesensor_temp.csv").c_str());
        output_file=true;
      }
			// keep last frame pose
			for (int i = 0; i < body->numJoints(); ++i) {
				Link* joint = body->joint(i);
				double q_ref =oldFrame[i];
				double q = joint->q();

				double dq_ref = 0;
				double dq = (q - q0[i]) / timeStep();
				joint->u() = (q_ref - q) * pgain[gain_step][i] + (dq_ref - dq) * dgain[gain_step][i];
				q0[i] = q;
			}
		}
		return false;
	}

	void log_header(std::ofstream *logfile){
    *logfile << "#time,";
    *logfile << "ref_x,impedance_ref_pos_x,curpos_x,stop_pos_x,basepos_x,";
    *logfile << "ref_y,impedance_ref_pos_y,curpos_y,stop_pos_y,basepos_y,";
    *logfile << "ref_z,impedance_ref_pos_z,curpos_z,stop_pos_z,basepos_z,";
    *logfile << "ref_roll,ref_pitch,ref_yaw,";
    *logfile << "impedance_roll,impedance_pitch,impedance_yaw,";
    *logfile << "roll,pitch,yaw,";
    *logfile << "v_x,v_y,v_z,";
    *logfile << "f_x,f_y,f_z,";
    *logfile << "f_x(LPF),f_y(LPF),f_z(LPF),";
    *logfile << "accel_x,accel_y,accel_z,";
//    *logfile << "raw_f_x,raw_f_y,raw_f_z,";
//    *logfile << "raw_moment_x,raw_moment_y,raw_moment_z,";
    *logfile << "moment_x,moment_y,moment_z,";
    *logfile << "moment_x(LPF),moment_y(LPF),moment_z(LPF),";
    *logfile << "control_mode,";
    *logfile << "stop,";
    *logfile << "Ph_ref_x,Ph_ref_y,Ph_ref_z,";
    *logfile << "Ph_x,Ph_y,Ph_z,";
    *logfile << "Pe_ref_x,Pe_ref_y,Pe_ref_z,";
    *logfile << "Re_ref_x,Re_ref_y,Re_ref_z,";
    *logfile << "Pe_x,Pe_y,Pe_z,";
    *logfile << "Dsection_x,Ksection_x,Fsection_x,";
    *logfile << "Dsection_y,Ksection_y,Fsection_y,";
    *logfile << "Dsection_z,Ksection_z,Fsection_z,";
    *logfile << "rdXd_x,rdXd_y,rdXd_z,";
    *logfile << "ikresult,ikfast_result,";
    *logfile << endl;
	}

	void record_log(double time, HRP2STEP1Arm *Arm, std::ofstream *logfile){
		string fs = ",";
		*logfile << time;

		*logfile << Arm->rpy0(0)*r2d << fs << Arm->rpy0(1)*r2d << fs << Arm->rpy0(2)*r2d << fs;
		*logfile << endl;
	}

	Vector3 estimate_center(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4){
		Vector3 d = Vector3(1,0,0);
		Vector3 q1 = p1 - p1.dot(d)*d;
		Vector3 q2 = p2 - p2.dot(d)*d;
		Vector3 q3 = p3 - p3.dot(d)*d;
		Vector3 q4 = p4 - p4.dot(d)*d;
		Vector3 n1 = q2 - q1;
		Vector3 n2 = q4 - q3;
		Matrix3d A(3,3);
		A << n1.transpose(), n2.transpose(), d.transpose();
		Vector3 b, x;
		b << (q1+q2).dot(n1)/2, (q3+q4).dot(n2)/2, q1.dot(d);
		FullPivLU<Matrix3d> solver(A);
		x = solver.solve(b);
		cout << "q1:" << endl << q1 << endl;
		cout << "q2:" << endl << q2 << endl;
		cout << "q3:" << endl << q3 << endl;
		cout << "q4:" << endl << q4 << endl;
		cout << "n1.transpose" << endl << n1.transpose() << endl;
		cout << "n2.transpose" << endl << n2.transpose() << endl;
		cout << "A" << endl << A << endl;
		cout << "b" << endl << b << endl;
		cout << "x:" << endl << x << endl;

		cout << p1(1) << ", " << p1(2) << endl;
		cout << p2(1) << ", " << p2(2) << endl;
		cout << p3(1) << ", " << p3(2) << endl;
		cout << p4(1) << ", " << p4(2) << endl;
		cout << x(1) << ", " << x(2) << endl;
		*center_estimate_log << p1(1) << ", " << p1(2) << endl;
		*center_estimate_log << p2(1) << ", " << p2(2) << endl;
		*center_estimate_log << p3(1) << ", " << p3(2) << endl;
		*center_estimate_log << p4(1) << ", " << p4(2) << endl;
		*center_estimate_log << x(1) << ", " << x(2) << endl;

		return x;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ImpedanceController)

//============================================================================================
// HRP2STEP1Arm
//============================================================================================
HRP2STEP1Arm::HRP2STEP1Arm(std::string name_in, BodyPtr body_in, unsigned int num_q0_in, double period_in, float ang_limits_in[6][5], Vector3 ePh_in, Matrix3 eRh_in, Vector3 hPfs_in, Matrix3 hRfs_in)
  :f_moving(false), f_reached(false),
   f_gc(false),     f_gc_init(false),  step_gc(0),
   mass(0.0),       GACC(9.8),         wait_step(200),
   /*fs(0),*/       step_fs(0),        max_step_fs(500),
   MAX_JVEL(0.5),   TIME_LIMIT(100.0), MIN_PERIOD(1.0e-9), TOLERANCE(1.0e-6)
{
#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Entering HRP2STEP1Arm constructor" << std::endl;
#endif

  // Save to local members
  name = name_in;             // Right or left arm
  body = body_in;             // Body object extracted from ModelLoader
  NUM_q0 = num_q0_in;         // Which joint is the starting joint. Right arm=3, Left arm=9
  DEL_T = period_in;          // Time step set at 0.0005 secs in calling function
  for(int i = 0; i < ARM_DOF; i++)
  {
    for(int j = 0; j < 5; j++)
    {
      ang_limits[i][j] = ang_limits_in[i][j];     // Set angle limits
    }
  }

  // Arms' Current state
  ePh = ePh_in;         // Wrist 2 End Effector Left: <0.0715, 0.0, 0.0>, Right: <-0.12,0.0,0.0>
  eRh = eRh_in;         // Equivalent Rotation matrix. Set to the identity matrix.
  hPfs = hPfs_in;       //

  hRfs = hRfs_in;       // FUKUMOTO added (2014.12.24)

#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Setting body links." << std::endl;
#endif

  // Set Body linksOB
  Link* S0 = body->joint(CHEST_JOINT0);
  Link* W6;
  if(NUM_q0 == RARM_JOINT0){
    W6 = body->joint(RARM_JOINT5); // Is it joint or link?
  } else {
    W6 = body->joint(LARM_JOINT5); // Is it joint or link?
  }
  x0 = W6->p();
  dx0 = Vector3(0,0,0);
  ddx0 = Vector3(0,0,0);
  rpy0 = rpyFromRot(W6->attitude());
  fsF_accum = Vector3(0,0,0);
  fsM_accum = Vector3(0,0,0);
	fsF_prev[0] = Vector3(0,0,0);
	fsF_prev[1] = Vector3(0,0,0);
	fsF_prev[2] = Vector3(0,0,0);
	fsF_prev[3] = Vector3(0,0,0);
	fsM_prev[0] = Vector3(0,0,0);
	fsM_prev[1] = Vector3(0,0,0);
	fsM_prev[2] = Vector3(0,0,0);
	fsM_prev[3] = Vector3(0,0,0);
  frame_cnt = 0;
  Dsection = Vector3(0,0,0);
  Ksection = Vector3(0,0,0);
  Fsection = Vector3(0,0,0);
  prev_stop = false;
	gc_gravaccel = Vector3(0,0,-9.80665);
	gc_M = 1;
	gc_cm = Vector3(0,0,0);
	force_resolution = 0;
	moment_resolution = 0;

  if(!S0)
    std::cerr << "S0 is null\n";

  if(!W6)
    std::cerr << "W6 is null\n";
#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Getting the joint path." << std::endl;
#endif

//  m_path = body->getJointPath(S0,W6);>>-// choreonoid 1.0~1.3>>-// Base to Wrist Path
  m_path = boost::make_shared< JointPath >(S0,W6);   // choreonoid 1.4>>->-// Base to Wrist Path>

#ifdef DEBUG_PLUGIN2
  cout << "Joint path:" << m_path->numJoints() << std::endl;
  cerr << m_path->joint(0)->name() << std::endl;
  cerr << m_path->joint(1)->name() << std::endl;
  cerr << m_path->joint(2)->name() << std::endl;
  cerr << m_path->joint(3)->name() << std::endl;
  cerr << m_path->joint(4)->name() << std::endl;
  cerr << m_path->joint(5)->name() << std::endl;
#endif

  // Inverse Kinematics gains for
  ikGains[0] >= 0.3;
  ikGains[1] >= 0.6;
  ikGains[2] >= 0.7;
  ikGains[3] >= 0.8;

  ikRot[0] >= 1e-2;
  ikRot[1] >= 1e-7;
  ikRot[2] >= 1e-8;
  ikRot[3] >= 1e-10;

  ikTrans[0] >= 1e-2;
  ikTrans[1] >= 1e-7;
  ikTrans[2] >= 1e-8;
  ikTrans[3] >= 1e-10;

  // Program Design Parameters
  p_param.method = CONSTANT_VELOCITY;  // Interpolation Method: Velocity
  p_param.max_jvel = MAX_JVEL;         // With MAX_JVEL as maximum joint velocity limit = 0.5
  controlmode = NOCONTROL;             // One of six control modes. No control.

  //************************** Pivot Approach ************************************************/
  // Takes the following parameters int NUM_q0, Vector3 base2endEffectorPos, Matrix3 base2endEffectorRot, MatrixXd jacobian, double curr_time, vector6 curr_force
  Vector3 pos = Vector3::Zero(3);
  Matrix3 rot;//x(0);
  double momentGainFactor = 10.0;

#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Getting current position and attitude values" << std::endl;
  std::cerr << "HRP2STEP1Arm(): Retrieving the position" << std::endl;
#endif
  // HOMING Position and rotation matrix for right arm's WRSIT
  // Right arm's last joint is joint 8 (index starts at 0, chest pan tilt...)
  pos=m_path->joint(5)->p(); // instead of RARM_JOINT5 i had 5 before.
#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Position retrieved" << std::endl;
  std::cerr << "HRP2STEP1Arm(): Retrieving the attitude." << std::endl;
#endif
  //rot=m_path->joint(5)->attitude(); >->-// used in linux simulation
//  rot=m_path->joint(5)->segmentAttitude();    // Instead of RARM_JOINT5 I had a 5 before.
  rot=m_path->joint(5)->R();    // FUKUMOTO 2015.01.05
#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Attitude retrieved." << std::endl;
  std::cerr << "HRP2STEP1Arm(): Allocating AssemblyStrategy" << std::endl;
#endif

  // Pivot Approach variables
//  PA = new AssemblyStrategy(NUM_q0,pos,rot,ePh,momentGainFactor);>// Wrist position, Wrist Rotation Matrix, Wrist2EndEffXform

#ifdef DEBUG_PLUGIN2
  std::cerr << "HRP2STEP1Arm(): Finished allocating AssemblyStrategy" << std::endl;
#endif

  // Sensors>-// Todo: sensors should be limited to sensors mounted on its arm.
  forceSensors = body->devices();
  rateGyroSensors = body->devices();
  accelSensors = body->devices();
  std::cout << forceSensors.size() << " force sensors are found." << endl;
  std::cout << rateGyroSensors.size() << " rate gyro sensors are found." << endl;
  std::cout << accelSensors.size() << " accel sensors are found." << endl;

#ifdef DEBUG_PLUGIN2
  std::cout << "HRP2STEP1Arm: exit constructor"  << std::endl;
#endif

}

// Destructor
HRP2STEP1Arm::~HRP2STEP1Arm()
{
//>-delete PA;
  fclose(fp1);
  fclose(fp2);
  fclose(fp3);
}

bool HRP2STEP1Arm::impedance_control2( Vector3 rXd_des, Vector3 rRd_des, Vector3 rFd_des, Vector3 rMd_des, int control_step, bool stop_flag, bool init_impedance )
{
  if(controlmode != IMP_CONTROL)
  {
    rPh_ref = rPh;          // Set the base2wrist parameters as ref params: position
    rRh_ref = rRh;
    controlmode = IMP_CONTROL;    // Change the control mode to impedance control

    rdXd << 0,0,0;
    rdRd << 0,0,0;
  }

  // Inertia and damping matrix coefficients for position/orientation
//  Matrix3 Kt, Kr;
//  Mt << 10, 0, 0, 0, 10, 0, 0, 0, 10;
//  Dt << 300, 0, 0, 0, 300, 0, 0, 0, 300;
//  Kt << 500, 0, 0, 0, 500, 0, 0, 0, 500;
  Mt = Mass_param[control_step].asDiagonal();
  Dt = Damper_param[control_step].asDiagonal();
  Kt = K_param[control_step].asDiagonal();
//  Mr << 5, 0, 0, 0, 5, 0, 0, 0, 5;
//  Dr << 75, 0, 0, 0, 75, 0, 0, 0, 75;
//  Kr << 100, 0, 0, 0, 100, 0, 0, 0, 100;
  Mr = rotMass_param[control_step].asDiagonal();
  Dr = rotDamper_param[control_step].asDiagonal();
  Kr = rotK_param[control_step].asDiagonal();

  if(!prev_stop && stop_flag){
    rdXd = Vector3::Zero(3);
    rdRd = Vector3::Zero(3);
    rPh_ref = rPh;
    rRh_ref = rRh;
  }
//  Matrix3 Ft, Fr;
  // original; Ft=2, Fr=1
//  Ft << 2, 0, 0, 0, 2, 0, 0, 0, 2;
//  Fr << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  if(stop_flag){
    Dt << 0,0,0, 0,0,0, 0,0,0;
    Dr << 0,0,0, 0,0,0, 0,0,0;
    Kt << 0,0,0, 0,0,0, 0,0,0;
    Kr << 0,0,0, 0,0,0, 0,0,0;
    Ft << 0,0,0, 0,0,0, 0,0,0;
    Fr << 0,0,0, 0,0,0, 0,0,0;
    prev_stop = true;
  } else {
    Ft = F_param[control_step].asDiagonal();
    Fr = rotF_param[control_step].asDiagonal();
  }
  if(init_impedance){
    rPh_ref = rPh;
    rRh_ref = rRh;
    rdXd = Vector3::Zero(3);
    rdRd = Vector3::Zero(3);
    Ft << 0,0,0, 0,0,0, 0,0,0;
    Fr << 0,0,0, 0,0,0, 0,0,0;
  }

  // Desired position and rotation data
  for(int i=0; i<9; i++)
  {
    double t;
    fin_rpos >> t;
  }
  // 力とモーメントをworld座標に直す
  Vector3 rFh_w = m_path->joint(5)->attitude()*rFh_gc;
  Vector3 rMh_w = m_path->joint(5)->attitude()*rMh_gc;
  for(int i=0;i<3; i++)
  {
    Dsection(i) = -Dt(i,i)*rdXd(i)/Mt(i,i)*DEL_T;
    Ksection(i) = -Kt(i,i)*(rPh(i) - rXd_des(i))/Mt(i,i)*DEL_T;
    Fsection(i) = Ft(i,i)*(rFh_w(i) - rFd_des(i))/Mt(i,i)*DEL_T;
#ifdef DEBUG_PLUGIN2
    cout << "Dt(i)=" << Dt(i,i) << ", rdXd(i)=" << rdXd(i) << endl;
    cout << "Kt(i)=" << Kt(i,i) << ", rPh(i)=" << rPh(i) << ", rXd_des(i)=" << rXd_des(i) << endl;
    cout << "Ft(i)=" << Ft(i,i) << ", rFh_w(i)=" << rFh_w(i) << ", rFd_des(i)=" << rFd_des(i) << endl;
#endif
//    rdXd(i) = rdXd(i) - (Dt(i,i)*rdXd(i) + Kt(i,i)*(rPh(i) - rXd_des(i)) - Ft(i,i)*(rFh_gc(i) - rFd_des(i)))/Mt(i,i)*DEL_T;
    rdXd(i) = rdXd(i) - (Dt(i,i)*rdXd(i) + Kt(i,i)*(rPh(i) - rXd_des(i)) - Ft(i,i)*(rFh_w(i) - rFd_des(i)))/Mt(i,i)*DEL_T;

#ifdef DEBUG_PLUGIN2
    cout << "rRh: " << rpyFromRot(rRh)(i) << ", rRd_des: " << rRd_des(i) << endl;
#endif
    if( rpyFromRot(rRh)(i) - rRd_des(i) > M_PI ){
      rRd_des(i) += 2*M_PI;
    } else if( rpyFromRot(rRh)(i) - rRd_des(i) < -M_PI ){
      rRd_des(i) -= 2*M_PI;
    }
//    rdRd(i) = rdRd(i) - (Dr(i,i)*rdRd(i) + Kr(i,i)*(rpyFromRot(rRh)(i) - rRd_des(i)) - Fr(i,i)*(rMh_gc(i) - rMd_des(i)))/Mr(i,i)*DEL_T;
    rdRd(i) = rdRd(i) - (Dr(i,i)*rdRd(i) + Kr(i,i)*(rpyFromRot(rRh)(i) - rRd_des(i)) - Fr(i,i)*(rMh_w(i) - rMd_des(i)))/Mr(i,i)*DEL_T;

    rRd(i) = rdRd(i)*DEL_T;      //rRd=ASMRo*rRd;
  }
//  cout << "rRh(rpy): " << rpyFromRot(rRh)(0) << ", " << rpyFromRot(rRh)(1) << ", " << rpyFromRot(rRh)(2) << endl;
//  cout << "rRd_des: " << rRd_des(0) << ", " << rRd_des(1) << ", " << rRd_des(2) << endl;
  rPh_ref += rdXd*DEL_T;
  rRh_ref = rotFromRpy( rpyFromRot(rRh_ref) + rdRd*DEL_T  );

  // 目標手先位置・姿勢の計算 (rPh_ref, rRh_ref --> rPe_ref, rRe_ref --> q_ref)
  calc_rPe_rRe(rPh_ref, rRh_ref, rPe_ref, rRe_ref);
  bool ret;
  Vector6d q_ref_tmp;
  ret = calc_qref(rPe_ref, rRe_ref, q_ref_tmp);
  if(ret){
    q_ref = q_ref_tmp;
  } else {
#ifdef DEBUG_PLUGIN2
    std::cout << "calc_qref() returned false. (in impedance_control2())" << endl;
#endif
  }

  // calc_qrefの先でFKとリンク情報の設定をするために必要
  Vector6d temp;      // FUKUMOTO added 20150105
  calc_qref(rPe,rRe,temp);  // FUKUMOTO added 20150105

#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref = " << q_ref   << std::endl;
  std::cout << "q = "   << q     << std::endl;
#endif
  return ret;
}

//--------------------------------------------------------------------------------//
// protected functions
//--------------------------------------------------------------------------------//
bool HRP2STEP1Arm::calc_qref(Vector3 rPe_ref_in, Matrix3 rRe_ref_in, Vector6d &q_out)
{
#ifdef DEBUG_PLUGIN2
  std::cout << "[cacl_qref]" << rPe_ref_in << endl << rRe_ref_in << std::endl;
#endif
  bool ikRet;

//  ikRet = m_path->calcInverseKinematics(rPe_ref_in,rRe_ref_in);
  IK_result = m_path->calcInverseKinematics(rPe_ref_in,rRe_ref_in);
  IKfast_result = grasp::PlanBase::instance()->arm(0)->IK_arm(rPe_ref_in, grasp::PlanBase::instance()->arm(0)->arm_path->joint(5)->calcRfromAttitude(rRe_ref_in));
#ifdef DEBUG_PLUGIN2
  if(!IK_result)
  {
    std::cout << "calcInverseKinematics() returned false!" << endl;
    std::cout << "rPe_ref_in:" << endl << rPe_ref_in << endl;
    std::cout << "rRe_ref_in:" << endl << rRe_ref_in << endl;
  }
  if(IKfast_result) cout << "IK_arm solved(ikfast)" << endl;
  else cout << "IK_arm failed(ikfast)" << endl;

  cout << "numeric : " << ikRet << ",  ikfast : " << ikRet2 << endl;
  cout << "joint num : " << grasp::PlanBase::instance()->arm(0)->arm_path->numJoints() << endl;
  cout << "q_ref: " ;
  for (int i =0; i<ARM_DOF; i++){
    std::cout << (body->joint(NUM_q0 + i)->q() )*180/M_PI << ", ";
    cout << grasp::PlanBase::instance()->arm(0)->arm_path->joint(i+1)->q()*180/M_PI << endl;
  }
  cout << endl;
#endif

  if(IK_result){
    for(int i=0; i<ARM_DOF; i++) {
      q_out[i] = body->joint(NUM_q0 + i)->q();
    }
    return true;
  }
  else if(IKfast_result){
    for(int i=0; i<ARM_DOF; i++) {
      q_out[i] = grasp::PlanBase::instance()->arm(0)->arm_path->joint(i+1)->q();
    }
    return true;
  }
  else{
    return false;
  }
}

//--------------------------------------------------------------------------------//
// calc_rPe_rRe
// Convert from robot2endeffector to robot2wrist position and rotation
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::calc_rPe_rRe(Vector3 rPh_in, Matrix3 rRh_in, Vector3 &rPe_out, Matrix3 &rRe_out)
{
  Matrix3 rRe_tmp;
  rRe_tmp = rRh_in * (eRh).transpose();    // original
  rRe_out = rRe_tmp;
  rPe_out = rPh_in - rRe_tmp * ePh;
}

//--------------------------------------------------------------------------------//
// setResolution
// set resolution of force sensor
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::setResolution(double f_resol, double m_resol)
{
	force_resolution = f_resol;
	moment_resolution = m_resol;
}

//--------------------------------------------------------------------------------//
// get_raw_forces
// Extract the raw forces from the forcetorque sensor
//
//  NOTICE: read the value of only 0-th force sensor
//
//--------------------------------------------------------------------------------//
//bool HRP2STEP1Arm::get_raw_forces(double f_out[6])
bool HRP2STEP1Arm::get_raw_forces()
{
  body->calcForwardKinematics();

  // read force sensor output
  int id;          // right(0) or left(1)
  if(NUM_q0 == RARM_JOINT0){
    id = 0;
  } else {
    id = 1;
  }
  ForceSensor *forceSensor = forceSensors.get(id);   // left Arm
  fsF_raw = forceSensor->f().block(0,0,3,1);
  fsM_raw = forceSensor->f().block(3,0,3,1);
	if( force_resolution !=0 || moment_resolution !=0 ){
		for(int i=0; i<3; ++i){
			fsF_raw(i) = floor(fsF_raw(i)/force_resolution)*force_resolution;
			fsM_raw(i) = floor(fsM_raw(i)/moment_resolution)*moment_resolution;
		}
	}
  return true;
}

//--------------------------------------------------------------------------------//
// setGC
// -  set Mass and position of center of Mass for Gravity compensation
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::setGC(Vector3 grav_accel, double M, Vector3 cm)
{
	gc_gravaccel = grav_accel;
	gc_M = M;
	gc_cm = cm;
}
//--------------------------------------------------------------------------------//
// calc_gc_forces
// -  gravity compensation
//--------------------------------------------------------------------------------//
bool HRP2STEP1Arm::calc_gc_forces()
{
  double M;
//  double g = 9.80665;
  Vector3 cm_hand;
//  if(NUM_q0 == RARM_JOINT0){
// 元のモデル用
//    M = 1.0526;
//    cm_hand = Vector3(-0.048429, -0.00051637, -0.038863);
// 実物のデータを反映したモデル用
//      M = 1.24186;
//      cm_hand = Vector3(-0.040969, -0.041044, -0.032983);
//  } else {
//    M = 1.052;
//    cm_hand = Vector3(0,0,0);
//  }
	M = gc_M;
	cm_hand = gc_cm;
//  Vector3 gravity = Vector3(0, 0, -M*g);
//  Vector3 gravity = Vector3(M*g, 0, 0);
	Vector3 gravity = M*gc_gravaccel;
  Vector3 gravity_force;
  gravity_force = m_path->joint(5)->attitude().transpose()*gravity;
  Vector3 cur_pos = m_path->joint(5)->p();
  Vector3 cur_veloc = (cur_pos - x0)/DEL_T;
  Vector3 cur_accel = (cur_veloc - dx0)/DEL_T;
  // 角速度
  // wrist の角速度
  Vector3 rpy = rpyFromRot(m_path->joint(5)->attitude());
  Vector3 rpy_temp;
  for(int i=0; i<3; ++i){
    if( rpy(i) - rpy0(i) > M_PI){
      rpy_temp(i) = rpy(i) - rpy0(i) - 2*M_PI;
    }
    else if( rpy(i) - rpy0(i) < -M_PI){
      rpy_temp(i) = rpy(i) - rpy0(i) + 2*M_PI;
    }
    else {
      rpy_temp(i) = rpy(i) - rpy0(i);
    }
    omega(i) = rpy_temp(i) / DEL_T;
  }
  rpy0 = rpy;

  x0 = cur_pos;
  dx0 = cur_veloc;
  ddx0 = cur_accel;
  // 力覚センサーの重力補償(慣性力もキャンセル)
  rFfs_gc = fsF_raw - gravity_force + M*m_path->joint(5)->attitude().transpose()*ddx0;
  Matrix3 Torque;
  Torque << 0, gravity_force(2), -gravity_force(1), -gravity_force(2), 0, gravity_force(0), gravity_force(1), -gravity_force(0), 0;
  Vector3 torque_offset;
  torque_offset = Torque*cm_hand;
  rMfs_gc = fsM_raw - torque_offset;
  // low pass filter
  if(frame_cnt==2){
    fsF_accum = 0.99*fsF_accum + 0.01*rFfs_gc;
    fsM_accum = 0.99*fsM_accum + 0.01*rMfs_gc;
  }else{
    // 最初のフレームは力覚センサーの値が0のため2フレーム目から
    fsF_accum = rFfs_gc;
    fsM_accum = rMfs_gc;
    ++frame_cnt;
  }
  //rFh_lpf = fsF_accum;
  //rMh_lpf = fsM_accum;
	rFh_lpf = (fsF_prev[0]+fsF_prev[1]+fsF_prev[2]+fsF_prev[3]+rFfs_gc)/5;
	fsF_prev[3] = fsF_prev[2];
	fsF_prev[2] = fsF_prev[1];
	fsF_prev[1] = fsF_prev[0];
	fsF_prev[0] = rFfs_gc;
	rMh_lpf = (fsM_prev[0]+fsM_prev[1]+fsM_prev[2]+fsM_prev[3]+rMfs_gc)/5;
	fsM_prev[3] = fsM_prev[2];
	fsM_prev[2] = fsM_prev[1];
	fsM_prev[1] = fsM_prev[0];
	fsM_prev[0] = rMfs_gc;
  return true;
}

//--------------------------------------------------------------------------------//
// calc_gc_forces_at_hand()
// Compute the arm's force and moment gravitational components at the hand
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::calc_gc_forces_at_hand()
{
  // 手先での力・モーメントに変換 // Converted to forces and moments in the hand
  rFh_gc = rFfs_gc;          // Arm force gravitational component
  rMh_gc = rMfs_gc + (rRh*hPfs).cross(rFfs_gc);      // Arm moment gravitational component
//  rFh_gc = Vector3(0,0,0);
//  rMh_gc = Vector3(0,0,0);
}

//--------------------------------------------------------------------------------//
// * update_currposdata()
// * Update current arm configuration by:
// * First, retreving:
// *   The wrist position,
// *   The base2wrist rotation, and
// *   The current joint angles of the body
// * Second, by computing the:
// *   base2EndEffectorPosition
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::update_currposdata()
{
  // base2wrist position. Homing pos: <0.3019, -0.1724, 0.064>
  rPe = body->joint(NUM_q0 + ARM_DOF -1)->p();
#ifdef DEBUG_PLUGIN2
  cerr << "HRP2STEP1Arm::updated_currposdata - rPe is: " << rPe(0) << " " << rPe(1) << " " << rPe(2) << std::endl;
#endif
  // base2wrist rotation matrix
  rRe = body->joint(NUM_q0 + ARM_DOF -1)->R();  // calcInverseKinematics() works with R() not attitude(), FUKUMOTO

  // Current joint angles
  for(int i=0; i<ARM_DOF; i++)
    q[i] = body->joint(NUM_q0 + i)->q();

  wrist2EndEffXform(/*in*/rPe, /*in*/rRe, /*out*/rPh, /*out*/rRh);
#ifdef DEBUG_PLUGIN2
  cerr << "HRP2STEP1Arm::updated_currposdata - rPh is: " << rPh(0) << " " << rPh(1) << " " << rPh(2) << std::endl;
#endif
}

#if 0
void HRP2STEP1Arm::savedata()
{
  // Initialization
  Vector3 tmp = rpyFromRot(rRh);          // Get RPY terms for the wrist

  // Hand (not arm) Gravity Compensation terms for force and moment
  fprintf(fp1,"%f %f %f %f %f %f\n", rFh_gc[0], rFh_gc[1], rFh_gc[2], rMh_gc[0], rMh_gc[1], rMh_gc[2]);
  fprintf(fp2,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",rPh[0], rPh[1], rPh[2],    // Position translation from base 2 end effector
     tmp[0], tmp[1], tmp[2],    // RPY terms
     rRh(0,0),rRh(0,1),rRh(0,2),  // Rotation matrix terms for base 2 end effector
     rRh(1,0),rRh(1,1),rRh(1,2),
     rRh(2,0),rRh(2,1),rRh(2,2));
}
#endif

//--------------------------------------------------------------------------------//
// * wrist2EndEffXform()
// * Takes the base2wrist position and rotation and transforms them to base2endeff
// * position and orientation
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::wrist2EndEffXform(/*in*/Vector3 rPe, /*in*/Matrix3 rRe, /*out*/Vector3& rPh, /*out*/Matrix3& rRh)
{
  // base2EndEffector
  // Caculation rPh & rRh from rPe & rRe
  rRh = rRe * eRh;            // base2EndEffRot=base2wristRot*wrist2EndEffRot
  rPh = rPe + rRe * ePh;          // base2EndEffPos=base2wristPos + transformed wrist2EndEffPos

}

//--------------------------------------------------------------------------------//
// wrist2EndEffXform()
// Takes the base2wrist position and rotation and transforms them to base2endeff
// position and orientation
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::EndEff2wristXform(/*in*/Vector3 rPh, /*in*/Matrix3 rRh, /*out*/Vector3& rPe)
{
  // base2EndEffector
  // Caculation rPh & rRh from rPe & rRe
  rPe = rPh - rRe * ePh;          // base2EndEffPos=base2wristPos + transformed wrist2EndEffPos
}

//--------------------------------------------------------------------------------//
 // get_qref()
 // Retrieve reference joint angle in degrees
//--------------------------------------------------------------------------------//
Vector6d HRP2STEP1Arm::get_qref()
{
#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref in HRP2STEP1Arm::get_qref: " << q_ref*180/M_PI << std::endl;
#endif

  return q_ref;
}

//--------------------------------------------------------------------------------//
 // update_currforcedata()
 // Return latest force and moment data.
 // If gravitational compensation is consider, it uses that information to compute the latest values
//--------------------------------------------------------------------------------//
void HRP2STEP1Arm::update_currforcedata()
{
  get_raw_forces();   // Read raw forces
  // Calculate the hand's gravitational force and moment
  calc_gc_forces();        // Arm's force and moment
  calc_gc_forces_at_hand();    // Hand's force and moment
}

