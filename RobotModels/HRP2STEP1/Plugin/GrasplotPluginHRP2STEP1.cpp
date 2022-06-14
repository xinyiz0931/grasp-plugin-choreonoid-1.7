// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
   c) Kensuke Harada (AIST)
*/
#include "GrasplotPluginHRP2STEP1.h"
#include <extplugin/graspPlugin/Grasp/exportdef.h>
#include <cnoid/ExecutablePath>


using namespace std;
using namespace cnoid;
using namespace grasp;

bool HRP2STEP1_Arm::IK_arm(const Vector3& p, const Matrix3& R){
        bothArm = false;
        numeric = true;

        VectorXd q_old(nJoints);
        for(int i=0;i<nJoints;i++){
                q_old[i] = armStandardPose[i];
        }
        return solveIK( p,  R, 0.0,  q_old);
}

bool  HRP2STEP1_Arm::IK_arm(const Vector3& p, const Matrix3& R, const VectorXd& q_old){

        bothArm = false;
        numeric = true;

        return solveIK( p,  R, 0.0,  q_old);
}

bool  HRP2STEP1_Arm::IK_arm(const Vector3& p, const Matrix3& R, double phi, const VectorXd& q_old){

        bothArm = true;
        numeric = false;

        return solveIK( p,  R, phi,  q_old);
}

void HRP2STEP1_Arm::adjustArm(const VectorXd& q_old){

    int n = arm_path->numJoints()-1;

    while(arm_path->joint(n)->q() < arm_path->joint(n)->q_lower() ) arm_path->joint(n)->q() += 2*m_pi;
    while(arm_path->joint(n)->q() > arm_path->joint(n)->q_upper() ) arm_path->joint(n)->q() -= 2*m_pi;

    while( arm_path->joint(n)->q() - q_old(n) >  m_pi && arm_path->joint(n)->q()-2*m_pi > arm_path->joint(n)->q_lower())   arm_path->joint(n)->q() -= 2*m_pi;
    while( arm_path->joint(n)->q() - q_old(n) < -m_pi && arm_path->joint(n)->q()+2*m_pi < arm_path->joint(n)->q_upper())   arm_path->joint(n)->q() += 2*m_pi;
}

bool HRP2STEP1_Arm::solveIK(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){
        if(base->name() != "WAIST"){
                if(Arm::IK_arm(p, Rp)) {
                        adjustArm(q_old);
                        return true;
                }
                else
                        return false;
        }
        int n = arm_path->numJoints();
        Matrix33 R = Rp*arm_path->joint(n-1)->Rs(); //R -> attitude()

//        Vector3 wristOffset(-0.059, 0,0); //Force sensor offset
//        if(arm_path->joint(n-1)->name()=="RARM_JOINT5")
//                wristOffset = Vector3(-0.009,0,0); //Maybe a bug in IKfast

//        Vector3 bp = base->attitude().transpose()*(p - R*wristOffset - base->p());
//        Matrix3 bR = base->attitude().transpose()*R;

        if(arm_path->joint(n-1)->name()=="RARM_JOINT5"){
            if(ik_r == NULL){
#ifdef WIN32
                ik_handle = LoadLibrary(_T("ikfast61.HRP2STEP1_RARM.x86.dll"));
                if(!ik_handle) cout << "loading error (adviced to check library name described in lines 77 and 86)" << endl;
                ik_r = (bool (*)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&))GetProcAddress(ik_handle, "ComputeIk");
#else
                string library_name = cnoid::executableTopDirectory() + "/" + string("extplugin/graspPlugin/RobotModels/HRP2STEP1/Plugin/ikfast61.HRP2STEP1_RARM.") + SYSTEM_PROCESSOR  + string(".so");
                ik_handle = dlopen(library_name.c_str(), RTLD_LAZY);
                if(!ik_handle) cout << "loading error (adviced to check library name described in lines 77 and 86)" << endl;
                ik_r = (bool (*)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&))dlsym(ik_handle, "ComputeIk");
#endif
            }
            ik_ = ik_r;
//                arm_path->joint(0)->q() = atan2(bp(1), bp(0)) + asin(0.145/sqrt(bp(0)*bp(0)+bp(1)*bp(1)));
        }
        else if(arm_path->joint(n-1)->name()=="LARM_JOINT5"){
            if(ik_l == NULL){
#ifdef WIN32
                ik_handle = LoadLibrary(_T("ikfast61.HRP2STEP1_LARM.x86.dll"));
                if(!ik_handle) cout << "loading error (adviced to check library name described in lines 77 and 86)" << endl;
                ik_l = (bool (*)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&))GetProcAddress(ik_handle, "ComputeIk");
#else
                string library_name = cnoid::executableTopDirectory() + "/" + string("extplugin/graspPlugin/RobotModels/HRP2STEP1/Plugin/ikfast61.HRP2STEP1_LARM.") + SYSTEM_PROCESSOR  + string(".so");
                ik_handle = dlopen(library_name.c_str(), RTLD_LAZY);
                if(!ik_handle) cout << "loading error (adviced to check library name described in lines 77 and 86)" << endl;
                ik_l = (bool (*)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&))dlsym(ik_handle, "ComputeIk");
#endif
            }
            ik_ = ik_l;
//                arm_path->joint(0)->q() = atan2(bp(1), bp(0)) - asin(0.145/sqrt(bp(0)*bp(0)+bp(1)*bp(1)));
        }

        //if(fabs(phi) > 0.0001)
        if(bothArm)
                arm_path->joint(0)->q() = phi;

        // IKのBase = CHESTの位置・姿勢
        Matrix3 Rz = rotFromRpy(0,0,arm_path->joint(0)->q());
        Vector3 pz = arm_path->joint(0)->p();
        // CHEST基準に変換
        Vector3 p0 = Rz.transpose()*(p - pz);
        Matrix3 R0 = Rz.transpose()*R;

        IkReal p_[3],R_[9];
        for(int i=0; i<3; i++){
                p_[i] = p0(i);
                for(int j=0; j<3; j++) {
                        R_[3*i+j] = R0(i,j);
                }
        }

        bool solved = true;

        ikfast::IkSolutionList<IkReal> vsolutions;
        if(! ik_(p_, R_, NULL, vsolutions) ) solved = false;

        if(solved){
                vector<IkReal> sol(6);      // 関節数 6で固定 (GetNumJoints())
                bool ret = false;
                for(std::size_t i = 0; i < vsolutions.GetNumSolutions(); ++i){
                        vector<IkReal> vsolfree(vsolutions.GetSolution(i).GetFree().size());
                        const ikfast::IkSolutionBase<IkReal>& solution = vsolutions.GetSolution(i);
                        solution.GetSolution(&sol[0], vsolfree.size()>0?&vsolfree[0]:NULL);
                        for( std::size_t j = 0; j < sol.size(); ++j){
                                arm_path->joint(j+1)->q() = sol[j];
                        }
                        // 条件を満たす関節角で最初に見つけたものが設定される
                        if(Arm::checkArmLimit()){
                                ret = true;
                                break;
                        }
                }

                if(!ret) solved = false;
        }

        if(!solved && numeric) solved = Arm::IK_arm(p, Rp);
        if(!solved && bothArm) {
            arm_path->calcForwardKinematics();
            Arm tmp_arm = Arm(body, arm_path->joint(0), palm);
            tmp_arm.multithread_mode = true;
            solved = tmp_arm.IK_arm(p, Rp);
        }

        //if(solved) cout << "IKfast not solvable. Used numerical solution instead." << endl;

        if(!solved) return false;

        adjustArm(q_old);

        if(!Arm::checkArmLimit()) return false;

        arm_path->calcForwardKinematics();

        return true;
}

bool HRP2STEP1_Arm::getPalmPos(const Vector3& Pco1, const Vector3& Pco2, const Matrix3& Rp, const Vector3& pPcr1, const Matrix3& pRcr1, Vector3& Pp, cnoid::VectorXd& theta, double offset)
{
        theta.resize(4);

        Vector3 P1 = Pco1 + offset*unit(Pco2-Pco1);
        Vector3 P2 = Pco2 + offset*unit(Pco1-Pco2);
        if(norm2(Pco1-Pco2) < offset*2.0){
            P1 = 0.5*(Pco1+Pco2);
            P2 = 0.5*(Pco1+Pco2);
        }

        Vector3 pNcr1 = col(pRcr1, 0);
        Vector3 pTcr1 = col(pRcr1, 1);

        //Vector3 pPcr2( pPcr1 + 0.046*pNcr1 );
        Vector3 pPcr2( pPcr1 + 0.03*pNcr1 );

        double l = 0.0419;
        double S = dot((Rp*pNcr1), (Rp*(pPcr1-pPcr2) - (P1-P2)) ) / (2.0*l);
        double q = asin(S);
        theta(0) = q;
        theta(1) = -q;
        theta(2) = -q;
        theta(3) = q;

        Matrix3 R=v3(pNcr1, pTcr1, cross(pNcr1,pTcr1));
        Vector3 d1(sin(q), 1-cos(q), 0);
        Pp = P1 - Rp*(pPcr1 - l*R*d1);

        return true;
}

bool HRP2STEP1_Arm::getPalmPos(const Vector3& Pco1, const Vector3& Pco2,
                          const Matrix3& Rp, const Vector3& pPcr1,
                          const Matrix3& pRcr1, Vector3& Pp,
                          cnoid::VectorXd& theta,
                          const std::vector<double>& offset)
{
        theta.resize(4);

        Vector3 P1 = Pco1;
        Vector3 P2 = Pco2;

        Vector3 pNcr1 = col(pRcr1, 0);
        Vector3 pTcr1 = col(pRcr1, 1);

        //Vector3 pPcr2( pPcr1 + 0.046*pNcr1 );
        Vector3 pPcr2( pPcr1 + 0.03*pNcr1 );

        double l = 0.0419;
        double S = dot((Rp*pNcr1), (Rp*(pPcr1-pPcr2) - (P1-P2)) ) / (2.0*l);
        double q = asin(S);
        theta(0) = q + ((offset.size() > 0) ? offset[0] : 0);
        theta(1) = -q + ((offset.size() > 1) ? offset[1] : 0);
        theta(2) = -q + ((offset.size() > 2) ? offset[2] : 0);
        theta(3) = q + ((offset.size() > 3) ? offset[3] : 0);

        Matrix3 R=v3(pNcr1, pTcr1, cross(pNcr1,pTcr1));
        Vector3 d1(sin(q), 1-cos(q), 0);
        Pp = P1 - Rp*(pPcr1 - l*R*d1);

        return true;
}




extern "C" EXCADE_API void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm)
{
    return new HRP2STEP1_Arm(body, base, palm);
}
