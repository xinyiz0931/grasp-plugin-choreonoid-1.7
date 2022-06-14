/**
   c) Kensuke Harada (AIST)
*/

#ifndef GRASPLOTPLUGIN_HRP2STEP_H
#define GRASPLOTPLUGIN_HRP2STEP_H

#include <iostream>
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/
#include <extplugin/graspPlugin/Grasp/Arm.h>
#include <extplugin/graspPlugin/Grasp/VectorMath.h>
#include <extplugin/graspPlugin/Grasp/exportdef.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#ifdef WIN32
#include <windows.h>
#include <tchar.h>
#else
#include <dlfcn.h>
#endif
#include "OpenRAVE/ikfast.h"

#include "Config.h"

#define m_pi 3.141592

typedef double IkReal;

namespace grasp{

class EXCADE_API HRP2STEP1_Arm: public Arm
{
    public:

        HRP2STEP1_Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) : Arm(body, base, palm) {
            this->base = base;
            bothArm = true;
            numeric = true;
            ik_handle = NULL;
            ik_ = NULL;
            ik_r = NULL;
            ik_l = NULL;
            this->body = body;
            };
        ~HRP2STEP1_Arm() {
            if(ik_handle != NULL){
#ifdef WIN32
                FreeLibrary(ik_handle);
#else
                dlclose(ik_handle);
#endif
            }
        }
//		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix33 &R);
        bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R);
        bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, const cnoid::VectorXd& q_old);
        bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old= cnoid::VectorXd::Zero(7));
        bool solveIK(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old);
        bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta, double offset=0.0);
        bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta, const std::vector<double>& offset);
        void adjustArm(const cnoid::VectorXd& q_old);

        /*
        double IndexFunc(double a, double b){return Arm::IndexFunc(a,b);}
        cnoid::VectorXd calcGradient(double a, double b){return Arm::calcGradient(a,b);}
        bool checkArmLimit(){return Arm::checkArmLimit();}
        double Manipulability(){return Arm::Manipulability();}
        double avoidAngleLimit(){return Arm::avoidAngleLimit();}
        double avoidAngleLimit2(){return Arm::avoidAngleLimit2();}

        bool closeArm(int lk, int iter, cnoid::Vector3 &oPos, cnoid::Vector3 &objN){return Arm::closeArm(lk,iter,oPos,objN);}
        */


    private:
        cnoid::Link *base;
        cnoid::BodyPtr body;
        bool bothArm, numeric;
#ifdef WIN32
        HMODULE ik_handle;
#else
        void *ik_handle;
#endif
//        bool (*ik_)(const IkReal*, const IkReal*, const IkReal*,std::vector<ikfast::IkSolution<IkReal> >&);
        bool (*ik_)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&);
        bool (*ik_r)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&);
        bool (*ik_l)(const IkReal*, const IkReal*, const IkReal*, ikfast::IkSolutionListBase<IkReal>&);
};
}
#endif
