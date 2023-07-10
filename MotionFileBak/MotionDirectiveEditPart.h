#ifndef MOTIONDIRECTIVE_EDIT_PART_H
#define MOTIONDIRECTIVE_EDIT_PART_H

#include <QWidget>
#include <QFrame>
#include <QLayout>
#include <QComboBox>
#include <QSpinBox>

#include <string>
#include <vector>
#include <stack>

#include "MotionDirectiveInfo.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

namespace motionedit
{
	class MotionDirectiveEditLine;
	
	class IRobotInMotionEdit;

	 /**
	 * @brief ����w����ҏW����\���N���X�ł��B 
         * �����̓���w���ҏW��������܂��B
	 */
	class MotionDirectiveEditPart :public QVBoxLayout
	{

	public:
		 /**
		 * @brief �R���X�g���N�^�ł��B
		 */
		MotionDirectiveEditPart(QWidget*);

 		/**
	        * @brief ����w���ҏW���C���̐��𓾂郁���o�֐��ł��B
		*/
		int GetLineCount();
		 
		/**	 	
	        * @biref ���g���܂ޓ���w��ҏW��̂����A
	        * �w�肵���C���f�b�N�X�̗�̓���J�n���Ԃ𓾂郁���o�֐��ł��B
	        */
                int GetStartTime(int);
     
               /**
               * @biref ���g���܂ޓ���w��ҏW��̂����A
	       * �w�肵���C���f�b�N�X�̗�̓���I�����Ԃ𓾂郁���o�֐��ł��B
	       */
               int GetEndTime(int);

	       /**
               * @biref ���g���܂ޓ���w��ҏW��̂����A
	       * �w�肵���C���f�b�N�X�̗�̓���w��̎�ނ̖��O�𓾂郁���o�֐��ł��B
	       */
               std::string GetMotionDirectiveTypeName(int);

	       /**
               * @biref ���g���܂ޓ���w��ҏW��̂����A
	       * �w�肵���C���f�b�N�X�̗�̍��W�n�̎�ނ̖��O�𓾂郁���o�֐��ł��B
	       */
               std::string GetCoordinateTypeName(int);   

	       /**
               * @biref ���g���܂ޓ���w��ҏW��̂����A
	       * �w�肵���C���f�b�N�X�̗�̍��W�n�̎�ނ̖��O�𓾂郁���o�֐��ł��B
	       */
	       std::vector<double> GetMotionDirectiveValueList(int);
 
		/**
               * @biref ���g���܂ޓ���w��ҏW��̂����A
	       * �w�肵���C���f�b�N�X�̗�̐��l��̐��𓾂郁���o�֐��ł��B
	       */
	       int GetMotionDirectiveValueCount(int);

 		 /**
	        * @brief ����w���s���s���� 1 �s�ǉ����郁���o�֐��ł��B
		*/
		void AddLine();

		/**
	        * @brief �s���̓���w���s�� 1 �s�폜���郁���o�֐��ł��B
	        */
		void DeleteLine();
		
		/**
		* @brief �^����ꂽ���{�b�g�̏������ɂ��ē���w���ҏW�������������郁���o�֐��ł��B
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		void  InitializeByRobot(boost::intrusive_ptr<IRobotInMotionEdit>);
#else
		void  InitializeByRobot(cnoid::ref_ptr<IRobotInMotionEdit>);
#endif
		/**
		* @brief �^����ꂽ����w���������Ƃ�
		* �S�Ă̓���w���ҏW���C�����X�V���郁���o�֐��ł��B
		*/
		void Update(const std::vector<MotionDirectiveInfo>&);

 		/**
		* @biref ��
		*/
		void Update(int, double, double, std::string, std::string, const std::vector<double>&);	

		/**
		* @brief �S�Ă̓���w���ҏW���C�����폜���郁���o�֐��ł��B
		*/
		void Clear();

		 /**
		* @biref ����w���ҏW��̓��e�����Ƃɓ���w�����̃��X�g���쐬���郁���o�֐��ł��B
		*/
		std::vector<MotionDirectiveInfo> MakeMotionDirectiveInfoList();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief �����ꂩ�̍s�œ���w���̎�ނ��ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		*/
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > MotionDirectiveTypeChanged();

		/**
		* @brief �����ꂩ�̍s�œ���w���̍��W�n���ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B

		*/
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > CoordinateTypeChanged();
#else
		/**
		* @brief �����ꂩ�̍s�œ���w���̎�ނ��ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		*/
		cnoid::SignalProxy<void(int, const QString&)> MotionDirectiveTypeChanged();

		/**
		* @brief �����ꂩ�̍s�œ���w���̍��W�n���ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		*/
		cnoid::SignalProxy<void(int, const QString&)> CoordinateTypeChanged();
#endif
	      
	private:

	        /**
		* @brief ����w���ҏW��̃��X�g�ł��B
		*/
		std::vector<MotionDirectiveEditLine*> m_motionDirectiveEditLineList;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief ���g���܂ޓ���w���ҏW��̂����ꂩ�œ���w�����e���ύX���ꂽ�ꍇ�ɔ�������V�O�i���ł��B
		*/
		boost::signal<void(int, const QString&)> m_motionDirectiveTypeChanged;

		/**
		* @brief ���g���܂ޓ���w���ҏW��̂����ꂩ�ō��W�n�̓��e���ύX���ꂽ�ꍇ�ɔ�������V�O�i���ł��B
		*/
		boost::signal<void(int, const QString&)> m_coordinateTypeChanged;
		
		/**
		* @brief ����w���̓��e���ύX���ꂽ�ꍇ�̊e��̃V�O�i���ƃX���b�g�̌������i�[���郊�X�g�ł��B
		*/
		std::stack<boost::signals::connection> m_connectionStackOfMotionDirectiveTypeChangedSignal;
		
		/**
		* @brief ���W�n�̓��e���ύX���ꂽ�ꍇ�̊e��̃V�O�i���ƃX���b�g�̌������i�[���郊�X�g�ł��B
		*/
		std::stack<boost::signals::connection> m_connectionStackOfCoordinateTypeChangedSignal;
#else
		/**
		* @brief ���g���܂ޓ���w���ҏW��̂����ꂩ�œ���w�����e���ύX���ꂽ�ꍇ�ɔ�������V�O�i���ł��B
		*/
		cnoid::Signal<void(int, const QString&)> m_motionDirectiveTypeChanged;

		/**
		* @brief ���g���܂ޓ���w���ҏW��̂����ꂩ�ō��W�n�̓��e���ύX���ꂽ�ꍇ�ɔ�������V�O�i���ł��B
		*/
		cnoid::Signal<void(int, const QString&)> m_coordinateTypeChanged;
		
		/**
		* @brief ����w���̓��e���ύX���ꂽ�ꍇ�̊e��̃V�O�i���ƃX���b�g�̌������i�[���郊�X�g�ł��B
		*/
		std::stack<cnoid::Connection> m_connectionStackOfMotionDirectiveTypeChangedSignal;
		
		/**
		* @brief ���W�n�̓��e���ύX���ꂽ�ꍇ�̊e��̃V�O�i���ƃX���b�g�̌������i�[���郊�X�g�ł��B
		*/
		std::stack<cnoid::Connection> m_connectionStackOfCoordinateTypeChangedSignal;
#endif
	        /**
		* @brief ���W�n�̖��O�ƑΉ����鐔�l��̃}�b�v�ł��B
		*/
		std::map<std::string, std::vector<double> > m_motionAndValuesMap;

		/**
		* @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
		* �󂯎��X���b�g�ł��B
		*/
		void on_motionDirectiveTypeChanged(int, const QString&);

		/**
		* @brief ���W�n�̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
		* �󂯎��X���b�g�ł��B
		*/
		void on_coordinateTypeChanged(int, const QString&);
	};

}
#endif
