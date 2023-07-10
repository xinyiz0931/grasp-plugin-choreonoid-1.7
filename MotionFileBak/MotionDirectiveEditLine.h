#ifndef MOTIONDIRECTIVE_EDIT_LINE_H
#define MOTIONDIRECTIVE_EDIT_LINE_H

#include <QWidget>
#include <QFrame>
#include <QLayout>
#include <QComboBox>
#include <QSpinBox>

#include <cnoid/ComboBox>
#include <boost/bind.hpp>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#else
#include <cnoid/Signal>
#endif

#include "RobotInMotionEdit.h"

namespace motionedit
{
	class MotionDirectiveInfo;

	class MotionDirectiveValueEditLine;

	/**
	 * @brief ����w�����\���N���X�ł��B
	 */
	class MotionDirectiveEditLine : public QFrame
	{

	public:
		
		/**
		 * @brief �R���X�g���N�^�ł��B
		 */
		MotionDirectiveEditLine(QWidget*, int);

		MotionDirectiveEditLine(QWidget*, int, const std::map<std::string, std::vector<double> >& );

		/**
		 * @brief ���g�̍s�ԍ��𓾂郁���o�֐��ł��B
		 */
		int GetId();

		/**
		 * @brief ����J�n���Ԃ𓾂郁���o�֐��ł��B
		 */
		double GetStartTime();

		/**
		 * @brief ����I�����Ԃ𓾂郁���o�֐��ł��B
		 */
		double GetEndTime();

		std::string GetMotionDirectiveTypeName();

                std::string GetCoordinateTypeName();   

	        std::vector<double> GetMotionDirectiveVaueList();	

		int GetMotionDirectiveValueCount();

		/**
		 * @brief ����J�n���Ԃ�ݒ肷�郁���o�֐��ł��B
		 */
		void SetStartTime(double);

		/**
		 * @brief ����w���̎�ނ̖��O��ݒ肷�郁���o�֐��ł��B
		 */
		void SetMotionDirectiveTypeName(const QString&);

		/**
		 * @brief ���W�n�̎�ނ̖��O��ݒ肷�郁���o�֐��ł��B
		 */
                void SetCoordinateTypeName(const QString&);

		/**
		 * @brief ����I�����Ԃ�ݒ肷�郁���o�֐��ł��B

		 */
		void SetEndTime(double);

		/**
		 * @brief �^����ꂽ����w���������Ƃ�
		 * ���g���X�V���郁���o�֐��ł��B
		 */
		void Update(const MotionDirectiveInfo&);

		/**
		 * @brief �^����ꂽ����w���������Ƃ�
		 * ����w�������쐬���郁���o�֐��ł��B
		 */
		MotionDirectiveInfo MakeMotionDirectiveInfo();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		void InitializeByRobot(boost::intrusive_ptr<IRobotInMotionEdit>);
#else
		void InitializeByRobot(cnoid::ref_ptr<IRobotInMotionEdit>);
#endif

                std::vector<double> GetValueList();

		void Update(
			double startTime,
			double endTime,
			std::string motionDirectiveTypeName,
			std::string coordinateTypeName,
			const std::vector<double>& motionDirectiveValueList
			);
	//signals:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		 * @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		 */
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > MotionDirectiveTypeChanged();

		/**
		 * @brief ����w���̍��W�n���ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		 */
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > CoordinateTypeChanged();
#else
		/**
		 * @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		 */
		cnoid::SignalProxy<void(int, const QString&)> MotionDirectiveTypeChanged();

		/**
		 * @brief ����w���̍��W�n���ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		 */
		cnoid::SignalProxy<void(int, const QString&)> CoordinateTypeChanged();
#endif
		
		void Initialize(QWidget*);

	//public slots:

		/**
		 * @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
		 * �󂯎��X���b�g�ł��B
		 */
		void on_motionDirectiveTypeChanged(const QString&);

		/**
		 * @brief ���W�n�̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
		 * �󂯎��X���b�g�ł��B
		 */
		void on_coordinateTypeChanged(const QString&);
	    
	private:

		/**
		 * @brief ���g�̍s�ԍ��ł��B
		 * �s�ԍ��͎��g������w���ҏW���C���S�̂̏ォ�牽�ԖڂɈʒu���邩��\���܂��B
		 */
		int m_Id;

		/**
		 * @brief ����w���̊J�n���Ԃ�ݒ肷�邽�߂̃{�b�N�X�ł��B
		 */
		QDoubleSpinBox* m_startTimeEdit;

		/**
		 * @brief ����w���̏I�����Ԃ�ݒ肷�邽�߂̃{�b�N�X�ł��B
	  	 */
		QDoubleSpinBox* m_endTimeEdit;
		
		/**
		 * @brief ����w���̎�ނ�ݒ肷�邽�߂̃{�b�N�X�ł��B
		 */
		cnoid::ComboBox* m_motionDirectiveNameBox;
		
		/**
		 * @brief ����w���̍��W�n��ݒ肷�邽�߂̃{�b�N�X�ł��B
		 */
		cnoid::ComboBox* m_coordinateSystemNameBox;
		
                QString m_currentMotionDirectiveTypeName;
             
		/**
	  	 * @brief ����w���̐��l��ł��B
		 */
		MotionDirectiveValueEditLine* m_motionDirectiveValueEditLine;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::signal<void(int, const QString&)> m_motionDirectiveTypeChanged;
		boost::signal<void(int, const QString&)> m_coordinateTypeChanged;
#else
		cnoid::Signal<void(int, const QString&)> m_motionDirectiveTypeChanged;
		cnoid::Signal<void(int, const QString&)> m_coordinateTypeChanged;
#endif

		std::map<std::string, std::vector<double> > m_motionAndValuesMap;

		void on_motionDirectiveType_Changed(int);

	};
}
#endif
