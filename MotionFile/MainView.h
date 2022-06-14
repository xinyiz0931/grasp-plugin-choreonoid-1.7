#ifndef MOTION_EDIT_MAIN_VIEW_H
#define MOTION_EDIT_MAIN_VIEW_H

#include <string>
#include <vector>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/intrusive_ptr.hpp>
#else
#include <cnoid/Referenced>
#endif
#include <boost/bind.hpp>

namespace cnoid
{
	class BodyItem;
}

class QWidget;
class QFrame;
class QScrollArea;

namespace motionedit
{
	class FileIOAndExecutingPart;

	class MotionDirectiveEditPart;

	class LineEditPart;

	class MotionDirectiveInfo;

	class IMotionDirectiveTypeConverter;

	class IUsingHandTypeConverter;

	class ICoordinateTypeConverter;

	class IRobotInMotionEdit;

	class IMotionFileReader;

	class IMotionDirectiveWordAnalyzer;

	class IMotionDirectiveInfoWriter;

         /**
 	 * @brief ����w���ҏW�^�u�̃��C���ƂȂ镔���̃N���X�ł��B 
	 * ������ �t�@�C���ǂݍ��݁E�������݁E������s���A
	 * ����w���ҏW���A
         * ����w���s�ҏW���������܂��B
	 */
	class MainView : public QFrame
	{

	public:

                /**
                * @brief �R���X�g���N�^�ł��B
		*/
		MainView(QWidget*);       

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
	        * @biref �t�@�C���ǂݍ��ݗp�I�u�W�F�N�g��ݒ肷�郁���o�֐��ł��B
		*/
		void SetReader(boost::intrusive_ptr<IMotionFileReader>);

                /**
		* @biref �t�@�C���������ݗp�I�u�W�F�N�g��ݒ肷�郁���o�֐��ł��B
		*/
		void SetWriter(boost::intrusive_ptr<IMotionDirectiveInfoWriter>);

		/**
		* @biref ����ҏC�Ώۂ̃��{�b�g��ݒ肷�郁���o�֐��ł��B
		*/
		void SetRobot(boost::intrusive_ptr<IRobotInMotionEdit>);
#else
		/**
	        * @biref �t�@�C���ǂݍ��ݗp�I�u�W�F�N�g��ݒ肷�郁���o�֐��ł��B
		*/
		void SetReader(cnoid::ref_ptr<IMotionFileReader>);

                /**
		* @biref �t�@�C���������ݗp�I�u�W�F�N�g��ݒ肷�郁���o�֐��ł��B
		*/
		void SetWriter(cnoid::ref_ptr<IMotionDirectiveInfoWriter>);

		/**
		* @biref ����ҏC�Ώۂ̃��{�b�g��ݒ肷�郁���o�֐��ł��B
		*/
		void SetRobot(cnoid::ref_ptr<IRobotInMotionEdit>);
#endif

		/**
		* @brief ����w����̃��C����ǉ����郁���o�֐��ł��B
		*/
		void InitializeByRobot();

	        /**
		* @brief ����w����̃��C����ǉ����郁���o�֐��ł��B
		*/
		void AddLine(int);

                /**
		* @biref����w���ҏW������Ƃɓ���w�����̃��X�g���쐬���郁���o�֐��ł��B
		*/
		std::vector<MotionDirectiveInfo> MakeMotionDirectiveInfoList();
		
	private:

		/**
		* @brief ����w���ҏC�^�u�̂���
   		* �t�@�C���̓ǂݍ��ݏ����o���A����̎��s���s���{�^�����������ł��B
		*/
		FileIOAndExecutingPart* m_FileIOAndExecutingPart;

		/**
		* @brief ����w���ҏC�^�u�̂���
		* ����w���̕ҏC���s�������ł��B
		*/
		MotionDirectiveEditPart* m_motionDirectiveEditPart;

		/**
		* @brief ����w���ҏW��������Ɋi�[����X�N���[���G���A�ł��B
		*/
		QScrollArea* m_motionDirectiveEditPartScrollArea;

		/**
		* @brief ����w���ҏC�^�u�̂���
		*����w���s�̒ǉ��Ȃǂ̑�����s�������ł��B
		*/
		LineEditPart* m_LineEditPart;
		
		/**
		* @ brief �ǂݍ��񂾃t�@�C�����ł��B
		*/
		std::string m_loadFileName;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		 * @biref ����w����̏����ꂽ�t�@�C����ǂݍ��ނ��߂Ɏg�p���郁���o�ł��B
		 */
		boost::intrusive_ptr<IMotionFileReader> m_motionFileReader;

		/**
		* @biref ����w���̎�ނƂ����\��������𑊌݂ɕϊ����邽�߂̃����o�ł��B
		*/
		boost::intrusive_ptr<IMotionDirectiveTypeConverter> m_motionDirectiveTypeConverter;

		/**
		* @biref �g�p����r�̎�ނƂ����\��������𑊌݂ɕϊ����邽�߂̃����o�ł��B

		*/
		boost::intrusive_ptr<IUsingHandTypeConverter> m_usingHandTypeConverter;

		/**
		* @biref ���W�n�̎�ނ���ނƂ����\��������𑊌݂ɕϊ����邽�߂̃����o�ł��B

		*/
		boost::intrusive_ptr<ICoordinateTypeConverter> m_coordinateTypeConverter;

		/**

		* @biref ���������͂��Ęr�̎�ނƓ���w���̎�ނ̕�����𓾂邽�߂̃����o�ł��B  
		*/
		boost::intrusive_ptr<IMotionDirectiveWordAnalyzer> m_motionDirectiveWordAnalyzer;

		/**
 		* @biref ����w���̕ҏW���e�̃t�@�C���������݂��s�������o�ł��B
		*/
		boost::intrusive_ptr<IMotionDirectiveInfoWriter> m_motionDirectiveInfoWriter;
#else
		/**
		 * @biref ����w����̏����ꂽ�t�@�C����ǂݍ��ނ��߂Ɏg�p���郁���o�ł��B
		 */
		cnoid::ref_ptr<IMotionFileReader> m_motionFileReader;

		/**
		* @biref ����w���̎�ނƂ����\��������𑊌݂ɕϊ����邽�߂̃����o�ł��B
		*/
		cnoid::ref_ptr<IMotionDirectiveTypeConverter> m_motionDirectiveTypeConverter;

		/**
		* @biref �g�p����r�̎�ނƂ����\��������𑊌݂ɕϊ����邽�߂̃����o�ł��B

		*/
		cnoid::ref_ptr<IUsingHandTypeConverter> m_usingHandTypeConverter;

		/**
		* @biref ���W�n�̎�ނ���ނƂ����\��������𑊌݂ɕϊ����邽�߂̃����o�ł��B

		*/
		cnoid::ref_ptr<ICoordinateTypeConverter> m_coordinateTypeConverter;

		/**

		* @biref ���������͂��Ęr�̎�ނƓ���w���̎�ނ̕�����𓾂邽�߂̃����o�ł��B  
		*/
		cnoid::ref_ptr<IMotionDirectiveWordAnalyzer> m_motionDirectiveWordAnalyzer;

		/**
 		* @biref ����w���̕ҏW���e�̃t�@�C���������݂��s�������o�ł��B
		*/
		cnoid::ref_ptr<IMotionDirectiveInfoWriter> m_motionDirectiveInfoWriter;
#endif

		/**
	  	* @ brief �^����ꂽ����w����񃊃X�g�����Ƃ�
		* ���g�̑S�Ă̓���w������X�V���郁���o�ϐ��ł��B
		*/
		void Update(const std::vector<MotionDirectiveInfo>&);
		
		/**
		* @ ����w���ҏW�̑ΏۂƂ��郍�{�b�g�����o�ϐ��ł��B
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<IRobotInMotionEdit> m_robot;	
#else
		cnoid::ref_ptr<IRobotInMotionEdit> m_robot;	
#endif

	        /*******************************************
		 * �X���b�g�錾��
                 ******************************************/		
		/**
		* @brief ���[�h�{�^�����N���b�N���ꂽ�ꍇ���s����X���b�g�ł��B
		*/
		void on_loadButton_Clicked();

		/**
	 	* @brief �Z�[�u�{�^�����N���b�N���ꂽ�ꍇ�Ɏ��s����X���b�g�ł��B
		*/
		void on_saveButton_Clicked();

		/**
		* @brief �v���C�{�^�����N���b�N���ꂽ�ꍇ�Ɏ��s����X���b�g�ł��B
		*/
		void on_playButton_Clicked();

		/**
		* @brief ���C���ǉ��{�^�����N���b�N���ꂽ�ꍇ�Ɏ��s����X���b�g�ł��B
		*/
		void on_appendButton_Clicked();

		/**
		* @brief �����ꂩ�̍s�œ���w���̎�ނ��ύX���ꂽ�ꍇ�Ɏ��s����X���b�g�ł��B
		*/
		void on_motionDirectiveType_Changed(int, const QString&);

		/**
	 	* @brief �����ꂩ�̍s�ō��W�n�̎�ނ��ύX���ꂽ�ꍇ�Ɏ��s����X���b�g�ł��B
		*/
		void on_coordinateType_Changed(int, const QString&);
		
		/**
	 	* @brief ���{�b�g�A�C�e���o�[�őI������Ă��郍�{�b�g���ύX���ꂽ�ꍇ�Ɏ��s����X���b�g�ł��B
		*/
		void on_bodyItem_Changed(cnoid::BodyItem*);
	};
}

#endif
