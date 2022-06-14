#ifndef MOTIONFILEREADER_H
#define MOTIONFILEREADER_H

#include <istream>
#include <vector>
#include "MotionDirectiveInfo.h"
#include "MotionFileData.h"
#include "TextFieldParser.h"
#include "ToMotionDirectiveInfoConverter.h"

#include <cnoid/Referenced>

namespace motionedit
{
	/**
	 * @brief �t�@�C�����瓮��w���̏���ǂݍ��ރC���^�[�t�F�[�X�ł��B
	 */
	class IMotionFileReader : public cnoid::Referenced
	{
	public:
		IMotionFileReader() { }

	        // �����ŗ^����ꂽ�t�@�C����ǂݍ��ݓ���w���f�[�^�𓾂鏃�����z�֐��ł��B
		virtual MotionFileData Read(std::string) = 0;
	};

	/**
	* @brief �t�@�C�����瓮��w���̏���ǂݍ��ރC���^�[�t�F�[�X�ł��B
	*/
	class MotionFileReader : public IMotionFileReader
	{
	public:
		/**
		* @brief �R���X�g���N�^�ł��B
		*/
		MotionFileReader() { } 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	        /**
		* @brief �t�@�C���ǂݍ��݂̍ۂɎg�p���� �p�[�T�[ �� �R���o�[�^ �������Ɏ��܂��B 
		*/
		MotionFileReader(boost::intrusive_ptr<ITextFieldParser>, boost::intrusive_ptr<IToMotionDirectiveInfoConverter>);
#else
	        /**
		* @brief �t�@�C���ǂݍ��݂̍ۂɎg�p���� �p�[�T�[ �� �R���o�[�^ �������Ɏ��܂��B 
		*/
		MotionFileReader(cnoid::ref_ptr<ITextFieldParser>, cnoid::ref_ptr<IToMotionDirectiveInfoConverter>);
#endif
		/**
		* @brief �����̖��O�����t�@�C����ǂݍ��ݓ���w�����𓾂�C���^�[�t�F�[�X�̎����ł��B
		*/
		MotionFileData Read(std::string);


	private:
		/**
		* @brief �t�@�C���ǂݍ��݃N���X�̎����N���X�ł��B
		*/
		class Impl : public cnoid::Referenced
		{
		public:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			// �p�[�T �C���^�[�t�F�[�X�ł��B
			boost::intrusive_ptr<ITextFieldParser> textFieldParser;
			// �p�[�X���ꂽ�����񂩂瓮��w���������o���C���^�[�t�F�[�X�ł��B
			boost::intrusive_ptr<IToMotionDirectiveInfoConverter> toMotionDirectiveConverter;
#else
			// �p�[�T �C���^�[�t�F�[�X�ł��B
			cnoid::ref_ptr<ITextFieldParser> textFieldParser;
			// �p�[�X���ꂽ�����񂩂瓮��w���������o���C���^�[�t�F�[�X�ł��B
			cnoid::ref_ptr<IToMotionDirectiveInfoConverter> toMotionDirectiveConverter;
#endif

			// �t�@�C�����Ɋ܂܂�鑼�̓���w���t�@�C���̖��O�𓾂郁���o�֐��ł��B 
			std::string GetMotionFileName(std::vector<std::vector<std::string> >&);
		    
			// �����ŕ\�L���ꂽ����w�����̃��X�g�𓾂郁���o�֐��ł��B
			std::vector<MotionDirectiveInfo> 
				GetMotionDirectiveInfoList(std::vector<std::vector<std::string> >&);

			// �t�@�C���ɂ͑��̓���w���t�@�C���̖��O���L�ڂ���Ă���ꍇ������܂��B���̈ʒu�̃C���f�b�N�X�ł��B
			static const int motionFileNamePos = 0;
		};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<Impl> mImpl;
#else
		cnoid::ref_ptr<Impl> mImpl;
#endif
	};
}

#endif
