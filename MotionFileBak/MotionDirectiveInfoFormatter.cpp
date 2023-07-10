#include <iostream>
#include <exception>

#include "MotionDirectiveInfoFormatter.h"
#include "MotionDirectiveInfo.h"

#include "MotionDirectiveTypeConverter.h"
#include "UsingHandTypeConverter.h"
#include "CoordinateTypeConverter.h"
#include "MotionUtil.h"

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace motionedit;

/**
*@brief �^����ꂽ���W�n�̎�ށA�g�p����r�̎�ދy�э��W�n�̎�ނ���A����w������\����������쐬���郁���o�֐��̎����ł��B
*/
string MotionDirectiveInfoFormatter::Impl::GetMotionDirectiveAndCoordinateString(
	MotionDirectiveType motionDirectiveType,
	UsingHandType usingHandType,
	CoordinateType coordinateType
	)
{
	// �g�p����r�̎�ނɑΉ����镶������쐬���܂��B
	string usingHandTypeString;
	{
		// �g�p����r�̎�ނ���܂�Ȃ��ꍇ�͋�̕������Ԃ��܂��B
		if (usingHandType == UsingHandType_None)
		{
			usingHandTypeString = "";
		}
		// �g�p����r�̎�ނ��R���o�[�^��p���ĕ�����ɕϊ����܂��B
		else
		{
			usingHandTypeString = usingHandTypeConverter->ConvertToString(usingHandType);
		}
	}

	// ����w���̎�ނ��R���o�[�^��p���ĕ�����ɕϊ����܂��B
	string motionDirectiveTypeString = motionDirectiveTypeConverter->ConvertToString(motionDirectiveType);

	// ���W�n�̎�ނ��R���o�[�^��p���ĕ�����ɕϊ����܂��B
	string coordinateTypeString = coordinateTypeConverter->ConvertToString(coordinateType);

	// ����ꂽ 3 �̕���������킹�� 1  �̒P����쐬���܂��B
        // ���W�n��\��������̑O�ɂ� "_"  ��ǉ����܂��B
	string resultString = usingHandTypeString+ motionDirectiveTypeString;
	if(!coordinateTypeString.empty())
	{
		resultString += "_";
	}

	return resultString + coordinateTypeString;
}

/**
*@brief ����w�����𕶎���ɕϊ�����N���X�̃R���X�g���N�^�̎����ł��B
* �����Ŏg�p����A����w���̎�ނ̕ϊ��R���o�[�^�A�r�̎�ނ̕�����ւ̕ϊ��R���o�[�^�A�y�э��W�n�̕�����ւ̕ϊ��R���o�[�^�������Ɏ��܂��B
*/
MotionDirectiveInfoFormatter::MotionDirectiveInfoFormatter(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter,
	boost::intrusive_ptr<IUsingHandTypeConverter> usingHandTypeConverter,
	boost::intrusive_ptr<ICoordinateTypeConverter> coordinateTypeConverter
#else
	cnoid::ref_ptr<IMotionDirectiveTypeConverter> motionDirectiveTypeConverter,
	cnoid::ref_ptr<IUsingHandTypeConverter> usingHandTypeConverter,
	cnoid::ref_ptr<ICoordinateTypeConverter> coordinateTypeConverter
#endif
	) :mImpl(new Impl())
{

	mImpl->motionDirectiveTypeConverter = motionDirectiveTypeConverter;
	mImpl->usingHandTypeConverter = usingHandTypeConverter;
	mImpl->coordinateTypeConverter = coordinateTypeConverter;
}

/**
*@brief ����w�����^�̃f�[�^�𕶎���ɕϊ������郁���o�֐��̎����ł��B
*/
string MotionDirectiveInfoFormatter::Format(const MotionDirectiveInfo& motionDirectiveInfo)
{
	// �ϊ���̕�������i�[����ϐ����쐬���܂��B
	string formattedMotionDirectiveInfo;
		
	// ����J�n���Ԃ�\��������𓾂܂��B�J�n���ԏ��𕶎���ɕϊ�����ۂɗ�O�����������ꍇ�� 
        // "0" �Ƃ��܂��B 
	string startTimeString;
	try{

		startTimeString = lexical_cast<string>(motionDirectiveInfo.GetStartTime());
	        //startTimeString = util::GetFormattedDoubleString(motionDirectiveInfo.GetStartTime());	

	}catch(std::exception e)
	{
		startTimeString = "0.0000";
	}

	formattedMotionDirectiveInfo += startTimeString;
	formattedMotionDirectiveInfo += m_delimiter;

	// �J�n���ԂƏI�����Ԃ��قȂ�ꍇ�̂ݏI�����Ԃ𕶎���ɕϊ����܂��B
	if (motionDirectiveInfo.GetStartTime() != motionDirectiveInfo.GetEndTime())
	{
		// ����I�����Ԃ�\��������𓾂܂��B�I�����ԏ��𕶎���ɕϊ�����ۂɗ�O�����������ꍇ�� 
       		// "0" �Ƃ��܂��B  
		string endTimeString;
		try{
			endTimeString = lexical_cast<string>(motionDirectiveInfo.GetEndTime());
			//endTimeString = util::GetFormattedDoubleString(motionDirectiveInfo.GetEndTime());
		
		}catch(std::exception e)
		{
			endTimeString = "0.0000";
		}
		formattedMotionDirectiveInfo += endTimeString;
		formattedMotionDirectiveInfo += m_delimiter;
	}

	//�^����ꂽ����w�����^�f�[�^���́A����w���̎�ށA�g�p����r�̎�ދy�э��W�n�̎�ނ���A
	// ����̓��e�ƍ��W�n����킷 1 �̒P����쐬���ǉ����܂��B
	string motionDirectiveName = mImpl->GetMotionDirectiveAndCoordinateString(
		motionDirectiveInfo.GetMotionDirectiveType(),
		motionDirectiveInfo.GetUsingHandType(),
		motionDirectiveInfo.GetCoordinateType()
		);

	formattedMotionDirectiveInfo += motionDirectiveName;

        // ���l��������쐬���܂��B

	for (int i = 0; i < motionDirectiveInfo.GetDirectiveValueCount(); i++)
	{
		formattedMotionDirectiveInfo += m_delimiter;
		formattedMotionDirectiveInfo += util::GetFormattedDoubleString(motionDirectiveInfo.GetDirectiveValue(i));
	}

	return formattedMotionDirectiveInfo;
}

/**
*@brief ����w�����̊e�v�f����؂��؂蕶����ݒ肷�郁���o�֐��̎����ł��B
*/
void MotionDirectiveInfoFormatter::SetDelimiter(const string& delimiter)
{
	m_delimiter = delimiter;

	return;
}
