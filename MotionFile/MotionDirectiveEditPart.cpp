#include "MotionDirectiveEditPart.h"

#include <vector>
#include <QMessageBox>
#include <iostream>

#include "MotionDirectiveEditLine.h"
#include "RobotInMotionEdit.h"
#include "MotionUtil.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace motionedit;

/**
* @brief MotionDirectiveEditPart �̃R���X�g���N�^�̎����ł��B
*/
MotionDirectiveEditPart::MotionDirectiveEditPart(QWidget* parent) :QVBoxLayout(parent)
{}

/**
* @brief ����w���ҏW���C���̐��𓾂郁���o�֐��̎����ł��B
*/
int MotionDirectiveEditPart::GetLineCount()
{

	return m_motionDirectiveEditLineList.size();
}

/**
* @brief �^����ꂽ����w���������Ƃ�
* �S�Ă̓���w���ҏW���C�����X�V���郁���o�֐��̎����ł��B
*/
void MotionDirectiveEditPart::Update(const std::vector<MotionDirectiveInfo>& motionDirectiveInfoList)
{
	// ���g�̂�����w�����C����S�č폜���܂��B
	this->Clear();

	// �^����ꂽ����w�����̃��X�g�̗v�f����������w���ҏW���C����V���ɒǉ����Ă����܂��B
	for (vector<MotionDirectiveInfo>::const_iterator iter = motionDirectiveInfoList.begin();
		iter != motionDirectiveInfoList.end(); iter++)
	{
		// ���g�ɓ���w���ҏW���C���� 1 �s�ǉ����܂��B
		this->AddLine();

		// �ǉ���������w���ҏW���C����
		// �Ή����铮��w���������ƂɍX�V���܂��B
		this->m_motionDirectiveEditLineList.back()->Update(*iter);
	}

	return;
}

/**
* @brief ����w��ҏW��� 1 �s�ǉ����郁���o�֐��ł��B
* 
*/
void MotionDirectiveEditPart::AddLine()
{
	// �쐬���铮��w���ҏW���C���ɗ^���� ID ���쐬���܂��B
	// ID �͓���w���ҏW���C�����ォ�牽�ԖڂɈʒu���邩��\���܂��B
	int id = GetLineCount();

	// �V���ȓ���w������쐬���܂��B
	MotionDirectiveEditLine* motionDirectiveEditLine =
		//new motionedit::MotionDirectiveEditLine(0, id);
		new motionedit::MotionDirectiveEditLine(0, id, m_motionAndValuesMap);
	// �ǉ�����J�n���ԂƏI�����Ԃ̒������s���܂��B
	{
		const double diffFromPrevEndTimeToCurStartTime = 1.0;

		const double diffFromStartTimeToEndTime = 1.0;

		double startTime = 0;

		if (id != 0)
		{
			startTime = 
				m_motionDirectiveEditLineList.back()->GetEndTime() + diffFromPrevEndTimeToCurStartTime;
		}

		motionDirectiveEditLine->SetStartTime(startTime);

		motionDirectiveEditLine->SetEndTime(startTime + diffFromStartTimeToEndTime);
	}

	// ���g�̃����o�ϐ��A
	// �y�у��C�A�E�g�ɍ쐬��������w�����ǉ����܂��B
	m_motionDirectiveEditLineList.push_back(motionDirectiveEditLine);

	// �쐬��������w���ҏW���C�������C�A�E�g�ɒǉ����܂��B
	addWidget(motionDirectiveEditLine);

	// ���g�̃V�O�i���ƍ쐬��������w����̃V�O�i����A�����܂��B
	{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		signals::connection connectionOfMotionDirectiveTypeChangedSignal = 
			motionDirectiveEditLine->MotionDirectiveTypeChanged().connect(bind(&MotionDirectiveEditPart::on_motionDirectiveTypeChanged, this, _1, _2));
		signals::connection connectionOfCoordinateTypeChangedSignal = 
			motionDirectiveEditLine->CoordinateTypeChanged().connect(bind(&MotionDirectiveEditPart::on_coordinateTypeChanged, this, _1, _2));
#else
		Connection connectionOfMotionDirectiveTypeChangedSignal = 
			motionDirectiveEditLine->MotionDirectiveTypeChanged().connect(bind(&MotionDirectiveEditPart::on_motionDirectiveTypeChanged, this, _1, _2));
		Connection connectionOfCoordinateTypeChangedSignal = 
			motionDirectiveEditLine->CoordinateTypeChanged().connect(bind(&MotionDirectiveEditPart::on_coordinateTypeChanged, this, _1, _2));
#endif
		m_connectionStackOfMotionDirectiveTypeChangedSignal.push(connectionOfMotionDirectiveTypeChangedSignal);
		
		m_connectionStackOfCoordinateTypeChangedSignal.push(connectionOfCoordinateTypeChangedSignal);
	}

	return;
}


/**
* @brief �S�Ă̓���w���ҏW���C�����폜���郁���o�֐��̎����ł��B
*/
void MotionDirectiveEditPart::Clear()
{
	// ���g�̎�����w���ҏW���C���̐����擾���A
	// 1 �s���폜���܂��B
	int count = GetLineCount();

	for (int i = 0; i <= count; i++)
	{
		DeleteLine();
	}

	util::ClearLayout(this);

	return;
}

/**
* @brief �S�Ă̓���w���ҏW���C���� 1 �ԉ�����
* �폜���Ă��������o�֐��ł��B
*/
void MotionDirectiveEditPart::DeleteLine()
{
	// ���g������w���ҏW���C���� 1 �������Ȃ��ꍇ�͉������܂���B
	if (GetLineCount() == 0)
	{
		return;
	}

	MotionDirectiveEditLine* deletingLine = 
		m_motionDirectiveEditLineList[m_motionDirectiveEditLineList.size() - 1];

	// ���g�̃V�O�i���ƍ폜���铮��w�����C���̃V�O�i���̘A�����폜���܂��B
	{
		m_connectionStackOfMotionDirectiveTypeChangedSignal.top().disconnect();

		m_connectionStackOfCoordinateTypeChangedSignal.top().disconnect();

		m_connectionStackOfMotionDirectiveTypeChangedSignal.pop();

		m_connectionStackOfCoordinateTypeChangedSignal.pop();
	}

	m_motionDirectiveEditLineList.erase(--m_motionDirectiveEditLineList.end());
}


/**
 * @biref ����w���ҏW������Ƃɓ���w�����̃��X�g���쐬���郁���o�֐��̎����ł��B
 */
vector<MotionDirectiveInfo> MotionDirectiveEditPart::MakeMotionDirectiveInfoList()
{
	vector<MotionDirectiveInfo> motionDirectiveInfoList;

	int lineCount = m_motionDirectiveEditLineList.size();

	for (vector<MotionDirectiveEditLine*>::iterator iter = m_motionDirectiveEditLineList.begin();
		iter != m_motionDirectiveEditLineList.end();
		iter++)
	{
		motionDirectiveInfoList.push_back((*iter)->MakeMotionDirectiveInfo());
	}

	return motionDirectiveInfoList;
}

/**
* @biref �^����ꂽ���{�b�g�̏������ɑ���w���ҏW������������郁���o�֐��̎����ł��B
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void  MotionDirectiveEditPart::InitializeByRobot(intrusive_ptr<IRobotInMotionEdit> robot)
#else
void  MotionDirectiveEditPart::InitializeByRobot(ref_ptr<IRobotInMotionEdit> robot)
#endif
{
	int lineCount = m_motionDirectiveEditLineList.size();

	for (vector<MotionDirectiveEditLine*>::iterator iter = m_motionDirectiveEditLineList.begin();
		iter != m_motionDirectiveEditLineList.end();
		iter++)
	{
		(*iter)->InitializeByRobot(robot);
	}

        m_motionAndValuesMap = robot->GetMotionAndDefaultValuesMap();

	return;
}

/**
* @biref �����ŗ^����ꂽ����w���̏������ɂ��āA���g���܂ގw�肳�ꂽ�C���f�b�N�X�̓���w����̓��e���X�V���郁���o�֐��̎����ł��B
*/
void MotionDirectiveEditPart::Update(
	int lineIndex,
	double startTime,
	double endTime,
	std::string motionDirectiveTypeName,
	std::string coordinateTypeName,
	const std::vector<double>& motionDirectiveValueList
	)
{
	this->m_motionDirectiveEditLineList[lineIndex]->Update(
		startTime,
		endTime,
		motionDirectiveTypeName,
		coordinateTypeName,
		motionDirectiveValueList
		);
}

/**
* @biref�@���g���܂ޓ���w��ҏW��̂����A�w�肵���C���f�b�N�X�̗�̓���J�n���Ԃ𓾂郁���o�֐��̎����ł��B
*/
int MotionDirectiveEditPart::GetStartTime(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetStartTime();
}

/**
* @biref�@���g���܂ޓ���w��ҏW��̂����A�w�肵���C���f�b�N�X�̗�̓���I�����Ԃ𓾂郁���o�֐��̎����ł��B
*/
int MotionDirectiveEditPart::GetEndTime(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetEndTime();
}

/**
* @biref�@���g���܂ޓ���w��ҏW��̂����A�w�肵���C���f�b�N�X�̗�̓���w���̎�ނ̖��O�𓾂郁���o�֐��̎����ł��B
*/
string MotionDirectiveEditPart::GetMotionDirectiveTypeName(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetMotionDirectiveTypeName();
}

/**
* @biref�@���g���܂ޓ���w��ҏW��̂����A�w�肵���C���f�b�N�X�̗�̍��W�n�̎�ނ̖��O�𓾂郁���o�֐��̎����ł��B
*/
string MotionDirectiveEditPart::GetCoordinateTypeName(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetCoordinateTypeName();
}

/**
* @biref�@���g���܂ޓ���w��ҏW��̂����A�w�肵���C���f�b�N�X�̗�̐��l��𓾂郁���o�֐��̎����ł��B
*/
vector<double> MotionDirectiveEditPart::GetMotionDirectiveValueList(int lineIndex)
{
	return  m_motionDirectiveEditLineList[lineIndex]->GetMotionDirectiveVaueList();
}

/**
* @biref�@���g���܂ޓ���w��ҏW��̂����A�w�肵���C���f�b�N�X�̗�̐��l�̐��𓾂郁���o�֐��ł��B
*/
int  MotionDirectiveEditPart::GetMotionDirectiveValueCount(int lineIndex)
{
	m_motionDirectiveEditLineList[lineIndex]->GetMotionDirectiveValueCount();
} 

/**
* @brief �����ꂩ�̍s�œ���w���̎�ނ��ύX���ꂽ�ꍇ�ɔ���������V�O�i���𓾂郁���o�֐��ł��B
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal<void(int, const QString&)> > MotionDirectiveEditPart::MotionDirectiveTypeChanged()
#else
SignalProxy<void(int, const QString&)> MotionDirectiveEditPart::MotionDirectiveTypeChanged()
#endif
{
	return m_motionDirectiveTypeChanged;
}

/**
 * @brief �����ꂩ�̍s�œ���w���̍��W�n���ύX���ꂽ�ꍇ�ɔ���������V�O�i���𓾂郁���o�֐��ł��B
 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal<void(int, const QString&)> > MotionDirectiveEditPart::CoordinateTypeChanged()
#else
SignalProxy<void(int, const QString&)> MotionDirectiveEditPart::CoordinateTypeChanged()
#endif
{
	return m_coordinateTypeChanged;
}

/**
 * @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
 * �󂯎��X���b�g�ł��B
 */
void MotionDirectiveEditPart::on_motionDirectiveTypeChanged(int lineCount, const QString& type)
{
	m_motionDirectiveTypeChanged(lineCount, type);
}

/**
 * @brief ���W�n�̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
 * �󂯎��X���b�g�ł��B
 */
void MotionDirectiveEditPart::on_coordinateTypeChanged(int lineCount, const QString& type)
{
	m_coordinateTypeChanged(lineCount, type);
}



