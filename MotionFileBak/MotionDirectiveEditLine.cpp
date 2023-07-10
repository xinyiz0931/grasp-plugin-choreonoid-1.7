#include <iostream>

#include <QLabel>
#include <QLayout>
#include <QSpacerItem>
#include <QMessageBox>

#include <Eigen/Dense>

#include "MotionDirectiveInfo.h"
#include "MotionDirectiveEditLine.h"
#include "MotionDirectiveValueEditLine.h"
#include "MotionUtil.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace Eigen;
using namespace motionedit;

/**
 * @brief �R���X�g���N�^�̎����ł��B
 */
MotionDirectiveEditLine::MotionDirectiveEditLine(QWidget* parent, int id) :QFrame(parent)
{
	// Id ��ݒ肵�܂��B
	m_Id = id;
	
	Initialize(parent);
	
}

MotionDirectiveEditLine::MotionDirectiveEditLine(QWidget* parent, int id, const std::map<std::string, std::vector<double> >& motionAndValuesMap):QFrame(parent)
{
	m_Id = id;

	m_motionAndValuesMap = motionAndValuesMap;
	
	Initialize(parent);
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void MotionDirectiveEditLine::InitializeByRobot(intrusive_ptr<IRobotInMotionEdit> robot)
#else
void MotionDirectiveEditLine::InitializeByRobot(ref_ptr<IRobotInMotionEdit> robot)
#endif
{
	m_motionAndValuesMap = robot->GetMotionAndDefaultValuesMap();
	
	{
		for(int i = 0; i < m_motionDirectiveNameBox->count(); i++)
		{
			m_motionDirectiveNameBox->removeItem(0);
		}

		vector<string> motionNameList = util::GetKeyList(m_motionAndValuesMap);
		for(vector<string>::iterator iter = motionNameList.begin(); iter != motionNameList.end(); iter++)
		{
			m_motionDirectiveNameBox->addItem(util::ConvertToQString(*iter));
		}
	}

	m_currentMotionDirectiveTypeName = m_motionDirectiveNameBox->currentText();

	on_motionDirectiveType_Changed(m_motionDirectiveNameBox->currentIndex());
}

void MotionDirectiveEditLine::Initialize(QWidget* parent)
{
	// ����̊J�n���Ԃ̃��x�����쐬���܂��B
	QLabel* startTimeLabel = new QLabel(tr("StartTime:"));

	// ����̊J�n���Ԃ�ݒ肷��{�b�N�X���쐬���܂��B
	m_startTimeEdit = new QDoubleSpinBox;
	{
		m_startTimeEdit->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// ����̏I�����Ԃ̂����x�����쐬���܂��B
	QLabel* endTimeLabel = new QLabel(tr("EndTime:"));

	// ����̏I�����Ԃ�ݒ肷��{�b�N�X���쐬���܂��B
	m_endTimeEdit = new QDoubleSpinBox;
	{
		m_endTimeEdit->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// ����w���̎�ނ�ݒ肷�镔���̃��x�����쐬���܂��B
	QLabel* motionLabel = new QLabel(tr("Motion:"));

	// ����w���̎�ނ�ݒ肷��{�b�N�X���쐬���܂��B
	m_motionDirectiveNameBox = new ComboBox;
	{
		m_motionDirectiveNameBox->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
		if(m_motionAndValuesMap.empty())
		{
			m_motionDirectiveNameBox->addItem("NONE");
		}
		else
		{
			vector<string> motionNameList = util::GetKeyList(m_motionAndValuesMap);

			for(vector<string>::iterator iter = motionNameList.begin(); iter != motionNameList.end(); iter++)
			{
				m_motionDirectiveNameBox->addItem(util::ConvertToQString(*iter));
			}
		}
	}

	// ����w���̍��W��ݒ肷�镔���̃��x�����쐬���܂��B
	QLabel* coordinateLabel = new QLabel("Coordinate:");

	// ����w���̍��W��ݒ肷��{�b�N�X���쐬���܂��B
	m_coordinateSystemNameBox = new ComboBox;
	{
		m_coordinateSystemNameBox->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
		m_coordinateSystemNameBox->addItem("NONE");
		m_coordinateSystemNameBox->addItem("ABS");
		m_coordinateSystemNameBox->addItem("REL");
	}

	// ����w���̐��l����쐬���܂��B
	m_motionDirectiveValueEditLine = new MotionDirectiveValueEditLine(parent);
	
	if(!m_motionAndValuesMap.empty())
	{
		 m_motionDirectiveValueEditLine->MakeValueBoxes(m_motionAndValuesMap[util::ConvertToString(m_motionDirectiveNameBox->currentText())]);
	}

	// ���g�̃��C�A�E�g�ɍ쐬�����E�B�W�F�b�g��ǉ����܂��B
	{
		QGridLayout* layout = new QGridLayout();
		layout->addItem(new QSpacerItem(40, 20), 1, 0);
		layout->addWidget(startTimeLabel, 0, 1);
		layout->addWidget(m_startTimeEdit, 1, 1);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 2);
		layout->addWidget(endTimeLabel, 0, 3);
		layout->addWidget(m_endTimeEdit, 1, 3);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 4);
		layout->addWidget(motionLabel, 0, 5);
		layout->addWidget(m_motionDirectiveNameBox, 1, 5);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 6);
		layout->addWidget(coordinateLabel, 0, 7);
		layout->addWidget(m_coordinateSystemNameBox, 1, 7);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 8);
	        layout->addLayout(m_motionDirectiveValueEditLine, 1, 9);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding), 1, 10);
		this->setLayout(layout);
	}
	
	// ���g�̃X���b�g�ƍ쐬�����E�B�W�F�b�g�̃V�O�i�����������܂��B
	{
		m_motionDirectiveNameBox->sigCurrentIndexChanged().connect(bind(&MotionDirectiveEditLine::on_motionDirectiveType_Changed, this, _1));	
	}
	
	m_currentMotionDirectiveTypeName = m_motionDirectiveNameBox->currentText();

	return;
}

/**
 * @brief ���g�� Id �𓾂郁���o�ϐ��̎����ł��B
 */
int MotionDirectiveEditLine::GetId()
{
	return m_Id;
}

/**
 * @brief ����J�n���Ԃ𓾂郁���o�֐��̎����ł��B
 */
double MotionDirectiveEditLine::GetStartTime()
{
	return m_startTimeEdit->value();
}

/**
 * @brief ����I�����Ԃ𓾂郁���o�֐��̎����ł��B
 */
double MotionDirectiveEditLine::GetEndTime()
{
	return m_endTimeEdit->value();
}

string MotionDirectiveEditLine::GetMotionDirectiveTypeName()
{
	return util::ConvertToString(m_motionDirectiveNameBox->currentText());
}

string MotionDirectiveEditLine::GetCoordinateTypeName()
{
 	return util::ConvertToString(m_coordinateSystemNameBox->currentText());
}

vector<double> MotionDirectiveEditLine::GetMotionDirectiveVaueList()
{

	int valueEditBoxCount =
		this->m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount();
	
	vector<double> valueList;

	for (int i = 0; i < valueEditBoxCount; i++)
	{
		valueList.push_back(m_motionDirectiveValueEditLine->GetValue(i));
	}

	return valueList;
	
}

int MotionDirectiveEditLine::GetMotionDirectiveValueCount()
{
	return m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount(); 
}

/**
 * @brief ����w���̎�ނ̖��O��ݒ肷�郁���o�֐��ł��B
 */
void MotionDirectiveEditLine::SetMotionDirectiveTypeName(const QString& motionDirectiveTypeName)
{
	int index = m_motionDirectiveNameBox->findText(motionDirectiveTypeName);

	m_motionDirectiveNameBox->setCurrentIndex((index >= 0) ? index : 0 );
}

/**
 * @brief ���W�n�̎�ނ̖��O��ݒ肷�郁���o�֐��ł��B
 */
void MotionDirectiveEditLine::SetCoordinateTypeName(const QString& coordinateTypeName)
{
	int index = m_coordinateSystemNameBox->findText(coordinateTypeName);
		
	m_coordinateSystemNameBox->setCurrentIndex((index >= 0) ? index : 0);
}

/**
 * @brief ����J�n���Ԃ�ݒ肷�郁���o�֐��̎����ł��B
 */
void MotionDirectiveEditLine::SetStartTime(double startTime)
{
	m_startTimeEdit->setValue(startTime);
}

/**
* @brief ����I�����Ԃ�ݒ肷�郁���o�֐��̎����ł��B
*/
void MotionDirectiveEditLine::SetEndTime(double endTime)
{
	m_endTimeEdit->setValue(endTime);
}

/**
 * @brief �^����ꂽ����w���������ƂɎ��g���X�V���郁���o�ϐ��̎����ł��B
 */
void MotionDirectiveEditLine::Update(const MotionDirectiveInfo& motionDirectiveInfo)
{
	// ����̊J�n���ԂƏI�����Ԃ��X�V���܂��B
	this->m_startTimeEdit->setValue(motionDirectiveInfo.GetStartTime());
	this->m_endTimeEdit->setValue(motionDirectiveInfo.GetEndTime());

	// ����w���̐ݒ萔�l���X�g�𓾂܂� 
	vector<double> valueList = util::Convert(motionDirectiveInfo.GetDirectiveValueVector());
	
	// �ݒ萔�l���X�g�����ƂɎ��g�̐��l�ҏW�{�b�N�X���X�V���܂��B
	this->m_motionDirectiveValueEditLine->MakeValueBoxes(valueList);
}

/**
 * @brief �^����ꂽ����w���������Ƃ�
 * ����w�������쐬���郁���o�֐��̎����ł��B
 */
MotionDirectiveInfo MotionDirectiveEditLine::MakeMotionDirectiveInfo()
{
	MotionDirectiveInfo motionDirectiveInfo;

	motionDirectiveInfo.SetStartTime(this->m_startTimeEdit->value());
	motionDirectiveInfo.SetEndTime(this->m_endTimeEdit->value());
	
	motionDirectiveInfo.SetMotionDirectiveType(MotionType_None);

	motionDirectiveInfo.SetCoordinateType(CoordinateType_None);

	int valueEditBoxCount =
		this->m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount();

	{
		VectorXd valueList(valueEditBoxCount);

		for (int i = 0; i < valueEditBoxCount; i++)
		{
			valueList[i] = m_motionDirectiveValueEditLine->GetValue(i);
		}
	}

	return motionDirectiveInfo;

}

void MotionDirectiveEditLine::Update(
	double startTime,
	double endTime,
	std::string motionDirectiveTypeName,
	std::string coordinateTypeName,
	const std::vector<double>& motionDirectiveValueList
	)
{

	this->m_startTimeEdit->setValue(startTime);

	this->m_endTimeEdit->setValue(endTime);
	
	QString qMotionDirectiveTypeName = util::ConvertToQString(motionDirectiveTypeName);

	this->SetMotionDirectiveTypeName(qMotionDirectiveTypeName);

	QString qCoordinateTypeName = util::ConvertToQString(coordinateTypeName);

	this->SetCoordinateTypeName(qCoordinateTypeName);

	this->m_motionDirectiveValueEditLine->MakeValueBoxes(motionDirectiveValueList);
}

vector<double> MotionDirectiveEditLine::GetValueList()
{
	vector<double> valueList;
	{
		for(int i = 0; i < m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount();i++)
		{
			valueList.push_back(m_motionDirectiveValueEditLine->GetValue(i));
		}
	}
	return valueList;
}

/**
�@* @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal< void(int, const QString&)> > MotionDirectiveEditLine::MotionDirectiveTypeChanged()
#else
SignalProxy<void(int, const QString&)> MotionDirectiveEditLine::MotionDirectiveTypeChanged()
#endif
{
	return m_motionDirectiveTypeChanged;
}

/**
 * @brief ����w���̍��W�n���ύX���ꂽ�ꍇ�ɔ���������V�O�i���ł��B
 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal< void(int, const QString&)> > MotionDirectiveEditLine::CoordinateTypeChanged()
#else
SignalProxy< void(int, const QString&)> MotionDirectiveEditLine::CoordinateTypeChanged()
#endif
{
	return m_coordinateTypeChanged;
}

/**
 * @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
 * �󂯎��X���b�g�̎����ł��B
 */
void MotionDirectiveEditLine::on_motionDirectiveTypeChanged(const QString& motionDiectiveTypeName)
{
	// ���g�� ID ����ǉ����ăV�O�i�����Ĕ��s���܂��B	
	//QQQ##        
	//emit MotionDirectiveTypeChanged(m_Id, motionDiectiveTypeName);

	m_motionDirectiveTypeChanged(m_Id, motionDiectiveTypeName);
}

/**
 * @brief ���W�n���ύX���ꂽ�ꍇ�ɔ�������V�O�i����
 * �󂯎��X���b�g�̎����ł��B
 */
void MotionDirectiveEditLine::on_coordinateTypeChanged(const QString& coordinateTypeName)
{
	// ���g�� ID ����ǉ����ăV�O�i�����Ĕ��s���܂��B
	m_coordinateTypeChanged(m_Id, coordinateTypeName);
}

/**
 * @brief ����w���̎�ނ��ύX���ꂽ�ꍇ�ɔ�������V�O�i����
 * �󂯎��X���b�g�̎����ł��B
 */
void MotionDirectiveEditLine::on_motionDirectiveType_Changed(int index)
{
	QString motionDirectiveTypeName = m_motionDirectiveNameBox->currentText();

	m_motionAndValuesMap[util::ConvertToString(m_currentMotionDirectiveTypeName)] = GetValueList();

	string stdMotionDirectiveTypeName = util::ConvertToString(motionDirectiveTypeName);
	
	if(m_motionAndValuesMap.find(stdMotionDirectiveTypeName) == m_motionAndValuesMap.end())
	{
		return ;
	}
		
	m_motionDirectiveValueEditLine->MakeValueBoxes(m_motionAndValuesMap[stdMotionDirectiveTypeName]);

	return;
}



