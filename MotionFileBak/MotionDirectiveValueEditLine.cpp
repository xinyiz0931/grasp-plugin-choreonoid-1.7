#include <QSpinBox>
#include <QLayout>

#include "MotionDirectiveValueEditLine.h"
#include "MotionDirectiveValueEditBox.h"

#include "MotionUtil.h"

#include<iostream>

using namespace std;
using namespace motionedit;

/**
 * @birsf �R���X�g���N�^�̎����ł��B
 */
MotionDirectiveValueEditLine::MotionDirectiveValueEditLine(QWidget* parent) : QGridLayout(parent)
{}

/**
 * @birsf ���l�ݒ�{�b�N�X�̗v�f����Ԃ������o�֐��ł��B
 */
int MotionDirectiveValueEditLine::GetMotionDirectiveValueCount()
{
	return m_valueEditBoxList.size();
}

/**
 * @birsf ���l���̓{�b�N�X��S�č폜���郁���o�֐��ł��B
 */
void MotionDirectiveValueEditLine::DeleteValueEditBoxList()
{
	// ���l�ݒ�{�b�N�X�񂪋�̏ꍇ�͉������܂���B
	if (m_valueEditBoxList.empty())
	{
		return;
	}
	int removingCount = m_valueEditBoxList.size();
	
	for (int i = 0; i < removingCount; i++)
	{
		MotionDirectiveValueEditBox* removingBox = m_valueEditBoxList[m_valueEditBoxList.size() - 1];
		m_valueEditBoxList.erase(--m_valueEditBoxList.end());
	}

	util::ClearLayout(this);

	return;
}

/**
 * @birsf �^����ꂽ���l���X�g�����Ƃɐ��l���̓{�b�N�X���쐬���郁���o�֐��̎����ł��B
 */
void MotionDirectiveValueEditLine::MakeValueBoxes(const vector<double>& valueList)
{
	// �ŏ��ɑS�Ă̐��l�ݒ�{�b�N�X���폜���܂��B
	this->DeleteValueEditBoxList();

	int pos = 0;

	for (int i = 0; i < valueList.size(); i++, pos += 2)
	{
		// ���l���̓{�b�N�X���쐬�� Id ��^���܂��B
		// ����ɁA�����ŗ^����ꂽ���l���{�b�N�X�ɐݒ肵�܂��B
		MotionDirectiveValueEditBox* valueBox = new MotionDirectiveValueEditBox(this->parentWidget(), i);
		{
			valueBox->setValue(valueList[i]);
		}

		// ���l�ݒ�{�b�N�X�����g�̃t�B�[���h
		// �y�у��C�A�E�g�ɒǉ����܂��B
		m_valueEditBoxList.push_back(valueBox);
		addWidget(valueBox, 1, pos);

		addItem(new QSpacerItem(20, 20, QSizePolicy::Fixed), 1, pos + 1);
	}

	return;
}

/**
 * @birsf ���l�ݒ�{�b�N�X�̒l��Ԃ������o�֐��̎����ł��B
 */
double MotionDirectiveValueEditLine::GetValue(int i)
{
	return m_valueEditBoxList[i]->value();
}


