#include"LineEditPart.h"

#include <cnoid/Button>

#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>

using namespace cnoid;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
using namespace boost;
#endif
using namespace motionedit;

 /**
 * @brief �R���X�g���N�^�̎����ł��B
 */
LineEditPart::LineEditPart(QWidget* parent) : QGridLayout(parent)
{
	// �s�ǉ��X�s���{�b�N�X���쐬���܂��B
	m_LineAppendingSpinBox = new QSpinBox();
	{
		m_LineAppendingSpinBox->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// �s�ǉ��{�^�����쐬���܂��B
	m_LineAppendingButton = new cnoid::PushButton("&Append");
	{
		m_LineAppendingButton->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// �쐬�����E�B�W�F�b�g�����g�ɓo�^���Ă����܂��B
	{
		addItem(new QSpacerItem(40, 20), 1, 0);
	
		addWidget(m_LineAppendingSpinBox, 1, 1);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 2);

		addWidget(m_LineAppendingButton, 1, 3);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding), 1, 4);
	}
}

/**
* @brief �ǉ�����s���𓾂郁���o�֐��ł��B
*/
int LineEditPart::GetAppendingLineCount()
{
	return m_LineAppendingSpinBox->value();
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
/**
* @brief �s�ǉ��{�^���������ꂽ�ꍇ�ɔ�������V�O�i���ł��B
*/
SignalProxy<signal<void(bool)> >LineEditPart::AppendButtonClicked()
{
	return m_LineAppendingButton->sigClicked();
}
#elif defined(CNOID_15)
/**
* @brief �s�ǉ��{�^���������ꂽ�ꍇ�ɔ�������V�O�i���ł��B
*/
SignalProxy<void(bool)>LineEditPart::AppendButtonClicked()
{
	return m_LineAppendingButton->sigClicked();
}
#else
SignalProxy<void()>LineEditPart::AppendButtonClicked()
{
	return m_LineAppendingButton->sigClicked();
}
#endif


