#include "MotionDirectiveValueEditBox.h"
#include <QValidator>

using namespace motionedit;

/**
 * @brief �R���X�g���N�^�̎����ł��B
 */
MotionDirectiveValueEditBox::MotionDirectiveValueEditBox(QWidget* parent, int id) :QDoubleSpinBox(parent)
{
	m_Id = id;

	// ���̓{�b�N�X������͂����l�̏���Ɖ�����ݒ肵�܂��B
	const double limit = 1000.0;
	this->setRange(-limit, limit);

	// ���̓{�b�N�X�̏����_�ȉ��̌����ł��B
	const int decimals = 5;
	this->setDecimals(decimals);

	this->setSizePolicy(
		QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
		);
}

/**
 * @brief ���g�̔ԍ��𓾂郁���o�֐��̎����ł��B
 */
int MotionDirectiveValueEditBox::GetId()
{
	return m_Id;
}
