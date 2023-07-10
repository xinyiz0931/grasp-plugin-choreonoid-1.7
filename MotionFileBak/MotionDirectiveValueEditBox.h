#ifndef MOTIONDIRECTIVE_VALUE_EDIT_BOX_H
#define MOTIONDIRECTIVE_VALUE_EDIT_BOX_H

#include <QDoubleSpinBox>

namespace motionedit
{
	/**
	 * @brief ����w�����l�ݒ�p�̃{�b�N�X�N���X�ł��B
	 */
	class MotionDirectiveValueEditBox :public QDoubleSpinBox
	{

	public:

		/**
		 * @brief �R���X�g���N�^�ł��B
		 */
		MotionDirectiveValueEditBox(QWidget*, int);

		/**
		 * @brief ���g�̔ԍ��𓾂郁���o�֐��ł��B
		 */
		int GetId();

	private:

		/**
		 * @brief ���g�̔ԍ��ł��B
		 */
		int m_Id;
	};
}

#endif
