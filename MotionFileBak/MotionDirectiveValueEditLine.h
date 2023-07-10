#ifndef MOTIONDIRECTIVE_VALUE_EDIT_LINE_H
#define MOTIONDIRECTIVE_VALUE_EDIT_LINE_H

#include <vector>

#include <QWidget>
#include <QFrame>
#include <QLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>

#include <Eigen/Dense>

namespace motionedit
{
	class MotionDirectiveValueEditBox;

	/**
	 * @birsf ����w���̐��l���\���N���X�ł��B 
	 */
	class MotionDirectiveValueEditLine : public QGridLayout
	{
		//Q_OBJECT

	public:

		/**
		 * @birsf �R���X�g���N�^�ł��B
		 */
		MotionDirectiveValueEditLine(QWidget* parent);

		/**
		 * @birsf ���l���̓{�b�N�X��S�č폜���郁���o�֐��ł��B
		 */
		void DeleteValueEditBoxList();

		/**
		 * @birsf �^����ꂽ���l���X�g�����Ƃɐ��l���̓{�b�N�X���쐬���郁���o�֐��ł��B
		 */
		void MakeValueBoxes(const std::vector<double>&);

		/**
		 * @birsf ���l�ݒ�{�b�N�X�̗v�f����Ԃ������o�֐��ł��B
		 */
		int GetMotionDirectiveValueCount();

		/**
		 * @birsf ���l�ݒ�{�b�N�X�̒l��Ԃ������o�֐��ł��B
		 */
		double GetValue(int);

		/**
		 * @birsf ����̐��l���ω����ꂽ�ꍇ�ɔ���������V�O�i���ł��B
		 */
		void MotionDirectiveValueChanged(int, double);

	private:

		/**
		 * @birsf ���l�ݒ�{�b�N�X�̃��X�g�ł��B
	 	 */
		std::vector<MotionDirectiveValueEditBox* > m_valueEditBoxList;
	};
}


#endif
