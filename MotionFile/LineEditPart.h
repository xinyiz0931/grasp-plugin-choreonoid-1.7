#ifndef LINE_EDIT_PART_H
#define LINE_EDIT_PART_H

#include <QLayout>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#else
#include <cnoid/Signal>
#endif

namespace cnoid
{
	class PushButton;
}

class QWidget;
class QLineEdit;
class QPushButton;
class QSpinBox;

namespace motionedit
{
	/**
	* @brief ����w���ҏW���C���̒ǉ����s��������\���N���X�ł��B
	* ����w���ҏW���C���𑝉�������{�^����L���܂��B 	 
	*/
	class LineEditPart : public QGridLayout
	{
	public:

		/**
		* @brief �R���X�g���N�^�ł��B
		*/
		LineEditPart(QWidget*);
			
		 /**
		 * @brief �ǉ�����s���𓾂郁���o�֐��ł��B
		 */
		int GetAppendingLineCount();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	         /**
		 * @brief �s�ǉ��{�^���������ꂽ�ꍇ�ɔ�������V�O�i���ł��B
		 */
		cnoid::SignalProxy<boost::signal<void(bool)> > AppendButtonClicked();
#elif defined(CNOID_15)
	         /**
		 * @brief �s�ǉ��{�^���������ꂽ�ꍇ�ɔ�������V�O�i���ł��B
		 */
		cnoid::SignalProxy<void(bool)> AppendButtonClicked();
#else
		cnoid::SignalProxy<void()> AppendButtonClicked();
#endif

	private:
		
		/**
		* @brief �s�ǉ�����I�����邽�߂̃X�s���{�b�N�X�ł��B
		*/
		QSpinBox* m_LineAppendingSpinBox;

		/**
		* @brief �s�ǉ��{�^���ł��B
		*/
		cnoid::PushButton* m_LineAppendingButton;

	};
}

#endif
