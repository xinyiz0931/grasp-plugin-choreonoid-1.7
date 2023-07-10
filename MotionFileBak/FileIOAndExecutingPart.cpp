#include "FileIOAndExecutingPart.h"

#include <QLabel>
#include <QSpacerItem>
#include <QFileDialog>

#include "MotionUtil.h"

using namespace cnoid;
using namespace std;
using namespace motionedit;

/**
* @ brief FileIOAndExecutingPart �N���X�̃R���X�g���N�^�̎����ł��B
*/
FileIOAndExecutingPart::FileIOAndExecutingPart(QWidget* parent) :QGridLayout(parent)
{
	// �ǂݍ��񂾃t�@�C������\�����镔�����쐬���܂��B
	QLabel* fileDisplayLabel = new QLabel(tr("FileName:"));

	m_fileNameDisplay = new QLabel();
	{
		m_fileNameDisplay->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// ���[�h�{�^�����쐬���܂��B
	m_loadButton = new PushButton("Load");

	// �Z�[�u�{�^�����쐬���܂��B
	m_saveButton = new PushButton("Save");

	// ���s�{�^�����쐬���܂��B
	m_playButton = new PushButton("Play");

	// �쐬�����E�B�W�F�b�g�����g�ɓo�^���Ă����܂��B
	{
		addItem(new QSpacerItem(40, 20), 1, 0);
	
		addWidget(fileDisplayLabel, 0, 1);

		addWidget(m_fileNameDisplay, 1, 1);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 2);

		addWidget(m_loadButton, 1, 3);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 4);

		addWidget(m_saveButton, 1, 5);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 6);

		addWidget(m_playButton, 1, 7);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding), 1, 8);
	}
	return;
}

/**
* @brief �ǂݍ��񂾃t�@�C���������ɕ\�������X�V���郁���o�֐��̎����ł��B
*/
void FileIOAndExecutingPart::SetLoadFileName(std::string loadFileName)
{
	QString qFileName = util::ConvertToQString(loadFileName);

	m_fileNameDisplay->setText(qFileName);


	return;
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
/**
* @brief ���[�h�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<boost::signal<void(bool)> > FileIOAndExecutingPart::LoadButtonClicked()
{
	return m_loadButton->sigClicked();
}

/**
* @brief �Z�[�u�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<boost::signal<void(bool)> > FileIOAndExecutingPart::SaveButtonClicked()
{
	return m_saveButton->sigClicked();
}

/**
* @brief �v���C�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<boost::signal<void(bool)> > FileIOAndExecutingPart::PlayButtonClicked()
{
	return m_playButton->sigClicked();
}
#elif defined(CNOID_15)
/**
* @brief ���[�h�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<void(bool)> FileIOAndExecutingPart::LoadButtonClicked()
{
	return m_loadButton->sigClicked();
}

/**
* @brief �Z�[�u�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<void(bool)> FileIOAndExecutingPart::SaveButtonClicked()
{
	return m_saveButton->sigClicked();
}

/**
* @brief �v���C�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<void(bool)> FileIOAndExecutingPart::PlayButtonClicked()
{
	return m_playButton->sigClicked();
}
#else
/**
* @brief ���[�h�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<void()> FileIOAndExecutingPart::LoadButtonClicked()
{
	return m_loadButton->sigClicked();
}

/**
* @brief �Z�[�u�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<void()> FileIOAndExecutingPart::SaveButtonClicked()
{
	return m_saveButton->sigClicked();
}

/**
* @brief �v���C�{�^���������ꂽ�ꍇ�ɔ�������V�O�i����Ԃ������o�֐��̎����ł��B
*/
SignalProxy<void()> FileIOAndExecutingPart::PlayButtonClicked()
{
	return m_playButton->sigClicked();
}
#endif


