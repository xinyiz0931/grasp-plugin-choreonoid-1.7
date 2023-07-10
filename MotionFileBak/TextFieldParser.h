#ifndef TEXTFILE_PARSER_H
#define TEXTFILE_PARSER_H

#include <iostream>
#include <string>
#include <vector>

#include <cnoid/Referenced>

namespace motionedit
{
	/**
	* @brief �X�g���[�����̃f�[�^���p�[�X����C���^�[�t�F�[�X�ł��B
	*�@���g�������o�Ƃ��Ď���؂蕶�����X�g�y�уR�����g�A�E�g�g�[�N�����X�g�����Ƃ�
        * ������̃��C�����p�[�X���Ă����܂��B
	*/
	class ITextFieldParser : public cnoid::Referenced
	{
	public:

		ITextFieldParser(){ }

		/**
		 * @brief �p�[�X�̑ΏۂƂȂ�X�g���[����ݒ肷�鏃�����z�֐��ł��B
		 */
		virtual void SetStream(std::istream*) = 0;

		/**
		 * @brief �X�g���[�����̃f�[�^���p�[�X���鏃�����z�֐��ł��B
		 */
		virtual std::vector<std::string> ReadFiels() = 0;

		/**
		 * @brief �X�g���[���̍Ō�ɓ��B�������ǂ����𔻒肷�鏃�����z�֐��ł��B
		 */
		virtual bool HasReachedEndOfData() = 0;
	};

	/**
	 * @brief �X�g���[�����̃f�[�^���p�[�X����N���X�ł��B
	 */
	class TextFieldParser : public ITextFieldParser
	{
	public:

		TextFieldParser();

		/**
		 * @brief �p�[�X�̑ΏۂƂȂ�X�g���[����ݒ肷�郁���o�֐��ł��B
		 */
		void SetStream(std::istream*);

		/**
		 * @brief �X�g���[�����̃f�[�^���p�[�X���郁���o�֐����ł��B
		 */
		std::vector<std::string> ReadFiels();

		/**
		 * @brief �X�g���[���̍Ō�ɓ��B�������ǂ����𔻒肷�郁���o�֐��ł��B
		 */
		bool HasReachedEndOfData();

		/**
		 * @brief �f���~�^��ǉ����郁���o�֐��ł��B
		 */
		void AddDelimiter(std::string);

		/**
		 * @brief �R�����g�s�ł��邱�Ƃ�\��������ǉ����郁���o�֐��ł��B
		 */
		void AddCommentToken(std::string);

	private :

		/**
     �@	 * @brief �����N���X�ł��B
		 */
		class Impl;

		/**
		 * @brief �����ւ̃|�C���^�ł��B
		 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<Impl> mImpl;
#else
		cnoid::ref_ptr<Impl> mImpl;
#endif
	};
}

#endif
