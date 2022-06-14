
#include "TextFieldParser.h"
#include <sstream>
#include <boost/algorithm/string.hpp>   
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;
using namespace motionedit;

/**
 * @brief �e�L�X�g �p�[�T �N���X�̎����N���X�ł��B
 */
class TextFieldParser::Impl : public cnoid::Referenced
{
public:

	/**
	 * @brief �p�[�X�̑ΏۂƂȂ�X�g���[���ł��B
	 */
	std::istream* istream;

	/**
	 * @brief �f���~���^�����̏W���ł��B
	 */
	string delimiterList;

	/**
	 * @brief �R�����g�s�ł��邱�Ƃ�\���g�[�N�������̏W���ł��B
	 */
	string commentTokenList;

	/**
	 * @brief �^����ꂽ������𕪊��������ʂ�Ԃ������o�֐��ł��B
	 */
	vector<string> Split(string& line);

	/**
	 * @brief �^����ꂽ�����񂪃R�����g�s��\�����ǂ����𔻒肷�郁���o�֐��ł��B
	 */
	bool IsCommentLine(string& line);

	/**
	 * @brief �^����ꂽ�����񂪃X�L�b�v���ׂ��s���ǂ����𔻒肷�郁���o�֐��ł��B
	 */
	bool IsSkipLine(string& line);
};

/***
 * @brief �^����ꂽ���������؂蕶����p���ĕ������郁���o�֐��̎����ł��B 
 */
vector<string> TextFieldParser::Impl::Split(string& line)
{
	vector<string> splitStringList;

	typedef char_separator<char>     BOOST_CHAR_SEP;
	typedef tokenizer< BOOST_CHAR_SEP > BOOST_TOKENIZER;

	BOOST_CHAR_SEP sep(delimiterList.c_str());
	BOOST_TOKENIZER tokens(line, sep);

	BOOST_FOREACH(string s, tokens) {
		splitStringList.push_back(s);
	}

	return splitStringList;
}

/***
 * @brief �^����ꂽ�����񂪃R�����g�s��\�����ǂ����𔻒肷�郁���o�֐��ł��B
 */
bool TextFieldParser::Impl::IsCommentLine(string& line)
{
	// �z���C�g�X�y�[�X�ł͂Ȃ��ŏ��̕��������܂��B
	size_t pos = line.find_first_not_of(" \t\n");
	// ������ɋ󔒈ȊO�̂��̂�������Ȃ������ꍇ�̓R�����g���C���ł͂Ȃ��Ɣ��肵�܂��B(�X�L�b�v���C���̉\���͂���)
	if (pos == string::npos)
	{
		return false;
	}
	// �p�[�T�[�I�u�W�F�N�g�����R�����g���蕶���������邩�ǂ����Ŕ��肵�܂��B
	return commentTokenList.find(line[pos]) != string::npos;
}

/***
* @brief �^����ꂽ�����񂪃X�b�L�v���C�����ǂ����𔻒肷�郁���o�֐��ł��B
*/
bool TextFieldParser::Impl::IsSkipLine(string& line)
{
	// ��s�ɋ󔒂����܂܂�Ă��Ȃ��ꍇ�̓X�L�b�v����s�Ɣ��肵�܂��B
	return line.find_first_not_of(" \t\n") == string::npos;
}

/***
* @brief ��A�̕�����̃p�[�X���s���N���X�ł��B
*/
TextFieldParser::TextFieldParser() : mImpl(new Impl())
{
	mImpl->istream = 0;
}

// �p�[�X�̑ΏۂƂȂ�X�g���[�����Z�b�g���郁���o�֐��̎����ł��B
void TextFieldParser::SetStream(istream* stream)
{
	mImpl->istream = stream;
}

// ��؂蕶����ǉ����郁���o�֐��̎����ł��B
void TextFieldParser::AddDelimiter(string delimiiter)
{
	this->mImpl->delimiterList += delimiiter;
}

// �R�����g����p�̕������ǉ����郁���o�֐��̎����ł��B
void TextFieldParser::AddCommentToken(string comentToken)
{
	this->mImpl->commentTokenList += comentToken;
}

// �t�@�C���ǂݍ��̍Ō�̍s�ɒB�������ǂ����𔻒肷�郁���o�֐��̎����ł��B
bool TextFieldParser::HasReachedEndOfData()
{
	return mImpl->istream->eof();
}

/***
* @brief �p�[�T�[�̌��݂̈ʒu�ɂ��镶������p�[�X�������ʂ̕����񃊃X�g��Ԃ������o�֐��ł��B
*/
vector<string> TextFieldParser::ReadFiels()
{
	// �p�[�X�̑ΏۂƂȂ镶������i�[����ϐ��ł��B
	string line;

	// �R�����g�s�y�ы󔒍s�ł͂Ȃ��ŏ��̍s�̕���������܂��B
	while (getline(*mImpl->istream, line))
	{
		if ((mImpl->IsSkipLine(line)) || (mImpl->IsCommentLine(line)))
		{
			continue;
		}
		break;
	}

	//�ǂݍ��񂾍s����̏ꍇ�͂���̃R���e�i��Ԃ��܂��B
	if (line.empty())
	{
		return vector<string>();
	}

	// ����ꂽ������𕪉��������̂�Ԃ��܂��B
	return mImpl->Split(line);
}


