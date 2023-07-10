#ifndef MOTIONDIRECTIVEINFO_H
#define MOTIONDIRECTIVEINFO_H

#include <map>
#include <Eigen/Dense>

namespace motionedit
{
	/**
	 * @brief ���W�n�̎�ނ�\���񋓑̂ł��B
	 */
	enum CoordinateType 
	{
		CoordinateType_None,
		CoordinateType_Abs,
		CoordinateType_Rel,
	};

	/**
	 * @brief ����w���̎�ނ�\���񋓑̂ł��B
	 */
	enum MotionDirectiveType 
	{
		MotionType_None,
		MotionType_Pause,
		MotionType_HeadForward,
		MotionType_Joint,	
		MotionType_Arm_Xyz,
		MotionType_Arm_Linear,
		MotionType_Arm_Jnt,
		MotionType_Arm_Object,
		MotionType_Hand_Jnt_Open,
		MotionType_Hand_Jnt_Close,
		MotionType_Hand_Grasp,
	};

	/**
	 * @brief �g�p����r�̎�ނ�\���񋓑̂ł��B
	 */
	enum UsingHandType 
	{
		UsingHandType_None,
		UsingHandType_Right,
		UsingHandType_Left,
	};

	/**
         * @brief ����w���̓��e���i�[����N���X�ł��B
         */
	class MotionDirectiveInfo
	{
	public:
	
		/**
	 �@  * @brief ����̊J�n���Ԃ𓾂郁���o�֐��ł��B
	     */
		double GetStartTime() const;
		
		/**
	   �@* @brief ����̏I�����Ԃ𓾂郁���o�֐��ł��B
	�@   */
		double GetEndTime() const;
		
		UsingHandType GetUsingHandType() const;

		/**
		 * @brief ����w���̎�ނ𓾂郁���o�֐��ł��B
		 */
		MotionDirectiveType GetMotionDirectiveType() const;

		/**
		 * @brief ���W�n�𓾂郁���o�֐��ł��B
		 */
		CoordinateType GetCoordinateType() const;

		/**
	     * @brief ����w���̍ۂɐݒ肷��l�̐��𓾂郁���o�֐��ł��B
	     */
		int GetDirectiveValueCount() const;
		
		/**
	     * @brief ����w���̍ۂɐݒ肷��l�̂����Aindex �Ԗڂ̒l�𓾂郁���o�֐��ł��B
	     */
		double GetDirectiveValue(int index) const;
		
		/**
		 * @brief ����w���̍ۂɐݒ肷��l���x�N�g���Ƃ��ē��郁���o�֐��ł��B
		 */
		Eigen::VectorXd GetDirectiveValueVector() const;

		/**
		 * @brief ����̊J�n���Ԃ��Z�b�g���郁���o�֐��ł��B
		 */
		void SetStartTime(double);

		/**
		 * @brief ����̏I�����Ԃ��Z�b�g���郁���o�֐��ł��B
		 */
		void SetEndTime(double);

		void SetUsingHandType(UsingHandType);

		/**
		 * @brief ����w���̎�ނ��Z�b�g���郁���o�֐��ł��B
		 */
		void SetMotionDirectiveType(MotionDirectiveType);

		/**
		 * @brief ���W�n���Z�b�g���郁���o�֐��ł��B
		 */
		void SetCoordinateType(CoordinateType);

		/**
		 * @brief ����w���̍ۂɐݒ肷��l���܂Ƃ߂��x�N�g�����Z�b�g���郁���o�֐��ł��B
		 */
		void SetDirectiveValueVector(Eigen::VectorXd&);

		/**
		 * @brief ����w���̍ۂɐݒ肷��l���Z�b�g���郁���o�֐��ł��B
		 */
		void SetMotionDirectiveValue(int, double);

		/**
		 * @brief ���g�̎����𕶎���ɕϊ����郁���o�֐��ł��B
		 */
		std::string ToString();

	private:

		/**
		 * @brief ����̊J�n���Ԃł��B
		 */
		double m_startTime;

		/**
		 * @brief ����̏I�����Ԃł��B
		 */
		double m_endTime;

		UsingHandType m_usingHandType;

		/**
	  �@ * @brief ����w���̎�ނł��B
		 */
		MotionDirectiveType m_motionDirectiveType;

		/**
    	 * @brief ���W�n�ł��B
		 */
		CoordinateType m_coordinateType;

		/**
		 * @brief ����w���̍ۂɐݒ肷��l���܂Ƃ߂��x�N�g���ł��B
		 */
		Eigen::VectorXd m_directiveValueVector;
	};
}

#endif
