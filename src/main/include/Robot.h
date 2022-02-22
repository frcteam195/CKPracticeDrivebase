// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <vector>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include "DriveHelper.hpp"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "AHRS.h"

class Robot : public frc::TimedRobot
{
public:
	enum class ROBOT_POWER_MODE
	{
		LOW,
		MED,
		FULL,
		INVALID
	};

	void RobotInit() override;
	void RobotPeriodic() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void DisabledInit() override;
	void DisabledPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

	void SimulationInit() override;
	void SimulationPeriodic() override;

private:
	void setupGenericMotor(TalonFX* motor);
	void setupFollowerMotor(TalonFX* motor, TalonFX* master);
	void configRobotMode(ROBOT_POWER_MODE robotMode);
	ROBOT_POWER_MODE getJumperMode();
	ROBOT_POWER_MODE getButtonMode();

	TalonFX* mLeftMaster;
	TalonFX* mRightMaster;

	TalonFX* mLeftFollower1;
	TalonFX* mLeftFollower2;
	TalonFX* mRightFollower1;
	TalonFX* mRightFollower2;

	ROBOT_POWER_MODE mPrevRobotMode;
	ROBOT_POWER_MODE mCurrRobotMode;

	frc::DigitalInput* mFastModeEnableInput;
	frc::DigitalInput* mMedModeEnableInput;
	frc::DigitalOutput* mLEDOutput;

	frc::Joystick* mJoystick;
	DriveHelper mDriveHelper;

	frc::Joystick* mButtonBox1;

	bool mPrevBrakeMode;

	std::vector<TalonFX*> mAllMotors;
	std::vector<TalonFX*> mMasterMotors;
	std::vector<TalonFX*> mFollowerMotors;
	std::shared_ptr<nt::NetworkTable> networkTable;

	AHRS* mNavX;

	static constexpr double DRIVE_JOYSTICK_DEADBAND = 0.05;
};
