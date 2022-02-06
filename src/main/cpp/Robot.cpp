// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "PhoenixHelper.hpp"
#include "CKMath.hpp"

void Robot::RobotInit()
{
	mFastModeEnableInput = new frc::DigitalInput(0);
	mMedModeEnableInput = new frc::DigitalInput(1);
	mLEDOutput = new frc::DigitalOutput(9);
	mLEDOutput->Set(false);

	mLeftMaster = new TalonFX(1);
	mRightMaster = new TalonFX(4);
	mLeftFollower1 = new TalonFX(2);
	mLeftFollower2 = new TalonFX(3);
	mRightFollower1 = new TalonFX(5);
	mRightFollower2 = new TalonFX(6);

	mJoystick = new frc::Joystick(0);

	mMasterMotors.push_back(mLeftMaster);
	mMasterMotors.push_back(mRightMaster);

	mFollowerMotors.push_back(mLeftFollower1);
	mFollowerMotors.push_back(mLeftFollower2);
	mFollowerMotors.push_back(mRightFollower1);
	mFollowerMotors.push_back(mRightFollower2);

	mAllMotors.insert(mAllMotors.end(), mMasterMotors.begin(), mMasterMotors.end());
	mAllMotors.insert(mAllMotors.end(), mFollowerMotors.begin(), mFollowerMotors.end());

	for (TalonFX* tfx : mAllMotors)
	{
		setupGenericMotor(tfx);
	}
	ck::runTalonFunctionWithRetry([&]() { mRightMaster->SetInverted(true); return mRightMaster->GetLastError(); }, mRightMaster->GetDeviceID());
	
	setupFollowerMotor(mLeftFollower1, mLeftMaster);
	setupFollowerMotor(mLeftFollower2, mLeftMaster);
	setupFollowerMotor(mRightFollower1, mRightMaster);
	setupFollowerMotor(mRightFollower2, mRightMaster);

	mPrevBrakeMode = false;
}

void Robot::setupGenericMotor(TalonFX* motor)
{
	ck::runTalonFunctionWithRetry([&]() { return motor->ConfigFactoryDefault(); }, motor->GetDeviceID());
	ck::runTalonFunctionWithRetry([&]() { return motor->ConfigVoltageCompSaturation(12); }, motor->GetDeviceID());
	ck::runTalonFunctionWithRetry([&]() { motor->EnableVoltageCompensation(true); return motor->GetLastError(); }, motor->GetDeviceID());
	ck::runTalonFunctionWithRetry([&]() { motor->SetNeutralMode(NeutralMode::Coast); return motor->GetLastError(); }, motor->GetDeviceID());
}

void Robot::configRobotMode(ROBOT_POWER_MODE robotMode)
{
	SupplyCurrentLimitConfiguration defaultCurrConfig;
	switch (robotMode)
	{
		case ROBOT_POWER_MODE::LOW:
		{
			defaultCurrConfig = SupplyCurrentLimitConfiguration(true, 15, 0, 0);
			break;
		}
		case ROBOT_POWER_MODE::MED:
		{
			defaultCurrConfig = SupplyCurrentLimitConfiguration(true, 30, 0, 0);
			break;
		}
		case ROBOT_POWER_MODE::FULL:
		{
			defaultCurrConfig = SupplyCurrentLimitConfiguration(true, 60, 0, 0);
			break;
		}
		case ROBOT_POWER_MODE::INVALID:
		{
			defaultCurrConfig = SupplyCurrentLimitConfiguration(true, 1, 0, 0);
			break;
		}
	}
	for (TalonFX* tfx : mAllMotors)
	{
		ck::runTalonFunctionWithRetry([&]() { return tfx->ConfigGetSupplyCurrentLimit(defaultCurrConfig); }, tfx->GetDeviceID());
	}
}

void Robot::setupFollowerMotor(TalonFX* motor, TalonFX* master)
{
	ck::runTalonFunctionWithRetry([&]() { motor->Follow(*master); return motor->GetLastError(); }, motor->GetDeviceID());
	ck::runTalonFunctionWithRetry([&]() { motor->SetInverted(InvertType::FollowMaster); return motor->GetLastError(); }, motor->GetDeviceID());
}

Robot::ROBOT_POWER_MODE Robot::getJumperMode()
{
	if (!mFastModeEnableInput->Get())
	{
		return ROBOT_POWER_MODE::FULL;
	}
	else if (!mMedModeEnableInput->Get())
	{
		return ROBOT_POWER_MODE::MED;
	}
	return ROBOT_POWER_MODE::LOW;
}

void Robot::RobotPeriodic()
{
	mCurrRobotMode = getJumperMode();
	if (mCurrRobotMode != mPrevRobotMode)
	{
		configRobotMode(mCurrRobotMode);
		if (mCurrRobotMode == ROBOT_POWER_MODE::FULL || mCurrRobotMode == ROBOT_POWER_MODE::MED)
		{
			mLEDOutput->Set(true);
		}
		else
		{
			mLEDOutput->Set(false);
		}
	}
	mPrevRobotMode = mCurrRobotMode;
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
	bool brakeMode = false;
	bool quickTurn = false;
    double x = 0;
    double y = 0;
    //Check mJoystick is valid and connected
    if (mJoystick && mJoystick->IsConnected())
    {
        x = ck::math::normalizeWithDeadband(mJoystick->GetRawAxis(4), DRIVE_JOYSTICK_DEADBAND);
        y = -ck::math::normalizeWithDeadband(mJoystick->GetRawAxis(1), DRIVE_JOYSTICK_DEADBAND);
		quickTurn = mJoystick->GetRawAxis(2) > 0.2;
		brakeMode = mJoystick->GetRawAxis(3) > 0.2;
    }

	switch(mCurrRobotMode)
	{
		case ROBOT_POWER_MODE::FULL:
		{
			break;
		}
		case ROBOT_POWER_MODE::MED:
		{
			x *= 0.6;
			y *= 0.6;
			break;
		}
		default:
		{
			x *= 0.35;
			y *= 0.35;
			break;
		}
	};

	DriveHelper::DriveMotorValues driveValues = mDriveHelper.calculateOutput(y, x, quickTurn, true);
	mLeftMaster->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, std::fmax(std::fmin(driveValues.left, 1.0), -1.0));
	mRightMaster->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, std::fmax(std::fmin(driveValues.right, 1.0), -1.0));

	if (brakeMode != mPrevBrakeMode)
	{
		NeutralMode bcMode = brakeMode ? NeutralMode::Brake : NeutralMode::Coast;
		for (TalonFX* tfx : mAllMotors)
		{
			ck::runTalonFunctionWithRetry([&]() { tfx->SetNeutralMode(bcMode); return tfx->GetLastError(); }, tfx->GetDeviceID());
		}
	}
	mPrevBrakeMode = brakeMode;

	// mLeftMaster->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, std::fmax(std::fmin(y + x, 1.0), -1.0));
	// mRightMaster->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, std::fmax(std::fmin(y - x, 1.0), -1.0));
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
