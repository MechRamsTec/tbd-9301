// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveChassis;
import frc.robot.Robot;

public class Drive extends Command {

	CommandXboxController controller;
	SwerveChassis chassis;
	XboxController m_controller;
	Robot robot = new Robot();
	//XboxController control = new XboxController(0);

	int alliance = 1;

	SlewRateLimiter xLimiter = new SlewRateLimiter(20);
	SlewRateLimiter yLimiter = new SlewRateLimiter(20);
	SlewRateLimiter rotLimiter = new SlewRateLimiter(Units.rotationsToRadians(3));

	double rotMulti;
	double driveMulti;


		/** Creates a new Drive. */
		public Drive(SwerveChassis chassis, CommandXboxController controller) {
			// Use addRequirements() here to declare subsystem dependencies.
			this.chassis = chassis;
			this.controller = controller;
	
			addRequirements(chassis);
		}
	
		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				alliance = 1;
			} else {
				alliance = -1;
			}
		}

	
		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {

		if (controller.getHID().getLeftBumperButton() == true){
			driveMulti = .4;
			rotMulti = .2;
		} else{
			driveMulti = 1;
			rotMulti = 1;
		}

		double xInput = controller.getLeftY() * driveMulti;
		double yInput = controller.getLeftX() * driveMulti;
		double rotInput = controller.getRightX() * rotMulti;

		
		double xInputTrans = MathUtil.applyDeadband(xInput, 0.1) * Constants.SwerveModuleConstants.kMaxSpeed;
		double yInputTrans = MathUtil.applyDeadband(yInput, 0.1) * Constants.SwerveModuleConstants.kMaxSpeed ;
		double rotInputTrans = MathUtil.applyDeadband(rotInput, 0.1)
				* Constants.SwerveModuleConstants.KMaxAngularSpeed;

		// SmartDashboard.putNumber("Sticks/X", xInput);
		// SmartDashboard.putNumber("Sticks/Y", yInput);
		// SmartDashboard.putNumber("Sticks/R", rotInput);

		if (controller.getHID().getRightBumper()) {
			xInputTrans = xLimiter.calculate(xInputTrans);
			yInputTrans = yLimiter.calculate(yInputTrans);
			rotInputTrans = rotLimiter.calculate(rotInputTrans);

			chassis.driveRobotRelative(new ChassisSpeeds(xInputTrans, yInputTrans, rotInputTrans));
		} else {
			xInputTrans = xLimiter.calculate(xInputTrans * alliance);
			yInputTrans = yLimiter.calculate(yInputTrans * alliance);
			rotInputTrans = rotLimiter.calculate(rotInputTrans);

			chassis.driveFieldRelative(new ChassisSpeeds(xInputTrans, yInputTrans, rotInputTrans));
		}
		
	}

	// public Command multiplier(double Maxspeed){
	// 	return Commands.run(() -> Constants.SwerveModuleConstants.kMaxSpeed = Maxspeed, this);
	// }

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
