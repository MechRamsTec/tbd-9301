// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule extends SubsystemBase {
	// Motores
	private SparkMax driveMotor;
	private SparkMax turnMotor;

	// Sensores
	private CANcoder absoluteEncoder;
	private RelativeEncoder turnEncoder;
	private RelativeEncoder driveEncoder;
	private PIDController controller;

	private SimpleMotorFeedforward driverFeed;

	// Module Name
	private String name;

	// Module State
	private SwerveModuleState currentState = new SwerveModuleState();

	/** Creates a new SwerveModule. */
	public SwerveModule(int driveMotorID, int turnMotorID, int absoluteEncoderID, double offSet, String name,
			String canBus) {

		this.name = name;
		this.driveMotor = new SparkMax(driveMotorID, SparkMax.MotorType.kBrushless);
		this.turnMotor = new SparkMax(turnMotorID, SparkMax.MotorType.kBrushless);
		this.absoluteEncoder = new CANcoder(absoluteEncoderID, canBus);
		// driverFeed = new SimpleMotorFeedforward(0.1,0.1 , 0.1);

		this.controller = new PIDController(0.065, 0, 0);  
		// Enconder configuration
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
		encoderConfig.MagnetSensor.MagnetOffset = offSet;

		// Configure Relative Encoder
		this.turnEncoder = turnMotor.getEncoder();
		this.driveEncoder = driveMotor.getEncoder();

		// controller.enableContinuousInput(-180, 180); REVISAR

		// Set encoder configuration

		// Set motor configs
		SparkMaxConfig turnConfig = new SparkMaxConfig();
		turnConfig.inverted(false);
		turnConfig.smartCurrentLimit(40);
		turnConfig.closedLoopRampRate(0.1);
		turnConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
		turnConfig.inverted(true);
		turnConfig.encoder.positionConversionFactor(Constants.SwerveModuleConstants.kRotationToDegree);
		turnConfig.encoder.velocityConversionFactor(Constants.SwerveModuleConstants.kRPMToDegreePerSecond);

		//REVISAR
		turnConfig.closedLoop
			.positionWrappingEnabled(true)
			.positionWrappingMaxInput(2 * Math.PI)
			.positionWrappingMinInput(0);
			
		turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, null);


			
		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig
			.smartCurrentLimit(40)
			.openLoopRampRate(0.1)
			.idleMode(SparkBaseConfig.IdleMode.kBrake);
		driveConfig.encoder
			.positionConversionFactor(Constants.SwerveModuleConstants.kRotationToMeter)
			.velocityConversionFactor(Constants.SwerveModuleConstants.kRPMToMeterPerSecond);
		driveConfig.closedLoop
			.pid(0.4, 0.001, 0.01);

			//REVISAR
		driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, null);

		resetTurnEncoder();
		driveEncoder.setPosition(0.0);
	}

	// Reset the turn encoder to match the absolute encoder
	public void resetTurnEncoder() {
		// Get the absolute encoder position in degrees
		double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
		// Set the turn encoder position to the absolute encoder position
		turnEncoder.setPosition(absolutePosition);
	}

	// Get the speed of the drive motor in meters per second
	public double getDriveSpeed() {
		return driveEncoder.getVelocity();
	}

	// Get the angle of the turn motor in degrees
	public double getTurnAngle() {
		return turnEncoder.getPosition();
	}

	// Set the position in meters from the drive motor
	public double getDrivePosition() {
		return driveEncoder.getPosition();
	}

	// Set the velocity in meters per second to the drive motor
	public void setDriveSpeed(double speed) {
		driveMotor.getClosedLoopController().setReference(speed, SparkMax.ControlType.kDutyCycle);
	}

	// Get the actual module state
	public SwerveModuleState getActualState() {
		return new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getTurnAngle()));
	}

	// get the module position
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurnAngle()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		currentState = SwerveModuleState.optimize(desiredState, currentState.angle);

		// Set the drive motor speed
		setDriveSpeed(currentState.speedMetersPerSecond);

		// Set the turn motor angle

		// turnMotor.getPIDController().setReference(currentState.angle.getDegrees(),
		// ControlType.kPosition);

		turnMotor.setVoltage(controller.calculate(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360,
				currentState.angle.getDegrees()));
	}

	// Get the module position

	@Override
	public void periodic() {
		SmartDashboard.putNumber(name + "/Speed", getDriveSpeed());
		SmartDashboard.putNumber(name + "/Target Speed", currentState.speedMetersPerSecond);
		// SmartDashboard.putNumber(name + "/Position", getDrivePosition());
		// SmartDashboard.putNumber(name + "/Angle", getTurnAngle());
		SmartDashboard.putNumber(name + "/Absolute Angle",
				absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360);
		SmartDashboard.putNumber(name + "/Target Angle", currentState.angle.getDegrees());

		// This method will be called once per scheduler run
	}
}
