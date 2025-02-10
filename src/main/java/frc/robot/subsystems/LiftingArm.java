// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftingArm extends SubsystemBase {
  private final CANSparkMax liftingMotor;
  private final RelativeEncoder liftingMotorEncoder;

  // private double targetAngle = 0.0;

  public LiftingArm() {
    liftingMotor = new CANSparkMax(11, MotorType.kBrushless);

    liftingMotorEncoder = liftingMotor.getEncoder();
    liftingMotorEncoder.setPosition(0);
    liftingMotor.setInverted(true);
    liftingMotor.getPIDController().setP(0.02);
    liftingMotor.getPIDController().setI(0.0);
    liftingMotor.getPIDController().setD(0.0);

    liftingMotorEncoder.setPositionConversionFactor(Constants.LiftingArmConstants.kRotationToDegree);
		liftingMotorEncoder.setVelocityConversionFactor(Constants.LiftingArmConstants.kRPMToDegreePerSecond);

    liftingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    liftingMotor.setSmartCurrentLimit(40);
  }

  public Command setVoltage(double volts){
    return Commands.runOnce(() -> liftingMotor.setVoltage(volts), this);
  }

  public Command setAngle(double angle){
    return Commands.run(() -> liftingMotor.getPIDController().setReference(angle, ControlType.kPosition)).until(()-> Math.abs(liftingMotorEncoder.getPosition() - angle) < 2 );
  }

  // public void setTargetAngle(double targetAngle){
  //   this.targetAngle = targetAngle;
  // }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/LiftingArm", liftingMotorEncoder.getPosition());
    // SmartDashboard.putBoolean("Arm/isatsetpoint", liftingMotor.getPIDController());
    // SmartDashboard.putNumber("Arm/goal", liftingMotorPID.getGoal().position);
    // SmartDashboard.putNumber("Arm/setpoin", liftingMotorPID.getSetpoint().position);

  }
}
*/