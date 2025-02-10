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

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterRightMotor;
  private final CANSparkMax shooterLeftMotor;
  private final RelativeEncoder shooterRightEncoder;
  private double velocity = 0.0;

  public Shooter() {
    shooterRightMotor = new CANSparkMax(9, MotorType.kBrushless);
    shooterLeftMotor = new CANSparkMax(10, MotorType.kBrushless);

    shooterRightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterLeftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
    shooterRightEncoder = shooterRightMotor.getEncoder();

    shooterRightMotor.setSmartCurrentLimit(30);
    shooterLeftMotor.setSmartCurrentLimit(30);

    shooterLeftMotor.follow(shooterRightMotor, true);

    shooterRightMotor.getPIDController().setP(0);
    shooterRightMotor.getPIDController().setI(0);
    shooterRightMotor.getPIDController().setD(0);
    shooterRightMotor.getPIDController().setFF(0.0002);

    shooterRightMotor.setClosedLoopRampRate(.8);
    shooterLeftMotor.setClosedLoopRampRate(.8);

  }

  public Command setVelocity(double velocity){
    return Commands.run(() -> 
    shooterRightMotor.getPIDController().setReference(velocity, ControlType.kVelocity)).until(()-> 
    Math.abs(shooterRightEncoder.getVelocity() - velocity) < 100 ).beforeStarting(()-> this.velocity = velocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/current", shooterRightEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/target", velocity);
  }
}
*/