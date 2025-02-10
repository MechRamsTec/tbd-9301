package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private final SparkMax swingerMotor;
    private final RelativeEncoder swingerMotorEncoder;
    private final SparkClosedLoopController swingerPID;
    private final SparkMaxConfig swingerConfig;
    
    public Climber() {
        swingerMotor = new SparkMax(10, MotorType.kBrushless);
        swingerMotorEncoder = swingerMotor.getEncoder();
        swingerPID = swingerMotor.getClosedLoopController();
        swingerConfig = new SparkMaxConfig();
        
        swingerConfig
			.smartCurrentLimit(40)
			.openLoopRampRate(0.2)
			.idleMode(SparkBaseConfig.IdleMode.kBrake);
        swingerConfig.closedLoop
            .p(0.02)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
        
        swingerConfig.encoder
            .velocityConversionFactor(Constants.LiftingArmConstants.kRPMToDegreePerSecond)
            .positionConversionFactor(Constants.LiftingArmConstants.kRotationToDegree);

        swingerMotor.configure(swingerConfig, ResetMode.kResetSafeParameters, null);

        swingerMotorEncoder.setPosition(0);
    }

    public Command setVoltage(double volts){
        return Commands.runOnce(() -> swingerMotor.setVoltage(volts), this);
    }

    public Command setAngle(double angle){
        return Commands.run(() -> swingerPID.setReference(angle, ControlType.kPosition)).until(()-> Math.abs(swingerMotorEncoder.getPosition() - angle) < 2 );
    }

  // public void setTargetAngle(double targetAngle){
  //   this.targetAngle = targetAngle;
  // } 

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm/LiftingArm", swingerMotorEncoder.getPosition());
        // SmartDashboard.putBoolean("Arm/isatsetpoint", liftingMotor.getPIDController());
        // SmartDashboard.putNumber("Arm/goal", liftingMotorPID.getGoal().position);
        // SmartDashboard.putNumber("Arm/setpoin", liftingMotorPID.getSetpoint().position);
    }
}

