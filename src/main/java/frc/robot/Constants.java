package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final class SwerveModuleConstants {

		// Max speed of the drive motor
		public static final double kMaxSpeed = 4.5; // meters per second 3.7
		public static final double kMaxAcceleration = 10.0; // meters per second squared
		public static final double KMaxAngularSpeed = 2;

		// Ratio drive motor and turn motor
		public static final double kDriveGearRatio = 6.75; //8.14
		public static final double kTurningGearRatio = 150 / 7;

		// Wheel diameter
		public static final double kWheelDiameter = Units.inchesToMeters(4.0);

		// Physical conversion constants
		public static final double kRotationToMeter = Math.PI * kWheelDiameter / kDriveGearRatio;
		public static final double kRPMToMeterPerSecond = kRotationToMeter / 60.0;

		public static final double kRotationToDegree = 360.0 / kTurningGearRatio;
		public static final double kRPMToDegreePerSecond = kRotationToDegree / 60.0;

	}

	public static final class SwerveChassisConstants {

		public static final double kTrackWidth = Units.inchesToMeters(20.15);
		public static final double kWheelBase = Units.inchesToMeters(20.15);

		// Kinematics
		public static final Translation2d[] kModuleLocation = {
				new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // Front left
				new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // Front right
				new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) };
	}

	public static final class LiftingArmConstants{
		public static final double kLiftingArmGearRatio = 125;
		public static final double kRotationToDegree = 360.0 / kLiftingArmGearRatio;
		public static final double kRPMToDegreePerSecond = kRotationToDegree / 60.0;

		public static final double ExtendVolts = 12;
		public static final double StopVolts = 0;
		public static final double RetractVolts = -12;

		public static final double OpenAngle = 150;
		public static final double ClosedAngle = 0;
	}

	public static final class IntakeConstants{
		public static final double GrabVolts = 4;
		public static final double TrowVolts = -10;
		public static final double StopVolts = 0;
	}

	public static final class ShooterConstants{
		public static final double ShootVelocity = 5300;
		public static final double StopVelocity = 0;
	}
}
