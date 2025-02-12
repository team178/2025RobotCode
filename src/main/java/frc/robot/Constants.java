package frc.robot;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.ControlConstants;

public class Constants {
	public static enum RobotMode {
		/** Running on a real robot. */
		REAL,
	
		/** Running a physics simulator. */
		SIM,
	
		/** Replaying from a log file. */
		REPLAY;
	}
	public static final RobotMode simMode = RobotMode.SIM;
	public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;

	/** 
	 * Usually red means some calculations are flipped
	 */
	public static boolean isRed() {
		return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
	}
	
    public static class SwerveConstants { // all swerve on CAN ID range 1-9
		public static final double kWheelDistanceMeters = Units.inchesToMeters(20);
		
        // F = Front, B = Back, L = Left, R = Right
		public static final int kFLDriveCANID = 1;
		public static final int kFLTurnCANID = 5;

		public static final int kFRDriveCANID = 2;
		public static final int kFRTurnCANID = 6;

		public static final int kBLDriveCANID = 3;
		public static final int kBLTurnCANID = 7;

		public static final int kBRDriveCANID = 4;
		public static final int kBRTurnCANID = 8;

		public static final int kPigeonCANID = 9; // this needs to be checked too

        //THE BELOW WILL NEED TO BE CHECKED

		public static final double kSRXMagEncoderCPR = 4096; // may be 1024 counts per revolution
		public static final double kTurnRelPositionConversionFactor = Units.rotationsToRadians(1 / kSRXMagEncoderCPR); // radians per count (radians per revolution * revolutions per count)

		public static final double kDriveGearRatio = 6.12 / 1; // rotor rotations per wheel rotations
		public static final double kInternalNEOEncoderCPR = 42 / 1; // counts on encoder counts per revolution
		public static final double kWheelRadiusMeters = Units.inchesToMeters(4 / 2); // meters per revolution (wheel circumference)
		public static final double kDrivePositionConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio * kInternalNEOEncoderCPR); // wheel rad per rotor count
		public static final double kDriveVelocityConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio * kInternalNEOEncoderCPR); // wheel rad per second  per  rotor count per second
		public static final double kTurnPositionConversionFactor = Units.rotationsToRadians(1); // rotations -> radians
		public static final double kTurnVelocityConversionFactor = Units.rotationsToRadians(1); // rotations per second -> radians per second (not minutes?)

		public static final double kMaxWheelSpeed = 8; // m/s
		public static final double kMagVelLimit = 2.5; // m/s
		public static final double kDirVelLimit = 10; // rad/s
		public static final double kRotVelLimit = 6; // rad/s
		public static final double kMagAccelLimit = 48; // m/s^2
		public static final double kRotAccelLimit = 30; // rad/s^2

		public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
			new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
			new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2), // I REALLY DONT KNOW ANYMORE
			new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
			new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)
		);
		
		public static final double kDefaultTestTurn = 0;
		public static final double kDefaultTestDrive = 0;

		static {
			Preferences.initDouble("kSwerveTestTurn", kDefaultTestTurn);
			Preferences.initDouble("kSwerveTestDrive", kDefaultTestDrive);
			System.out.println("SwerveConstants initialized");
		}
	}
    
	public static class SwerveModuleConstants {
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

		public static final Rotation2d FLZeroRotation = new Rotation2d();
		public static final Rotation2d FRZeroRotation = new Rotation2d();
		public static final Rotation2d BLZeroRotation = new Rotation2d();
		public static final Rotation2d BRZeroRotation = new Rotation2d();

		public static final ControlConstants turnControlConstants = new ControlConstants(
			"swerveModule/turn",
			0.3, // 0.3
			0, // 0, used 0.0001 in the past
			0,
			0,
            0,
            0
		);

		public static final ControlConstants driveControlConstants = new ControlConstants(
			"swerveModule/drive",
			0.00009, // 0.01
			0, // 0, used 0.0001 in the past
			0,
			0.0069, // 0.145
            0,
            0.11
		);

        static {
            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
			; turnConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(turnControlConstants.kP())
                .i(turnControlConstants.kI())
                .d(turnControlConstants.kD())
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1)
			; turnConfig.absoluteEncoder
				.positionConversionFactor(SwerveConstants.kTurnPositionConversionFactor)
				.velocityConversionFactor(SwerveConstants.kTurnVelocityConversionFactor)
			;

            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
            ; driveConfig.closedLoop
                .p(driveControlConstants.kP())
                .i(driveControlConstants.kI())
                .d(driveControlConstants.kD())
				.velocityFF(driveControlConstants.kV())
                .outputRange(-1, 1)
            ; driveConfig.encoder
				.positionConversionFactor(SwerveConstants.kDrivePositionConversionFactor)
				.velocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor)
			;
			System.out.println("SwerveModuleConstants initialized");
        }
	}

	public static class ElevatorConstants { // CAN ID range 11-19
		public static final int kElevatorLeaderCANID = 11;
		public static final int kElevatorFollowerCANID = 12;
		public static final int kLeftEffectorCANID = 13;
		public static final int kRightEffectorCANID = 14;
		public static final int kFunnelMotorCANID = 15;

		public static final int kHighLimitDIO = 1;
		public static final int kLowLimitDIO = 2;
		public static final int kUpperPhotosensorDIO = 3;
		public static final int kLowerPhotosensorDIO = 4;

		public static final double kSprocketPitchDiameter = Units.inchesToMeters(1.7567); // meters
		public static final double kElevatorPositionConversionFactor = kSprocketPitchDiameter * Math.PI;

		public static final double kNEOCPR = 4096; // might be 1024, test
		public static final double kEffectorPositionConversionFactor = kNEOCPR; // counts to revolutions

		public static final ControlConstants elevatorControlConstants = new ControlConstants(
			"elevator",
			0.0001, // to test
			0,
			0,
			0,
			0,
			0
		);
		
		public static final SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
		public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
		public static final SparkMaxConfig effectorConfig = new SparkMaxConfig(); // also used for funnel motor, since is the same
		
		static {
			elevatorLeaderConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
			; elevatorLeaderConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.p(elevatorControlConstants.kP(), ClosedLoopSlot.kSlot0)
				.i(elevatorControlConstants.kI(), ClosedLoopSlot.kSlot0)
				.d(elevatorControlConstants.kD(), ClosedLoopSlot.kSlot0)
				.p(elevatorControlConstants.kP(), ClosedLoopSlot.kSlot1)
				.i(elevatorControlConstants.kI(), ClosedLoopSlot.kSlot1)
				.d(elevatorControlConstants.kD(), ClosedLoopSlot.kSlot1)
				.p(elevatorControlConstants.kP(), ClosedLoopSlot.kSlot2)
				.i(elevatorControlConstants.kI(), ClosedLoopSlot.kSlot2)
				.d(elevatorControlConstants.kD(), ClosedLoopSlot.kSlot2)
				.outputRange(-1, 1, ClosedLoopSlot.kSlot0) // no limit
				.outputRange(0, 1, ClosedLoopSlot.kSlot1) // low limit
				.outputRange(-1, 0.1, ClosedLoopSlot.kSlot2) // high limit
			; elevatorLeaderConfig.absoluteEncoder
				.positionConversionFactor(kElevatorPositionConversionFactor)
				.velocityConversionFactor(kElevatorPositionConversionFactor)
			;

			elevatorFollowerConfig
				.follow(kElevatorLeaderCANID)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
			;

            effectorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
            ; effectorConfig.encoder
				.positionConversionFactor(kEffectorPositionConversionFactor)
				.velocityConversionFactor(kEffectorPositionConversionFactor)
			;
		}
	}

	public static class ManipulatorConstants { // CAN ID range 21-24
		public static final int kDeployMotorCANID = 21;
		public static final int kRollerMotorCANID = 22;

		public static final int kPhotosensorDIO = 5;

		public static final double kNEOKv = 473;
		public static final double kNEOCPR = 4096; // might be 1024, test
		public static final double kPositionConversionFactor = kNEOCPR; // counts to revolutions

		public static final double kDeployEncoderOffset = 0.0; // TODO test

		public static final ControlConstants deployControlConstants = new ControlConstants(
			"manipulator",
			0.0001, // to test
			0,
			0,
			0,
			0,
			0
		);

		public static final SparkMaxConfig deployConfig = new SparkMaxConfig();
		public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();

		public static final AbsoluteEncoderConfig deployEncoderConfig = new AbsoluteEncoderConfig();

		static {
			deployConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
			; deployConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.p(deployControlConstants.kP())
				.i(deployControlConstants.kI())
				.d(deployControlConstants.kD())
				.outputRange(-1, 1)
			; deployConfig.absoluteEncoder
				.positionConversionFactor(Units.rotationsToDegrees(1))
				.velocityConversionFactor(Units.rotationsToDegrees(1))
			;

			rollerConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
			; rollerConfig.encoder
				.positionConversionFactor(kPositionConversionFactor)
				.velocityConversionFactor(kPositionConversionFactor)
			;

			deployEncoderConfig.zeroOffset(kDeployEncoderOffset);
		}
	}

	public static class ClimberConstants { // CAN ID range 25-29
		public static final int kClimberMotorCANID = 25;

		public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

		static {
			climberConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
			;
		}
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kAuxControllerPort = 1;
	}
}
