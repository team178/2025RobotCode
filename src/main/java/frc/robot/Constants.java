package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

	public static enum FieldType {
		ANDYMARK, WELDED
	}

	public static final SendableChooser<FieldType> kFieldType = new SendableChooser<>();

	static {
		kFieldType.setDefaultOption("AndyMark Field", FieldType.ANDYMARK);
		kFieldType.addOption("Welded Field", FieldType.WELDED);

		Shuffleboard.getTab("Teleoperated")
			.add("Field Type", kFieldType)
			.withWidget(BuiltInWidgets.kComboBoxChooser)
			.withSize(2, 1)
			.withPosition(0, 0);
	}

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

		public static final double kDriveGearRatio = 6.12 / 1; // rotor rotations per wheel rotations
		public static final double kInternalNEOEncoderCPR = 42 / 1; // counts on encoder counts per revolution
		public static final double kWheelRadiusMeters = Units.inchesToMeters(4 / 2); // meters per revolution (wheel circumference)
		// public static final double kDrivePositionConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio * kInternalNEOEncoderCPR); // wheel rad per rotor count
		public static final double kDrivePositionConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio); // wheel rad per rotor count
		public static final double kDriveVelocityConversionFactor = Units.rotationsToRadians(1) / (kDriveGearRatio * 60); // wheel rad per second  per  rotor revolutions per minute
		public static final double kTurnPositionConversionFactor = Units.rotationsToRadians(1); // rotations -> radians
		public static final double kTurnVelocityConversionFactor = Units.rotationsToRadians(1); // rotations per second -> radians per second (not minutes?)

		public static final double kMaxWheelSpeed = 20; // m/s
		public static final double kMagVelLimit = 5; // m/s (5 seems to be physical limit)
		public static final double kRotVelLimit = 18; // rad/s
		// public static final double kDirVelLimit = 10; // rad/s
		// public static final double kMagAccelLimit = 48; // m/s^2
		// public static final double kRotAccelLimit = 30; // rad/s^2

		public static final double kSlowedMult = 0.12;

		public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics( //! make sure these are the right order, FRONT left right, BACK left right
			new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2),
			new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, SwerveConstants.kWheelDistanceMeters / 2), // I REALLY DONT KNOW ANYMORE
			new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2),
			new Translation2d(SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)
		);

		public static final ControlConstants kPresetRotControlConstants = new ControlConstants(
			"SwervePresetRot",
			10,
			0,
			1,
			0,
			0,
			0
		);

		public static final ControlConstants kPresetPosControlConstants = new ControlConstants(
			"SwervePresetRot",
			1.5,
			0,
			0,
			0,
			0,
			0
		);

		static {
			// Preferences.initDouble("kSwerveTestTurn", 0);
			// Preferences.initDouble("kSwerveTestDrive", 0);
			Preferences.initDouble("odometry/setPoseX", 0);
			Preferences.initDouble("odometry/setPoseY", 0);
			Preferences.initDouble("odometry/setPoseRot", 0);
			System.out.println("SwerveConstants initialized");
		}
	}
    
	public static class SwerveModuleConstants {
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

		public static final Rotation2d FLZeroRotation = new Rotation2d(4.373);
		public static final Rotation2d FRZeroRotation = new Rotation2d(5.662);
		public static final Rotation2d BLZeroRotation = new Rotation2d(3.623);
		public static final Rotation2d BRZeroRotation = new Rotation2d(4.553);

		public static final ControlConstants turnControlConstants = new ControlConstants(
			"swerveModule/turn",
			0.6, // 0.3
			0, // 0, used 0.0001 in the past
			0,
			0,
            0,
            0
		);

		public static final ControlConstants driveControlConstants = new ControlConstants(
			"swerveModule/drive",
			0.0001, // 0.01 -> 0.00009
			0, // 0, used 0.0001 in the past
			0,
			0.01, // 0.145
            0,
            0.11 // 0.11
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
				.closedLoopRampRate(0.01)
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
		public static final int kElevatorLeaderCANID = 19;
		public static final int kElevatorFollowerCANID = 12;
		public static final int kLeftEffectorCANID = 13;
		public static final int kRightEffectorCANID = 14;
		public static final int kFunnelMotorCANID = 15;
		public static final int kDealgaeMotorCANID = 16;

		public static final int kHighLimitDIO = 1;
		public static final int kLowLimitDIO = 9;
		public static final int kUpperPhotosensorDIO = 0;
		public static final int kLowerPhotosensorDIO = 2;

		public static final double kSprocketPitchDiameter = Units.inchesToMeters(1.7567); // meters
		public static final double kElevatorPositionConversionFactor = kSprocketPitchDiameter * Math.PI;
		public static final double kElevatorPositionConversionFactorInternal = kSprocketPitchDiameter * Math.PI / 20;

		public static final ControlConstants elevatorControlConstants = new ControlConstants(
			"elevator",
			20, // to test
			0,
			1,
			0,
			0.22,
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
				.inverted(true)
				.closedLoopRampRate(0.1)
			; elevatorLeaderConfig.closedLoop
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
				.outputRange(-1, 0.03, ClosedLoopSlot.kSlot2) // high limit
			// ; elevatorLeaderConfig.absoluteEncoder
			// 	.positionConversionFactor(kElevatorPositionConversionFactor)
			// 	.velocityConversionFactor(kElevatorPositionConversionFactor)
			// 	.inverted(true)
			; elevatorLeaderConfig.encoder
				.positionConversionFactor(kElevatorPositionConversionFactorInternal)
				.velocityConversionFactor(kElevatorPositionConversionFactorInternal)
			;

			elevatorFollowerConfig
				.follow(kElevatorLeaderCANID)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
				//! may need to invert this too
			;

            effectorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
            ; effectorConfig.encoder
				.positionConversionFactor(1) // revolutions
				.velocityConversionFactor(1) // RPM
			;
		}
	}

	public static class ManipulatorConstants { // CAN ID range 21-24
		public static final int kDeployMotorCANID = 21;
		public static final int kRollerMotorCANID = 22;

		public static final int kPhotosensorDIO = 5;

		public static final double kDeployEncoderOffset = 0.0; // TODO test
		
		// TODO fill in
		public static final double kDeployRadiusToCenterOfMass = 0.0; // meters
		public static final double kDeployMass = 0.0; // kilograms
		public static final double kGravity = 9.81; // meters per second
		public static final double kVoltagePerTorque = 0.0; // volts / Newton meter
		
		public static final double kNEOKv = 473;
		
		// The torque should be equal to the
		// radius(l to center of mass) * the mass of the arm * gravity(9.8) * the cosine of the angle
		// Therefore, the G constant should be r*m*a, then account for how many volts it needs to have the right torque
		public static final ControlConstants deployControlConstants = new ControlConstants(
			"manipulator",
			0.0001, // TODO to test
			0,
			0,
			0,
			kDeployRadiusToCenterOfMass * kDeployMass * kGravity * kVoltagePerTorque,
			0
		);

		public static final SparkMaxConfig deployConfig = new SparkMaxConfig();
		public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();

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
				.zeroOffset(kDeployEncoderOffset)
			;

			rollerConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12)
			; rollerConfig.encoder
				.positionConversionFactor(1) // revolutions
				.velocityConversionFactor(1) // RPM
			;
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

	// TODO adjust
	public static class VisionConstants {
		public static final double linearStdDevBaseline = 8;
		public static final double angularStdDevBaseline = Double.POSITIVE_INFINITY;

		public static final double linearStdDevMT2Factor = 0.1;
		public static final double angularStdDevMT2Factor = Double.POSITIVE_INFINITY;

	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kAuxControllerPort = 1;
	}

	public static class FieldConstants {
		public static final double fieldWidth = 17.548; // m
		public static final double fieldHeight = 8.042; // m - ANDYMARK FIELD NOT WELDED BC FIRST IS ANNOYING :) 8.052

		public static final Translation2d reefCenterBlue = new Translation2d(4.489323, fieldHeight / 2);
		public static final Translation2d reefCenterRed = new Translation2d(13.058902, fieldHeight / 2);
		public static final Translation2d fieldCenter = new Translation2d(fieldWidth / 2, fieldHeight / 2);
		public static final Transform2d betweenReefsTransform = new Transform2d(reefCenterRed.minus(reefCenterBlue), Rotation2d.kZero);

		public static final Pose2d blueCloseLeftReef = new Pose2d(3.2512, fieldHeight / 2 + 0.164338, Rotation2d.kZero);
	    public static final Pose2d blueCloseRightReef = new Pose2d(3.2512, fieldHeight / 2 - 0.164338, Rotation2d.kZero);
		public static final Pose2d redFarLeftReef = blueCloseLeftReef.plus(betweenReefsTransform);
		public static final Pose2d redFarRightReef = blueCloseRightReef.plus(betweenReefsTransform);

		public static final Pose2d blueCoralY = new Pose2d(1.59336668626, 7.44378938214, Rotation2d.fromDegrees(306));
		public static final Pose2d blueCoralZ = new Pose2d(1.59336668626, fieldHeight - 7.44378938214, Rotation2d.fromDegrees(54));
		public static final Pose2d redCoralY = blueCoralY.rotateAround(fieldCenter, Rotation2d.k180deg);
		public static final Pose2d redCoralZ = blueCoralZ.rotateAround(fieldCenter, Rotation2d.k180deg);
	}
}
