package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.ControlConstants;

public class Constants {
    public static class SwerveConstants { // all swerve on CAN ID range 1-9
		public static final double kWheelDistanceMeters = Units.inchesToMeters(19.625); //! TO BE DETERMINED
		
        // F = Front, B = Back, L = Left, R = Right
		public static final int kFLDriveCANID = 1;
		public static final int kFLTurnCANID = 5;

		public static final int kFRDriveCANID = 2;
		public static final int kFRTurnCANID = 6;

		public static final int kBLDriveCANID = 4;
		public static final int kBLTurnCANID = 8;

		public static final int kBRDriveCANID = 3;
		public static final int kBRTurnCANID = 7;

		public static final int kPigeonCANID = 9; // this needs to be checked too

        //THE BELOW WILL NEED TO BE CHECKED

		public static final double kSRXMagEncoderCPR = 4096; // may be 1024 counts per revolution
		public static final double kTurnRelPositionConversionFactor = Units.rotationsToRadians(1 / kSRXMagEncoderCPR); // radians per count (radians per revolution * revolutions per count)

		public static final double kDriveGearRatio = 6.75 / 1; // rotations from input (motor) per rotations on output (wheel)
		public static final double kInternalNEOEncoderCPR = 42 / 1; // counts on encoder counts per revolution
		public static final double kWheelMetersPerRevolution = Units.inchesToMeters(Math.PI * 4); // meters per revolution (wheel circumference)
		public static final double kDriveConversionFactor = kWheelMetersPerRevolution / (kDriveGearRatio * kInternalNEOEncoderCPR); // meters per count
		public static final double kTurnAbsConversionFactor = Units.rotationsToRadians(1); // 2pi

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

		public static void initSwerveDrivePreferences() {
			Preferences.initDouble("kSwerveTestTurn", kDefaultTestTurn);
			Preferences.initDouble("kSwerveTestDrive", kDefaultTestDrive);
			System.out.println("Swerve preferences initialized");
		}

		public static void setSwerveDrivePreferences() {
			Preferences.setDouble("kSwerveTestTurn", kDefaultTestTurn);
			Preferences.setDouble("kSwerveTestDrive", kDefaultTestDrive);
		}
	}
    

	public static class SwerveModuleConstants {
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();

		public static final ControlConstants kTurnControlConstants = new ControlConstants(
			"swerveModule/turn",
			0, // 0.3
			0, // 0, used 0.0001 in the past
			0,
			0,
            0,
            0
		);

		public static final ControlConstants kDriveControlConstants = new ControlConstants(
			"swerveModule/drive",
			0, // 0.01
			0, // 0, used 0.0001 in the past
			0,
			0, // 0.145
            0,
            0
		);

		public static final double kTurnRatio = 12.8 / 1; // only use when using internal encoder

        public static void initMotorConfigs() {
            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12) // probably good voltage compensation
                .closedLoopRampRate(0) // set if needed
			; turnConfig.closedLoop
                .p(kTurnControlConstants.kP())
                .i(kTurnControlConstants.kI())
                .d(kTurnControlConstants.kD())
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-2, 2) // may change if necessary
			; turnConfig.absoluteEncoder
				.positionConversionFactor(SwerveConstants.kTurnAbsConversionFactor)
				.velocityConversionFactor(SwerveConstants.kTurnAbsConversionFactor)
			;

            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12) // probably good voltage compensation
                .closedLoopRampRate(0) // set if needed
            ; driveConfig.closedLoop
                .p(kDriveControlConstants.kP())
                .i(kDriveControlConstants.kI())
                .d(kDriveControlConstants.kD())
				.velocityFF(kDriveControlConstants.kV())
                .outputRange(-2, 2) // may change if necessary
            ; driveConfig.encoder
				.positionConversionFactor(SwerveConstants.kDriveConversionFactor)
				.velocityConversionFactor(SwerveConstants.kDriveConversionFactor)
			;
        }
	}
}
