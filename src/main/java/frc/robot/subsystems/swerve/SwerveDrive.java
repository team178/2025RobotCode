package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    //! to be implementated later
    public static final Lock odometryLock = new ReentrantLock();
    private SysIdRoutine sysId;

    private static final boolean useOdometryForFieldRelative = false;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroIOInputs;
    private SDSSwerveModule[] modules;
    private Rotation2d rawGyroRotation;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

    private SwerveModulePosition[] modulePositions; // reference stored for delta tracking
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    private double personalSpeedFactor;
    private double elevatorSpeedFactor;

    //! Vision to be added

    public SwerveDrive(GyroIO gyroIO, SDSModuleIO FLModuleIO, SDSModuleIO FRModuleIO, SDSModuleIO BLModuleIO, SDSModuleIO BRModuleIO) {
        initComponents(gyroIO, FLModuleIO, FRModuleIO, BLModuleIO, BRModuleIO);
        initMathModels();
    }

    private void initComponents(GyroIO gyroIO, SDSModuleIO FLModuleIO, SDSModuleIO FRModuleIO, SDSModuleIO BLModuleIO, SDSModuleIO BRModuleIO) {
        this.gyroIO = gyroIO;
        modules = new SDSSwerveModule[] {
            new SDSSwerveModule("0 Front Left", FLModuleIO, 0),
            new SDSSwerveModule("1 Front Right", FRModuleIO, 1),
            new SDSSwerveModule("2 Back Left", BLModuleIO, 2),
            new SDSSwerveModule("3 Back Right", BRModuleIO, 3)
        };
        gyroIOInputs = new GyroIOInputsAutoLogged();
    }

    private void initMathModels() {
        personalSpeedFactor = 1;
        elevatorSpeedFactor = 1;

        rawGyroRotation = new Rotation2d();
        modulePositions = Arrays.stream(modules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2,  SwerveConstants.kWheelDistanceMeters / 2), // FL
            new Translation2d( SwerveConstants.kWheelDistanceMeters / 2,  SwerveConstants.kWheelDistanceMeters / 2), // FR
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2), // BL
            new Translation2d( SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)  // BR
        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, new Pose2d());
    }

    private double adjustAxisInput(double controllerInput, double deadband, double minThreshold, double steepness) {
        // see https://www.desmos.com/calculator/wj59z401tq
        return
            Math.abs(controllerInput) > deadband ?
            (
                Math.signum(controllerInput) *
                (1 - minThreshold) *
                Math.pow(
                    (Math.abs(controllerInput) - deadband) / (1 - deadband),
                    steepness) +
                minThreshold
            ) : 0;
    }

    /**
     * 
     * @param xInput
     * @param yInput
     * @param omegaInput
     * @param robotCentric
     * @param noOptimize
     * @return
     */
    public Command runDriveInputs(
        DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput,
        BooleanSupplier robotCentric, BooleanSupplier noOptimize) {
        return run(() -> {
            double deadband = 0.2; // minimum axis input before robot input
            double minThreshold = 0.1; // minimum robot input to overcome resistance
            double steepness = 1.8; // power to raise input (which is on interval [-1, 1], so reduces lower values)

            // apply drive filters (negatives based on last year)
            double vx = adjustAxisInput(xInput.getAsDouble(), deadband, minThreshold, steepness);
            double vy = adjustAxisInput(-yInput.getAsDouble(), deadband, minThreshold, steepness);
            double omega = adjustAxisInput(-omegaInput.getAsDouble(), deadband, minThreshold, steepness);

            adjustDriveSpeeds(vx, vy, omega, !robotCentric.getAsBoolean(), !noOptimize.getAsBoolean());
        });
    }

    public void adjustDriveSpeeds(double vx, double vy, double omega, boolean fieldRelative, boolean optimize) {
        double mag = Math.hypot(vx, vy);
        double dir = Math.atan2(vy, vx);

        // scale to max, then scale square onto circle
        mag *= SwerveConstants.kMagVelLimit * personalSpeedFactor * elevatorSpeedFactor;
        mag /= Math.min(
            Math.sqrt(1 + Math.pow(Math.sin(dir), 2)),
            Math.sqrt(1 + Math.pow(Math.cos(dir), 2))
        );

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(mag * Math.cos(dir), mag * Math.sin(dir), omega);
        runChassisSpeeds(chassisSpeeds, fieldRelative, optimize);
    }

    public void runChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean optimize) {
        ChassisSpeeds adjustedSpeeds = chassisSpeeds;
        if(fieldRelative) {
            adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                useOdometryForFieldRelative ?
                    getPose().getRotation().rotateBy(new Rotation2d(Constants.isRed() ? Math.PI : 0)) :
                    rawGyroRotation
            );
        }
        adjustedSpeeds = ChassisSpeeds.discretize(adjustedSpeeds, LoggedRobot.defaultPeriodSecs);
        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleSetpoints, SwerveConstants.kMaxWheelSpeed);

        Logger.recordOutput("Swerve/States/Setpoints", moduleSetpoints);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", adjustedSpeeds);

        setRawModuleSetpoints(moduleSetpoints, optimize);

        if(optimize) Logger.recordOutput("Swerve/States/SetpointsOptimized", moduleSetpoints);
    }

    public void setRawModuleSetpoints(SwerveModuleState[] states, boolean optimize) {
        for(int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i], optimize);
        }
    }

    public Command runTestDrive() {
        return runOnce(() -> {
            SwerveModuleState testSwerveState = new SwerveModuleState(Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
                new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
            );
            modules[0].setDesiredState(testSwerveState);
            modules[1].setDesiredState(testSwerveState);
            modules[2].setDesiredState(testSwerveState);
            modules[3].setDesiredState(testSwerveState);
            // for(SDSSwerveModule module : modules) {
            //     module.openLoop(
            //         Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn),
            //         Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive)
            //     );
            // }
        });
    }
    
    public Command runStopDrive() {
        return runOnce(() -> {
            for(SDSSwerveModule module : modules) {
                module.stopDrive();
            }
        });
    }

    public Command runUpdateControlConstants() {
        return runOnce(() -> {
            for(SDSSwerveModule module : modules) {
                module.updateControlConstants();
            }
        });
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroIOInputs);  
        Logger.processInputs("Gyro", gyroIOInputs);

        for(SDSSwerveModule module : modules) {
            module.periodic();
        }

        if(DriverStation.isDisabled()) {
            //! loop to stop drive shouldn't be necessary?? but to be tested (from template)
            for(SDSSwerveModule module : modules) {
                module.stopDrive();
            }
            Logger.recordOutput("Swerve/States/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Swerve/States/SetpointsOptimized", new SwerveModuleState[] {});
        }
        
        // if using higher-frequency odometry, loop begin here
        SwerveModulePosition[] updatedModulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            updatedModulePositions[i] = modules[i].getPosition();
            moduleDeltas[i] = new SwerveModulePosition(
                updatedModulePositions[i].distanceMeters - modulePositions[i].distanceMeters,
                updatedModulePositions[i].angle.minus(modulePositions[i].angle) //! template code did not subtract, so will need to test the Twist2d method of updating odometry rotation
            );
            modulePositions[i] = updatedModulePositions[i];
        }

        if(gyroIOInputs.connected) {
            rawGyroRotation = gyroIOInputs.yawPosition;
        } else {
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
        // loop end

        // use updateWithTime if using higher-frequency odometry with timestamps
        poseEstimator.update(rawGyroRotation, updatedModulePositions);
        gyroDisconnectedAlert.set(!gyroIOInputs.connected && Constants.currentMode != RobotMode.SIM);
    }
}
