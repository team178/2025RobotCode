package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    //! to be implementated later
    public static final Lock odometryLock = new ReentrantLock();
    private SysIdRoutine sysId;

    private static final boolean useOdometryForFieldRelative = true;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroIOInputs;
    private SDSSwerveModule[] modules;
    private Rotation2d rawGyroRotation;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

    private SwerveModulePosition[] modulePositions; // reference stored for delta tracking
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    private PIDController trajVXController;
    private PIDController trajVYController;
    private PIDController trajHeadingController;

    private double elevatorSpeedFactor;
    private boolean toX;

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
        elevatorSpeedFactor = 1;
        toX = true;

        rawGyroRotation = new Rotation2d();
        modulePositions = Arrays.stream(modules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
        kinematics = new SwerveDriveKinematics( // NWU coordinate system
            new Translation2d( SwerveConstants.kWheelDistanceMeters / 2,  SwerveConstants.kWheelDistanceMeters / 2), // FL
            new Translation2d( SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2), // FR
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2,  SwerveConstants.kWheelDistanceMeters / 2), // BL
            new Translation2d(-SwerveConstants.kWheelDistanceMeters / 2, -SwerveConstants.kWheelDistanceMeters / 2)  // BR
        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, (Constants.isRed() ? new Pose2d(17.548, 8.052, Rotation2d.kPi) : new Pose2d()));

        trajVXController = new PIDController(10, 0, 0);
        trajVYController = new PIDController(10, 0, 0);
        trajHeadingController = new PIDController(7.5, 0, 0);
    }

    private double adjustAxisInput(double controllerInput, double deadband, double minThreshold, double steepness) {
        // see https://www.desmos.com/calculator/wj59z401tq
        return
            Math.abs(controllerInput) > deadband ?
            MathUtil.clamp(
                Math.signum(controllerInput) *
                ((1 - minThreshold) *
                Math.pow(
                    (Math.abs(controllerInput) - deadband) / (1 - deadband),
                    steepness) +
                minThreshold)
            , -1, 1) : 0;
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
        DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput, DoubleSupplier speedFactorInput,
        BooleanSupplier robotCentric, BooleanSupplier noOptimize) {
        return run(() -> {
            double xInputValue = xInput.getAsDouble();
            double yInputValue = yInput.getAsDouble();
            double omegaInputValue = omegaInput.getAsDouble();
            yInputValue *= -1; // y-axis is inverted on joystick
            omegaInputValue *= -1; // rotation is reversed

            // double deadband = 0.2; // minimum axis input before robot input
            // double minThreshold = 0.05; // minimum robot input to overcome resistance
            // double steepness = 1.8; // power to raise input (which is on interval [-1, 1], so reduces lower values)

            // rotate axes to NWU coordinate system
            // double vx = adjustAxisInput(yInputValue, deadband, minThreshold, steepness);
            // double vy = adjustAxisInput(-xInputValue, deadband, minThreshold, steepness);
            // double omega = adjustAxisInput(omegaInput.getAsDouble(), deadband, minThreshold, steepness);
            double speedFactor = 1 - (1 - SwerveConstants.kSlowedMult) * speedFactorInput.getAsDouble();

            adjustDriveSpeeds(yInputValue, -xInputValue, omegaInputValue, speedFactor, !robotCentric.getAsBoolean(), !noOptimize.getAsBoolean());
        });
    }

    public void adjustDriveSpeeds(double vx, double vy, double omega, double speedFactor, boolean fieldRelative, boolean optimize) {
        double mag = Math.hypot(vx, vy);
        double dir = Math.atan2(vy, vx);

        double deadband = 0.2; // minimum axis input before robot input
        double minThreshold = 0.03; // minimum robot input to overcome resistance
        double steepness = 1.8; // power to raise input (which is on interval [-1, 1], so reduces lower values)
        mag = adjustAxisInput(mag, deadband, minThreshold, steepness);
        mag *= SwerveConstants.kMagVelLimit * speedFactor * elevatorSpeedFactor;
        omega = adjustAxisInput(omega, deadband, minThreshold, steepness);
        omega *= SwerveConstants.kRotVelLimit;

        if(toX && mag == 0 && omega == 0) {
            toXPosition(optimize);
            return;
        }
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

    public void toXPosition(boolean optimize) {
        setRawModuleSetpoints(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        }, optimize);
    }

    public Command runToXPosition(boolean optimize) {
        return runOnce(() -> {
            toX = !toX;
        });
    }

    public void followSwerveSample(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + trajVXController.calculate(pose.getX(), sample.x),
            sample.vy + trajVYController.calculate(pose.getY(), sample.y),
            sample.omega + trajHeadingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        runChassisSpeeds(speeds, true, true);
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose);
    }

    @AutoLogOutput(key = "Swerve/toX")
    public boolean isToX() {
        return toX;
    }

    // public Command runTestDrive() {
    //     return runOnce(() -> {
    //         SwerveModuleState testSwerveState = new SwerveModuleState(Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
    //             new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
    //         );
    //         modules[0].setDesiredState(testSwerveState);
    //         modules[1].setDesiredState(testSwerveState);
    //         modules[2].setDesiredState(testSwerveState);
    //         modules[3].setDesiredState(testSwerveState);
    //         // for(SDSSwerveModule module : modules) {
    //         //     module.openLoop(
    //         //         Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn),
    //         //         Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive)
    //         //     );
    //         // }
    //     });
    // }

    // public Command runOpenTestDrive() {
    //     return runOnce(() -> {
    //         SwerveModuleState testSwerveState = new SwerveModuleState(Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive),
    //             new Rotation2d(Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn))
    //         );
    //         double drive = testSwerveState.speedMetersPerSecond;
    //         double turn = testSwerveState.angle.getRadians();
    //         modules[0].openLoop(turn, drive);
    //         modules[1].openLoop(turn, drive);
    //         modules[2].openLoop(turn, drive);
    //         modules[3].openLoop(turn, drive);
    //         // for(SDSSwerveModule module : modules) {
    //         //     module.openLoop(
    //         //         Preferences.getDouble("kSwerveTestTurn", SwerveConstants.kDefaultTestTurn),
    //         //         Preferences.getDouble("kSwerveTestDrive", SwerveConstants.kDefaultTestDrive)
    //         //     );
    //         // }
    //     });
    // }

    public Command runZeroGyro() {
        return runOnce(() -> {
            gyroIO.zeroGyro();
        }).andThen(new WaitCommand(0.1)).andThen(() -> {
            poseEstimator.resetRotation(Constants.isRed() ? Rotation2d.kPi : Rotation2d.kZero);
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
            System.out.println("Swerve control constants updated");
        });
    }

    public Command runSetTempPose() {
        return runOnce(() -> setPose(new Pose2d(
            Preferences.getDouble("odometry/setPoseX", 0),
            Preferences.getDouble("odometry/setPoseY", 0),
            new Rotation2d(Preferences.getDouble("odometry/setPoseRot", 0))
        )));
    }

    @AutoLogOutput(key = "Odometry/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> stdDevs) {
        // higher standard deviations means vision measurements are trusted less
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp, stdDevs);
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroIOInputs);  
        Logger.processInputs("Swerve/Gyro", gyroIOInputs);

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
        Logger.recordOutput("Swerve/Positions", updatedModulePositions);
        poseEstimator.update(rawGyroRotation, updatedModulePositions);
        gyroDisconnectedAlert.set(!gyroIOInputs.connected && Constants.currentMode != RobotMode.SIM);
    }
}
