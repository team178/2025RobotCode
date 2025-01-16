package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;

public class SDSSwerveModule {
    public static final NetworkTable constantPreferences = NetworkTableInstance.getDefault().getTable("Swerve Modules");
    
    private String name;

    private SparkMax turnMotor;
    private SparkMax driveMotor;

    private RelativeEncoder turnRelEncoder;
    private SparkAbsoluteEncoder turnAbsEncoder; // SRX Mag Encoder
    private RelativeEncoder driveEncoder;
    private boolean useAbsolute;

    private SparkClosedLoopController turnControlller;
    private SparkClosedLoopController driveController;

    private SwerveModuleState desiredModuleState;

    private Rotation2d zeroRotation;

    private NetworkTable moduleNT;

    public SDSSwerveModule(String name, int turnID, int driveID, Rotation2d zeroRotation, boolean useAbsolute) {
        Preferences.initBoolean("Swerve Modules/" + name + "enabled", true);
        this.name = name;
        moduleNT = constantPreferences.getSubTable(name);

        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);

        SparkMaxConfig turnConfig = Constants.SwerveModuleConstants.turnConfig;
        SparkMaxConfig driveConfig = Constants.SwerveModuleConstants.driveConfig;

        this.zeroRotation = zeroRotation;
        this.useAbsolute = useAbsolute;
        if(useAbsolute) turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // JUST GOTTA HOPE THIS ATTACHES TO THE ABSOLUTE ENCODER CORRECTLY (changed from last year)

        // likely disables CAN timeout, likely helpful, seen in other teams' code
        turnMotor.setCANTimeout(0);
        driveMotor.setCANTimeout(0);

        turnRelEncoder = turnMotor.getEncoder(); // only use if NO ACCESS to absolute encoder (SRX Mag Encoder)
        turnAbsEncoder = turnMotor.getAbsoluteEncoder();
        driveEncoder = driveMotor.getEncoder();

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredModuleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setDesiredState(desiredState, true);
    }

    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        if(!enabled()) {
            stopDrive();
            return;
        }

        SwerveModuleState correctedState = new SwerveModuleState(state.speedMetersPerSecond, zeroRotation.rotateBy(state.angle)); //! will check if rotateby functions correctly
        if(optimize) {
            if(useAbsolute) {
                correctedState.optimize(Rotation2d.fromRadians(turnAbsEncoder.getPosition()));
                correctedState.cosineScale(Rotation2d.fromRadians(getAdjTurnPos())); //! to be checked; see if adj turn pos is correct, and determine if this is helpful
            } else {
                correctedState.optimize(Rotation2d.fromRadians(turnRelEncoder.getPosition()).div(2));
                correctedState.angle = correctedState.angle.times(2);
                //! This 2x scaling was copied from previous year; may or may not still hold true (only needed for relative anyway, which shouldn't be used in general)
            }
        }
        desiredModuleState = correctedState;
        turnControlller.setReference(correctedState.angle.getRadians(), ControlType.kPosition);
        driveController.setReference(correctedState.speedMetersPerSecond, ControlType.kVelocity);
    }

    public SwerveModuleState stopState() {
        return new SwerveModuleState(0, desiredModuleState.angle);
    }

    public void stopDrive() {
        setDesiredState(stopState());
    }

    public double getRelTurnPos() {
        return turnRelEncoder.getPosition();
    }

    public double getAbsTurnPos() {
        return turnAbsEncoder.getPosition();
    }

    public double getAdjTurnPos() {
        return (getAbsTurnPos() - zeroRotation.getRadians() + 2 * Math.PI) % (2 * Math.PI);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            (useAbsolute
                ? Rotation2d.fromRadians(turnAbsEncoder.getPosition())
                : Rotation2d.fromRadians(turnRelEncoder.getPosition()).div(2) //rel is maybe divide by 2 according to earlier code
            ).rotateBy(zeroRotation.times(-1)) //! check if rotate by negative works as intended
        );
    }

    public SwerveModuleState getDesiredModuleState() {
        return desiredModuleState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            (useAbsolute
                ? Rotation2d.fromRadians(turnAbsEncoder.getPosition())
                : Rotation2d.fromRadians(turnRelEncoder.getPosition()).div(2) //! rel is maybe divide by 2 according to earlier code
            ).rotateBy(zeroRotation.times(-1)) //! check if rotate by negative works as intended
        );
    }

    public void updateControlConstants() { // don't spam run
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        turnConfig.closedLoop
            .p(SwerveModuleConstants.kTurnControlConstants.kP())
            .i(SwerveModuleConstants.kTurnControlConstants.kI())
            .d(SwerveModuleConstants.kTurnControlConstants.kD())
        ; driveConfig.closedLoop
            .p(SwerveModuleConstants.kDriveControlConstants.kP())
            .i(SwerveModuleConstants.kDriveControlConstants.kI())
            .d(SwerveModuleConstants.kDriveControlConstants.kD())
            .velocityFF(SwerveModuleConstants.kDriveControlConstants.kV())
        ;

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean enabled() {
        return Preferences.getBoolean("Swerve Modules/" + name + "enabled", true);
    }
    
    public void putInfo() {
        if(useAbsolute) {
            moduleNT.getEntry("turnRelPos").setDouble(turnRelEncoder.getPosition());
            moduleNT.getEntry("turnRelVel").setDouble(turnRelEncoder.getVelocity());
        } else {
            moduleNT.getEntry("turnAbsPos").setDouble(turnAbsEncoder.getPosition());
            moduleNT.getEntry("turnAbsVel").setDouble(turnAbsEncoder.getVelocity());
            moduleNT.getEntry("turnAdjPos").setDouble(getAdjTurnPos());
        }
        moduleNT.getEntry("driveVel").setDouble(driveEncoder.getVelocity());
        moduleNT.getEntry("desired/speed").setDouble(desiredModuleState.speedMetersPerSecond);
        moduleNT.getEntry("desired/angle").setDouble(desiredModuleState.angle.getRadians());
        moduleNT.getEntry("position/drive").setDouble(getPosition().distanceMeters);
        moduleNT.getEntry("position/turn").setDouble(getPosition().angle.getDegrees());
    }
}
