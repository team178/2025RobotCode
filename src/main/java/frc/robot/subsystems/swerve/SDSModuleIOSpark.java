package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SDSModuleIOSpark implements SDSModuleIO {
    private SparkMax turnMotor;
    private SparkMax driveMotor;

    private SparkAbsoluteEncoder turnEncoder; // SRX Mag Encoder
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController turnControlller;
    private SparkClosedLoopController driveController;

    private Rotation2d zeroRotation;

    public SDSModuleIOSpark(int module, boolean useAbsolute) {
        turnMotor = new SparkMax(
            switch(module) {
                case 0 -> SwerveConstants.kFLTurnCANID;
                case 1 -> SwerveConstants.kFRTurnCANID;
                case 2 -> SwerveConstants.kBLTurnCANID;
                case 3 -> SwerveConstants.kBRTurnCANID;
                default -> 0;
            },
            MotorType.kBrushless);
        driveMotor = new SparkMax(
            switch(module) {
                case 0 -> SwerveConstants.kFLDriveCANID;
                case 1 -> SwerveConstants.kFRDriveCANID;
                case 2 -> SwerveConstants.kBLDriveCANID;
                case 3 -> SwerveConstants.kBRDriveCANID;
                default -> 0;
            },
            MotorType.kBrushless);
        zeroRotation = switch(module) {
            case 0 -> SwerveModuleConstants.FLZeroRotation;
            case 1 -> SwerveModuleConstants.FRZeroRotation;
            case 2 -> SwerveModuleConstants.BLZeroRotation;
            case 3 -> SwerveModuleConstants.BRZeroRotation;
            default -> new Rotation2d();
        };

        SparkMaxConfig turnConfig = Constants.SwerveModuleConstants.turnConfig;
        SparkMaxConfig driveConfig = Constants.SwerveModuleConstants.driveConfig;

        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // JUST GOTTA HOPE THIS ATTACHES TO THE ABSOLUTE ENCODER CORRECTLY (changed from last year)

        // likely disables CAN timeout, likely helpful, seen in other teams' code
        turnMotor.setCANTimeout(0);
        driveMotor.setCANTimeout(0);

        turnEncoder = turnMotor.getAbsoluteEncoder();
        driveEncoder = driveMotor.getEncoder();

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void updateInputs(SDSModuleIOInputs inputs) {
        inputs.turnPosition = new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
        
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.turnAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.turnCurrentAmps = driveMotor. getOutputCurrent();
    }

    public void setTurnPosition(Rotation2d position) {
        double setpoint = MathUtil.inputModulus(position.rotateBy(zeroRotation).getRadians(), 0, 2 * Math.PI);
        turnControlller.setReference(setpoint, ControlType.kPosition);
    }

    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        driveController.setReference(velocityRadPerSec, ControlType.kVelocity);
    }

    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
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
}
