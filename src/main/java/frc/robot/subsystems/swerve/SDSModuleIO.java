package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SDSModuleIO {
    @AutoLog
    public static class SDSModuleIOInputs {
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0;
        
        public double turnAppliedVolts = 0;
        public double turnCurrentAmps = 0;
        
        public double drivePositionRad = 0;
        public double driveVelocityRadPerSec = 0;
        
        public double driveAppliedVolts = 0;
        public double driveCurrentAmps = 0;

        public SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d(0));
    }
    
    /** Updates loggable inputs. */
    public default void updateInputs(SDSModuleIOInputs inputs) {}

    /** Set turn position setpoint */
    public default void setTurnPosition(Rotation2d position) {}

    /** Set drive velocity setpoint */
    public default void setDriveVelocity(double velocity) {}

    /** Run turn motor at specified open loop value. */
    public default void setTurnOpenLoop(double output) {}

    /** Run drive motor at specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    /** Update control constants */
    public default void updateControlConstants() {}
}
