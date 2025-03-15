package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double desiredVolts = 0;
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double encoderPosition = 0;
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberVolts(double volts) {}
}