package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO{
    @AutoLog
    public static class ClimberIOInputs {}
    
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void updateClimberPosition() {}
    
}
