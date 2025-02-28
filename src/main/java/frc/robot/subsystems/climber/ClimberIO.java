package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        // public ClimberPosition desiredPosition = ClimberPosition.HOME;

        // public double climberHeight = 0;
        // public double climberVelocity = 0;

        // public double elevatorAppliedVolts = 0;
        // public double elevatorCurrentAmps = 0;
    }
    
    // public default void updateInputs(ClimberIOInputs inputs) {}

    // public default void setClimberPosition(ClimberPosition climberPosition) {}
    
    public default void setClimberVolts(boolean low, boolean high) {}

    // public default void updateControlConstants() {}

    // public default void setToggle(boolean toggle) {}

    public default void climberInputs(boolean set, boolean up, boolean down){}

    public default void updateKG() {}
}
