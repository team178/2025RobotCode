package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
    }
    
    public default void setClimberVolts(boolean low, boolean high) {}

    public default void climberInputs(boolean set, boolean up, boolean down){}

    public default void update() {}

    public default double getPos() {return 0;}

    public default void setVolts(double volts) {}
}
