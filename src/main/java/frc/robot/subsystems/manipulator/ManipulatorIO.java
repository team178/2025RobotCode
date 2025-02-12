package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs {
        public ManipulatorPosition desiredPosition = ManipulatorPosition.HOME;
        public double desiredDeployVoltage = 0;
        public double desiredRollerVoltage = 0;

        public double manipulatorAngle = 0;
        public double manipulatorVelocity = 0;
    }

    public default void updateInputs(ManipulatorIOInputs inputs) {}

    public default void setManipulatorPosition(ManipulatorPosition position) {}

    public default void setDeployOpenLoop(double output) {};

    public default void setRollerVolts(double volts) {};

    public default void updateControlConstants() {};
}
