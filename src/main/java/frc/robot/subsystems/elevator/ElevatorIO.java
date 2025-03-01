package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public ElevatorPosition desiredPosition = ElevatorPosition.HOME;
        public double leftEffectorDesiredVolts = 0;
        public double rightEffectorDesiredVolts = 0;
        public double funnelMotorDesiredVolts = 0;

        public double elevatorHeight = 0;
        public double elevatorVelocity = 0;
        public double leftEffectorVelocity = 0;
        public double rightEffectorVelocity = 0;
        public double funnelMotorVelocity = 0;
        
        public double elevatorAppliedVolts = 0;
        public double elevatorCurrentAmps = 0;
        public double leftEffectorAppliedVolts = 0;
        public double rightEffectorAppliedVolts = 0;
        public double leftEffectorCurrentAmps = 0;
        public double rightEffectorCurrentAmps = 0;
        public double funnelMotorAppliedVolts = 0;
        public double funnelMotorCurrentAmps = 0;

        public boolean highLimit = false;
        public boolean lowLimit = false;
        public boolean upperPhotosensor = false;
        public boolean lowerPhotosensor = false;
    }
    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorPosition(ElevatorPosition position) {}

    public default void setElevatorOpenLoop(double volts) {}
    
    public default void setLeftEffectorVolts(double volts) {}
    
    public default void setRightEffectorVolts(double volts) {}
    
    public default void setEffectorVolts(double left, double right) {}

    public default void setFunnelMotorVolts(double volts) {}

    public default void resetElevatorEncoder(double pos) {}

    public default void updateControlConstants() {}
}
