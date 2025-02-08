package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs;

    public Elevator(ElevatorIO effectorIO) {
        this.elevatorIO = effectorIO;
        elevatorIOInputs = new ElevatorIOInputsAutoLogged();
    }

    public Command runToElevatorPosition(ElevatorPosition position) {
        return runOnce(() -> elevatorIO.setElevatorPosition(position));
    }

    public Command runLeftVolts(double volts) {
        return runOnce(() -> elevatorIO.setLeftEffectorVolts(volts));
    }

    public Command runRightVolts(double volts) {
        return runOnce(() -> elevatorIO.setRightEffectorVolts(volts));
    }

    public Command runEffector(double left, double right) {
        return runOnce(() -> elevatorIO.setEffectorVolts(left, right));
    }

    public boolean getUpperPhotosensor() {
        return elevatorIOInputs.upperPhotosensor;
    }

    public boolean getLowerPhotosensor() {
        return elevatorIOInputs.lowerPhotosensor;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Elevator", elevatorIOInputs);
    }
}
