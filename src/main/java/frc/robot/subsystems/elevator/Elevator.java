package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs;
    private boolean openLoop;

    public Elevator(ElevatorIO effectorIO) {
        this.elevatorIO = effectorIO;
        elevatorIOInputs = new ElevatorIOInputsAutoLogged();
        openLoop = false;
        Preferences.initDouble("ele/leftvolts", 0);
        Preferences.initDouble("ele/rightvolts", 0);
        Preferences.initDouble("ele/elevatorvolts", 0);
    }

    public Command runElevatorOpenLoop(double volts) {
        return runOnce(() -> {
            openLoop = true;
            elevatorIO.setElevatorOpenLoop(volts);
        });
    }
    
    public Command runElevatorOpenLoopPreferences() {
        return runOnce(() -> elevatorIO.setElevatorOpenLoop(Preferences.getDouble("ele/elevatorvolts", 0)));
    }
    
    public Command runaElevatorOpenLoopPreferences() {
        return runOnce(() -> elevatorIO.setElevatorOpenLoop(-Preferences.getDouble("ele/elevatorvolts", 0)));
    }

    public Command runToElevatorPosition(ElevatorPosition position) {
        return runOnce(() -> {
            openLoop = false;
            elevatorIO.setElevatorPosition(position);
        });
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

    public Command runEffectorPreferences() {
        return runOnce(() -> elevatorIO.setEffectorVolts(Preferences.getDouble("ele/leftvolts", 0), Preferences.getDouble("ele/rightvolts", 0)));
    }

    public Command runaEffectorPreferences() {
        return runOnce(() -> elevatorIO.setEffectorVolts(-Preferences.getDouble("ele/leftvolts", 0), -Preferences.getDouble("ele/rightvolts", 0)));
    }

    public Command runSetFunnelVolts(double volts) {
        return runOnce(() -> elevatorIO.setFunnelMotorVolts(volts));
    }

    public boolean getUpperPhotosensor() {
        return elevatorIOInputs.upperPhotosensor;
    }

    public boolean getLowerPhotosensor() {
        return elevatorIOInputs.lowerPhotosensor;
    }

    public Command runUpdateControlConstants() {
        return runOnce(() -> elevatorIO.updateControlConstants());
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Elevator", elevatorIOInputs);

        if(elevatorIOInputs.lowLimit) {
            elevatorIO.resetElevatorEncoder(0);
        } else if(elevatorIOInputs.highLimit) {
            elevatorIO.resetElevatorEncoder(0.61);
        }
        if(!openLoop) elevatorIO.setElevatorPosition(elevatorIOInputs.desiredPosition);
    }

    /**
     * Moves elevator up, expells the Coral, waits until it's gone, turns off end effector, then lowers elevator.
     * @param position Elevator height to raise elevator to and score at.
     * @return Command Group representing the full scoring movement.
     */
    public Command runScoreToElevatorPosition(ElevatorPosition position) {
        return runToElevatorPosition(position)
            .andThen(!position.equals(ElevatorPosition.L1) ? runEffector(-4, 4) : runEffector(-6, 3))
            .andThen(new WaitUntilCommand(() -> !getLowerPhotosensor())) // May need to add a timer to this
            .andThen(runEffector(0, 0))
            .andThen(runToElevatorPosition(ElevatorPosition.HOME));
    }

    /**
     * Moves elevator to home position, turns on the funnel, and turns on the end effector. Then, once photosensor sees Coral, the end effector and funnel turn off.
     * @return Command Group representing the full intaking movement.
     */
    public Command runIntakeFromCoralStation() {
        return runToElevatorPosition(ElevatorPosition.HOME)
            .alongWith(runSetFunnelVolts(4)) // THIS IS COMPLETELY ARBITRARY PLEASE REPLACE THIS WITH WHATEVER NUMBER ANDY HAS BEEN USING
            .alongWith(runEffector(-4, 4))
            .andThen(new WaitUntilCommand(this::getLowerPhotosensor)) // Probably best to replace this with upper photosensor and add delay once upper photosensor is reliable
            .andThen(
                runEffector(0, 0)
                .alongWith(runSetFunnelVolts(0))
            );
    }
}
