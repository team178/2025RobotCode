package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs;
    private boolean openLoop;

    private boolean intaking;

    public Elevator(ElevatorIO effectorIO) {
        this.elevatorIO = effectorIO;
        elevatorIOInputs = new ElevatorIOInputsAutoLogged();
        openLoop = false;
        intaking = false;
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

    public Command runJogElevatorPosition(double bump) {
        return run(() -> {
            openLoop = false;
            elevatorIO.setElevatorPosition(elevatorIOInputs.desiredHeight + bump);
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

    public Command runSetDealgaeVolts(double volts) {
        return runOnce(() -> elevatorIO.setDealgaeMotorVolts(volts));
    }

    public boolean getUpperPhotosensor() {
        return elevatorIOInputs.upperPhotosensor;
    }

    public boolean getLowerPhotosensor() {
        return elevatorIOInputs.lowerPhotosensor;
    }

    public double getElevatorHeight() {
        return elevatorIOInputs.elevatorHeight;
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
            elevatorIO.resetElevatorEncoder(0.615);
        }
        if(!openLoop) {
            if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME)) {
                elevatorIO.setElevatorPosition(ElevatorPosition.HOME.height + ((!getLowerPhotosensor() || !getUpperPhotosensor()) && false ? (0.0025 * Math.sin(Timer.getFPGATimestamp() * 12)) : 0));
            } else elevatorIO.setElevatorPosition(elevatorIOInputs.desiredHeight);
            // elevatorIO.setElevatorPosition(elevatorIOInputs.desiredPosition);
        }
    }

    /**
     * Moves elevator up, waits until elevator is (almost) at position, expels coral, waits until it's placed, stops end effector, then lowers elevator.
     * @param position Elevator height to raise elevator to and score at.
     * @return Command Group representing the full scoring movement.
     */
    public Command runScoreToElevatorPosition(ElevatorPosition position) {
        if(position.equals(ElevatorPosition.HOME)) return runToElevatorPosition(position);
        return runToElevatorPosition(position)
            .andThen(new WaitUntilCommand(() -> Math.abs(position.height - elevatorIOInputs.elevatorHeight) < 0.02))
            .andThen(!position.equals(ElevatorPosition.L1) ? runEffector(-4, 4) : runEffector(-6, 3))
            .andThen(new WaitUntilCommand(() -> !getLowerPhotosensor()))
            .andThen(new WaitCommand(0.1)) // TODO may want to look at tweaking the time here
            .andThen(runEffector(0, 0))
            .andThen(runToElevatorPosition(ElevatorPosition.HOME));
    }

    /**
     * Waits until elevator is within tolerance distance away from home, useful for ensuring trajectory following a score is safe
     * @return The appropriate WaitUntilCommand
     */
    public Command runWaitUntilSafeToMove(double tolerance) {
        return new WaitUntilCommand(() -> Math.abs(ElevatorPosition.HOME.height - elevatorIOInputs.elevatorHeight) < tolerance);
    }

    /**
     * Moves elevator to home position, turns on the funnel, and turns on the end effector. Then, once photosensor sees Coral, the end effector and funnel turn off.
     * @return Command Group representing the full intaking movement.
     */
    public Command runIntakeFromCoralStation() {
        return runToElevatorPosition(ElevatorPosition.HOME)
            .andThen(runSetFunnelVolts(-2))
            .andThen(runEffector(-4, 4))
            .andThen(new WaitUntilCommand(() -> getLowerPhotosensor() && getUpperPhotosensor())) // TODO check
            .andThen(new WaitCommand(0.1))
            .andThen(runEffector(0, 0))
            .andThen(runSetFunnelVolts(0));
    }

    public Command runIntakeEffector(double left, double right) {
        return runOnce(() -> {
            if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) && intaking) {
                intaking = false;
                elevatorIO.setEffectorVolts(0, 0);
            } else {
                if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME)) intaking = true;
                else intaking = false;
                elevatorIO.setEffectorVolts(left, right);
            }
        })
            .andThen(new WaitUntilCommand(() -> getLowerPhotosensor() && elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME)))
            .andThen(new WaitCommand(0.03))
            .andThen(runEffector(0, 0));
    }

    public Command runStopIntakeEffector() {
        return runOnce(() -> {
            if(!intaking) elevatorIO.setEffectorVolts(0, 0);
        });
    }

    public Command runIntakeFunnel(double volts) {
        return runSetFunnelVolts(volts)
            .andThen(new WaitUntilCommand(() -> getLowerPhotosensor() && elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME)))
            .andThen(new WaitCommand(0.03))
            .andThen(runEffector(0, 0));
    }
 }
