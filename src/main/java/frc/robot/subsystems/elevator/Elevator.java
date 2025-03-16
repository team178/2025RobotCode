package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs;
    private boolean openLoop;

    private boolean intaking;
    private boolean bouncing;

    public Elevator(ElevatorIO io) {
        this.elevatorIO = io;
        elevatorIOInputs = new ElevatorIOInputsAutoLogged();
        openLoop = false;
        intaking = false;
        bouncing = true;
        Preferences.initDouble("ele/leftvolts", 0);
        Preferences.initDouble("ele/rightvolts", 0);
        Preferences.initDouble("ele/elevatorvolts", 0);

        ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleoperated");
        teleopTab.addBoolean("Low Photo", () -> elevatorIOInputs.lowerPhotosensor)
            .withPosition(2, 1)
            .withSize(1, 1);
        teleopTab.addBoolean("Up Photo", () -> elevatorIOInputs.upperPhotosensor)
            .withPosition(2, 0)
            .withSize(1, 1);
        teleopTab.addBoolean("Low Limit", () -> elevatorIOInputs.lowLimit)
            .withPosition(3, 1)
            .withSize(1, 1);
        teleopTab.addBoolean("High Limit", () -> elevatorIOInputs.highLimit)
            .withPosition(3, 0)
            .withSize(1, 1);
        teleopTab.addString("Elevator Position", () -> elevatorIOInputs.desiredPosition.name)
            .withPosition(0, 2)
            .withSize(2, 1);
        teleopTab.addBoolean("\"Intaking\"", () -> intaking)
            .withPosition(2, 2)
            .withSize(1, 1);
        teleopTab.addBoolean("Home Bounce", () -> bouncing)
            .withPosition(7, 0)
            .withSize(1, 1);
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
            if(intaking) {
                elevatorIO.setEffectorVolts(0, 0);
                elevatorIO.setFunnelMotorVolts(0);
                intaking = false;
            }
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

    public boolean hasCoral() {
        return getLowerPhotosensor() && getUpperPhotosensor();
    }

    public double getElevatorHeight() {
        return elevatorIOInputs.elevatorHeight;
    }

    public Command runUpdateControlConstants() {
        return runOnce(() -> elevatorIO.updateControlConstants());
    }

    public Command runToggleBouncing() {
        return runOnce(() -> {
            bouncing = !bouncing;
        });
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Elevator", elevatorIOInputs);

        if(elevatorIOInputs.lowLimit) {
            elevatorIO.resetElevatorEncoder(0);
        } else if(elevatorIOInputs.highLimit) {
            elevatorIO.resetElevatorEncoder(0.612);
        }
        if(!openLoop) {
            if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) && !hasCoral() && bouncing) {
                elevatorIO.setElevatorPosition(ElevatorPosition.HOME.height + (0.003 * Math.sin(Timer.getFPGATimestamp() * 12)));
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
            .andThen(new WaitUntilCommand(this::hasCoral)) // TODO check
            .andThen(new WaitCommand(0.1))
            .andThen(runEffector(0, 0))
            .andThen(runSetFunnelVolts(0));
    }

    public Command runIntakeEffector(double effectorVolts, double funnelVolts, BooleanSupplier isAlignedSupplier) {
        return runOnce(() -> {
            if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) && intaking) {
                intaking = false;
                elevatorIO.setEffectorVolts(0, 0);
                elevatorIO.setFunnelMotorVolts(0);
            } else {
                if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME)) {
                    intaking = true;
                    elevatorIO.setFunnelMotorVolts(funnelVolts);
                    elevatorIO.setEffectorVolts(-effectorVolts, effectorVolts);
                } else if(isAlignedSupplier.getAsBoolean()) {
                    if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.L1)) {
                        elevatorIO.setEffectorVolts(-effectorVolts * 6 / 7, effectorVolts * 3 / 7); // over 5 to over 7
                    } else {
                        elevatorIO.setEffectorVolts(-effectorVolts, effectorVolts);
                    }
                }
            }
        }).andThen(runWaitStopIntake());
    }

    public Command runStopIntakeEffector() {
        return runOnce(() -> {
            if(!intaking) elevatorIO.setEffectorVolts(0, 0);
        }).andThen(runWaitStopIntake());
    }

    public Command runIntakeFunnel(double volts) {
        return runSetFunnelVolts(volts)
            .andThen(runWaitStopIntake());
    }

    public Command runWaitStopIntake() {
        return new WaitUntilCommand(() -> hasCoral() && elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME))
        .andThen(new WaitCommand(0.03))
        .andThen(runEffector(0, 0)
        .andThen(runSetFunnelVolts(0))
        .andThen(runOnce(() -> intaking = false)));
    }
 }
