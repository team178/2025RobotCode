package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private boolean ejecting;
    private boolean bouncing;
    private boolean storedCoral;
    private Runnable offPresetRun;

    private double desiredLeftVolts;
    private double desiredRightVolts;
    private double desiredFunnelVolts;
    private double lastUpperPhotosensorTrigger;
    private double lastLowerPhotosensorTrigger;

    private BooleanSupplier isAlignedSupplier;
    private BooleanSupplier scoreComboSupplier;
    private boolean awaitingScoreCombo;
    private boolean dealgaeRunning;

    private DoubleSupplier errorDistanceSupplier;

    public Elevator(ElevatorIO io) {
        this.elevatorIO = io;
        elevatorIOInputs = new ElevatorIOInputsAutoLogged();
        openLoop = false;
        intaking = false;
        ejecting = false;
        bouncing = true;
        dealgaeRunning = false;
        awaitingScoreCombo = false;
        desiredLeftVolts = 0;
        desiredRightVolts = 0;
        desiredFunnelVolts = 0;
        lastUpperPhotosensorTrigger = 0;
        lastLowerPhotosensorTrigger = 0;
        Preferences.initDouble("ele/leftvolts", 0);
        Preferences.initDouble("ele/rightvolts", 0);
        Preferences.initDouble("ele/elevatorvolts", 0);

        ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleoperated");
        teleopTab.addBoolean("Low Photo", () -> elevatorIOInputs.lowerPhotosensor)
            .withPosition(9, 2)
            .withSize(1, 1);
        teleopTab.addBoolean("Up Photo", () -> elevatorIOInputs.upperPhotosensor)
            .withPosition(9, 1)
            .withSize(1, 1);
        // teleopTab.addBoolean("Low Limit", () -> elevatorIOInputs.lowLimit)
        //     .withPosition(3, 1)
        //     .withSize(1, 1);
        // teleopTab.addBoolean("High Limit", () -> elevatorIOInputs.highLimit)
        //     .withPosition(3, 0)
        //     .withSize(1, 1);
        teleopTab.addString("Limit Switches", () ->
            (Timer.getFPGATimestamp() % 1 > 0.5 ? "\\ " : "/ ") + 
            (elevatorIOInputs.lowerPhotosensor ?
            (elevatorIOInputs.upperPhotosensor ? "Both" : "Low") :
            (elevatorIOInputs.lowerPhotosensor ? "High" : "None")) +
            (Timer.getFPGATimestamp() % 1 > 0.5 ? " /" : " \\"))
            .withPosition(8, 1)
            .withSize(1, 1);
        teleopTab.addString("Elevator Position", () -> elevatorIOInputs.desiredPosition.name + (intaking ? " + Intaking" : ""))
            .withPosition(8, 0)
            .withSize(2, 1);
        // teleopTab.addBoolean("\"Intaking\"", () -> intaking)
        //     .withPosition(2, 2)
        //     .withSize(1, 1);
        teleopTab.addBoolean("Bouncing", () -> bouncing)
            .withPosition(9, 3)
            .withSize(1, 1);
        teleopTab.addBoolean("Awaiting Home", () -> awaitingScoreCombo)
            .withPosition(8, 3)
            .withSize(1, 1);
    }

    public void setIsAlignedSupplier(BooleanSupplier isAlignedSupplier) {
        this.isAlignedSupplier = isAlignedSupplier;
    }

    public void setScoreComboSupplier(BooleanSupplier scoreComboSupplier) {
        this.scoreComboSupplier = scoreComboSupplier;
    }
    
    public void setErrorDistanceSupplier(DoubleSupplier errorDistanceSupplier) {
        this.errorDistanceSupplier = errorDistanceSupplier;
    }

    public void setOffPresetRun(Runnable offPresetRun) {
        this.offPresetRun = offPresetRun;
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
                desiredLeftVolts = 0;
                desiredRightVolts = 0;
                // elevatorIO.setEffectorVolts(0, 0);
                // elevatorIO.setFunnelMotorVolts(0);
                desiredFunnelVolts = 0;
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
        return runOnce(() -> {
            desiredLeftVolts = left;
            desiredRightVolts = right;
            // elevatorIO.setEffectorVolts(left, right);
        });
    }

    public Command runEffectorPreferences() {
        return runOnce(() -> {
            desiredLeftVolts = Preferences.getDouble("ele/leftvolts", 0);
            desiredRightVolts = Preferences.getDouble("ele/rightvolts", 0);
            // elevatorIO.setEffectorVolts(Preferences.getDouble("ele/leftvolts", 0), Preferences.getDouble("ele/rightvolts", 0));
        });
    }

    public Command runReversedEffectorPreferences() {
        return runOnce(() -> {
            desiredLeftVolts = -Preferences.getDouble("ele/leftvolts", 0);
            desiredRightVolts = -Preferences.getDouble("ele/rightvolts", 0);
            // elevatorIO.setEffectorVolts(-Preferences.getDouble("ele/leftvolts", 0), -Preferences.getDouble("ele/rightvolts", 0));
        });
    }

    public Command runSetFunnelVolts(double volts) {
        // return runOnce(() -> elevatorIO.setFunnelMotorVolts(volts));
        return runOnce(() -> desiredFunnelVolts = volts);
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

    /**
     * Moves elevator up, waits until elevator is (almost) at position, expels coral, waits until it's placed, stops end effector, then lowers elevator.
     * @param position Elevator height to raise elevator to and score at.
     * @return Command Group representing the full scoring movement.
     */
    public Command runWaitToElevatorPosition(ElevatorPosition position, double tolerance) {
        return runToElevatorPosition(position)
            .andThen(new WaitUntilCommand(() -> Math.abs(position.height - elevatorIOInputs.elevatorHeight) < tolerance))
            .andThen(Commands.print("Done waiting"));
    }

    public Command runEjectScore() {
        return Commands.print("start eject")
        .andThen(runOnce(() -> {
            if(!elevatorIOInputs.desiredPosition.equals(ElevatorPosition.L1)) {
                desiredLeftVolts = -4;
                desiredRightVolts = 4;
            } else {
                desiredLeftVolts = -6;
                desiredRightVolts = 3;
            }
        }))
            // (!elevatorIOInputs.desiredPosition.equals(ElevatorPosition.L1)) ? (runEffector(-4, 4)) : (runEffector(-6, 3)))
        .andThen(Commands.print("please please please again " + desiredLeftVolts + " " + desiredRightVolts))
            .andThen(new WaitUntilCommand(() -> !getLowerPhotosensor()))
            .andThen(new WaitCommand(0.1)) // TODO may want to look at tweaking the time here
            .andThen(Commands.print("uh oh stop " + desiredLeftVolts + " " + desiredRightVolts))
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

    public Command runIntakeEffector(double effectorVolts, double funnelVolts) {
        return runOnce(() -> {
            if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) && intaking) {
                intaking = false;
                desiredLeftVolts = 0;
                desiredRightVolts = 0;
                // elevatorIO.setEffectorVolts(0, 0);
                // elevatorIO.setFunnelMotorVolts(0);
                desiredFunnelVolts = 0;
            } else {
                if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME)) {
                    intaking = true;
                    // elevatorIO.setFunnelMotorVolts(funnelVolts);
                    desiredFunnelVolts = funnelVolts;
                    desiredLeftVolts = -effectorVolts;
                    desiredRightVolts = effectorVolts;
                    // elevatorIO.setEffectorVolts(-effectorVolts, effectorVolts);
                } else {
                    awaitingScoreCombo = scoreComboSupplier.getAsBoolean();
                    if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.L1)) {
                        desiredLeftVolts = -effectorVolts * 6 / 7 * 5 / 4;
                        desiredRightVolts = effectorVolts * 3 / 7 * 5 / 4;
                        // elevatorIO.setEffectorVolts(-effectorVolts * 6 / 7, effectorVolts * 3 / 7); // over 5 to over 7
                    } else {
                        desiredLeftVolts = -effectorVolts;
                        desiredRightVolts = effectorVolts;
                        // elevatorIO.setEffectorVolts(-effectorVolts, effectorVolts);
                    }
                }
            }
        }).andThen(runWaitStopIntake());
    }

    public Command runStopIntakeEffector() {
        return runOnce(() -> {
            if(!intaking) {
                desiredLeftVolts = 0;
                desiredRightVolts = 0;
                // elevatorIO.setEffectorVolts(0, 0);
            }
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
                elevatorIO.setElevatorPosition(ElevatorPosition.HOME.height + (0.005 * Math.sin(Timer.getFPGATimestamp() * 12)));
            } else {
                if(elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) && bouncing) {
                    elevatorIO.setElevatorPosition(errorDistanceSupplier.getAsDouble() < 0.5 ? ElevatorPosition.homeCoralPos : ElevatorPosition.HOME.height);
                } else {
                    elevatorIO.setElevatorPosition(elevatorIOInputs.desiredHeight);
                }
            }
            // elevatorIO.setElevatorPosition(elevatorIOInputs.desiredPosition);
        }

        double realDesiredHeight = Math.max(Math.min(elevatorIOInputs.desiredHeight, 0.612), 0);
        if(desiredLeftVolts == 0) {
            elevatorIO.setLeftEffectorVolts(desiredLeftVolts);
        } else if((elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) ||
            (Math.abs(realDesiredHeight - elevatorIOInputs.elevatorHeight) < 0.0015 &&
            (isAlignedSupplier == null || isAlignedSupplier.getAsBoolean())
            )) || DriverStation.isAutonomous()
        ) {
            elevatorIO.setLeftEffectorVolts(desiredLeftVolts);
        } else {
            elevatorIO.setLeftEffectorVolts(0);
        }
        if(desiredRightVolts == 0) {
            elevatorIO.setRightEffectorVolts(desiredRightVolts);
        } else if((elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) ||
            (Math.abs(realDesiredHeight - elevatorIOInputs.elevatorHeight) < 0.0015 &&
            (isAlignedSupplier == null || isAlignedSupplier.getAsBoolean())
            )) || DriverStation.isAutonomous()
        ) {
            elevatorIO.setRightEffectorVolts(desiredRightVolts);
        } else {
            elevatorIO.setRightEffectorVolts(0);
        }

        if(elevatorIOInputs.upperPhotosensor) {
            lastUpperPhotosensorTrigger = Timer.getFPGATimestamp();
        }
        if(elevatorIOInputs.lowerPhotosensor) {
            lastLowerPhotosensorTrigger = Timer.getFPGATimestamp();
        }

        if(Timer.getFPGATimestamp() - lastUpperPhotosensorTrigger < 0.5 && !elevatorIOInputs.lowerPhotosensor) {
            elevatorIO.setFunnelMotorVolts(Timer.getFPGATimestamp() % 6 > 1.5 && Timer.getFPGATimestamp() % 1.5 > 0.75 ? -desiredFunnelVolts : desiredFunnelVolts);
        } else {
            elevatorIO.setFunnelMotorVolts(desiredFunnelVolts);
        }

        if(!dealgaeRunning && elevatorIOInputs.elevatorHeight > 0.01) {
            dealgaeRunning = true;
            elevatorIO.setDealgaeMotorVolts(9);
        }
        if(dealgaeRunning && elevatorIOInputs.elevatorHeight < 0.01) {
            dealgaeRunning = false;
            elevatorIO.setDealgaeMotorVolts(0);
        }

        if(
            awaitingScoreCombo &&
            !elevatorIOInputs.desiredPosition.equals(ElevatorPosition.HOME) &&
            Timer.getFPGATimestamp() - lastLowerPhotosensorTrigger > 0.3 &&
            Math.abs(realDesiredHeight - elevatorIOInputs.elevatorHeight) < 0.0015
        ) {
            awaitingScoreCombo = false;
            elevatorIO.setElevatorPosition(ElevatorPosition.HOME);
        }

        if(storedCoral != hasCoral()) {
            storedCoral = !storedCoral;
            if(offPresetRun != null && !storedCoral) {
                offPresetRun.run();
            }
        }

        Logger.recordOutput("Elevator/intaking", intaking);
        Logger.recordOutput("Elevator/bouncing", bouncing);
        Logger.recordOutput("Elevator/subdesiredLeftVolts", desiredLeftVolts);
        Logger.recordOutput("Elevator/subdesiredRightVolts", desiredRightVolts);
        Logger.recordOutput("Elevator/subdesiredFunnelVolts", desiredFunnelVolts);
        Logger.recordOutput("Elevator/lastUpperPhotosensorTrigger", lastUpperPhotosensorTrigger);
        Logger.recordOutput("Elevator/lastLowerPhotosensorTrigger", lastLowerPhotosensorTrigger);
        Logger.recordOutput("Elevator/awaitingScoreCombo", awaitingScoreCombo);
        Logger.recordOutput("Elevator/dealgaeRunning", dealgaeRunning);
        Logger.recordOutput("Elevator/storedCoral", storedCoral);
    }
 }
