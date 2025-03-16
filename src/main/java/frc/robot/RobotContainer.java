// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOSpark;
import frc.robot.subsystems.manipulator.ManipulatorPosition;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.Pigeon2IO;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive.PresetPositionType;
import frc.robot.subsystems.vision.LimelightLocations;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Combo;

public class RobotContainer {
    private CommandXboxController driverController;
    private CommandXboxController auxController;

    private SwerveDrive swerve;
    private Elevator elevator;
    private Manipulator manipulator;
    private Climber climber;
    private Vision vision;

    private Command autoCommand;

    public RobotContainer() {
        Preferences.removeAll();
        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        auxController = new CommandXboxController(OperatorConstants.kAuxControllerPort);

        switch(Constants.currentMode) {
            case REAL:
            case SIM: // no sim classes for now
                swerve = new SwerveDrive(
                    new Pigeon2IO(),
                    // new GyroIO() {},
                    new SDSModuleIOSpark(0),
                    new SDSModuleIOSpark(1),
                    new SDSModuleIOSpark(2),
                    new SDSModuleIOSpark(3)
                );
                elevator = new Elevator(new ElevatorIOSpark());
                // manipulator = new Manipulator(new ManipulatorIOSpark());
                climber = new Climber(new ClimberIOSpark());
                vision = new Vision(
                    swerve::addVisionMeasurement
                    , new VisionIOLimelight(LimelightLocations.FRONT3, () -> swerve.getPose().getRotation())
                    , new VisionIOLimelight(LimelightLocations.HIGH3, () -> swerve.getPose().getRotation())
                    // , new VisionIOLimelight(LimelightLocations.SIDE2, () -> swerve.getPose().getRotation())
                );
                break;
            default:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                // manipulator = new Manipulator(new ManipulatorIO() {});
                climber = new Climber(new ClimberIO() {});
                // vision = new Vision((pose, timestamp, stdDevs) -> {}, new VisionIO() {});
                break;
        }

        configureBindings();
        updateAutoCommand(); // ensure Autos class is loaded into memory
    }

    private void configureBindings() {
        Combo overrideElevatorFactorCombo = new Combo("overrideElevatorFactor Combo", 0.5,
            driverController.x(),
            driverController.x().negate(),
            driverController.rightTrigger(0.75),
            driverController.x()
        );
        overrideElevatorFactorCombo.getTrigger().onTrue(Commands.runOnce(() -> {}));
        Shuffleboard.getTab("Teleoperated")
            .addBoolean("EleFact Override", overrideElevatorFactorCombo.getTrigger()).
            withPosition(8, 1)
            .withSize(1, 1);

        swerve.setToAimSuppliers(
            driverController.leftTrigger()::getAsBoolean, // aim reef
            driverController.b()::getAsBoolean, // aim processor
            driverController.a()::getAsBoolean // aim station
        );
        swerve.setReefChooserSuppliers(
            driverController::getRightX,
            driverController::getRightY
        );
        swerve.setElevatorSuppliers(elevator::getElevatorHeight, overrideElevatorFactorCombo.getTrigger());
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX, // vx
            driverController::getLeftY, // vy
            driverController::getRightX, // omega
            driverController::getRightTriggerAxis, // raw slow input
            driverController.leftBumper()::getAsBoolean, // robot centric
            driverController.rightBumper()::getAsBoolean // no optimize
        ));

        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.back().onTrue(swerve.runToggleToXPosition(true));
        driverController.b().onTrue(swerve.runUpdateControlConstants().andThen(elevator.runUpdateControlConstants()));
        driverController.start().onTrue(swerve.runSetTempPose());
        driverController.povLeft().onTrue(swerve.runTogglePresetPosition(PresetPositionType.LEFTREEF));
        driverController.povRight().onTrue(swerve.runTogglePresetPosition(PresetPositionType.RIGHTREEF));
        driverController.povUp().onTrue(swerve.runTogglePresetPosition(PresetPositionType.PROCESSOR));
        
        // if(Constants.simMode.equals(RobotMode.SIM)) {
        //     swerve.setDefaultCommand(swerve.runSimOdometryMoveBy(
        //         driverController::getLeftX,
        //         driverController::getLeftY,
        //         driverController::getRightX
        //     ));
        // }

        // driverController.a().onTrue(swerve.runTestDrive());
        // driverController.a().onFalse(swerve.runStopDrive());
        // driverController.x().onTrue(swerve.runOpenTestDrive());
        // driverController.x().onFalse(swerve.runStopDrive());

        // driverController.povUp().onTrue(elevator.runEffectorPreferences());
        // driverController.povUp().onFalse(elevator.runEffector(0, 0));
        // driverController.povDown().onTrue(elevator.runaEffectorPreferences());
        // driverController.povDown().onFalse(elevator.runEffector(0, 0));

        // driverController.povLeft().onTrue(elevator.runElevatorOpenLoopPreferences());
        // driverController.povLeft().onFalse(elevator.runElevatorOpenLoop(0));
        // driverController.povRight().onTrue(elevator.runaElevatorOpenLoopPreferences());
        // driverController.povRight().onFalse(elevator.runElevatorOpenLoop(0));

        Combo alignedOverrideCombo = new Combo("isAligned Override Combo", 0.5,
            auxController.leftTrigger(),
            auxController.leftTrigger().negate(),
            auxController.leftTrigger()
        );
        alignedOverrideCombo.getTrigger().onTrue(Commands.run(() -> {}));
        Shuffleboard.getTab("Teleoperated")
            .addBoolean("Aligned Override", alignedOverrideCombo.getTrigger()).
            withPosition(3, 2)
            .withSize(1, 1);

        auxController.b().onTrue(elevator.runToElevatorPosition(ElevatorPosition.HOME));
        auxController.a().onTrue(elevator.runToElevatorPosition(ElevatorPosition.L1));
        auxController.x().onTrue(elevator.runToElevatorPosition(ElevatorPosition.L2));
        auxController.y().onTrue(elevator.runToElevatorPosition(ElevatorPosition.L3));
        auxController.leftBumper().onTrue(elevator.runIntakeEffector(
            5, // effector volts
            -2, // funnel volts
            () -> swerve.isAligned() || alignedOverrideCombo.getTrigger().getAsBoolean() // is aligned supplier
            // () -> true
        ));
        auxController.leftBumper().onFalse(elevator.runStopIntakeEffector());

        // auxController.leftBumper().onTrue(elevator.runEffector(-5, 5));
        // auxController.leftBumper().onFalse(elevator.runEffector(0, 0));
        // should auto adjust speed for L1
        // auxController.leftTrigger().onTrue(elevator.runEffector(-6, 3));
        // auxController.leftTrigger().onFalse(elevator.runEffector(0, 0));
        auxController.leftTrigger().onTrue(elevator.runUpdateControlConstants());
        // auxController.rightBumper().onTrue(elevator.runIntakeFunnel(-2));
        // auxController.rightBumper().onFalse(elevator.runSetFunnelVolts(0));

        auxController.leftStick().onTrue(elevator.runSetDealgaeVolts(5));
        auxController.leftStick().onFalse(elevator.runSetDealgaeVolts(0));
        auxController.rightStick().onTrue(elevator.runSetDealgaeVolts(-8));
        auxController.rightStick().onFalse(elevator.runSetDealgaeVolts(0));
        auxController.povUp().whileTrue(elevator.runJogElevatorPosition(0.001));
        auxController.povDown().whileTrue(elevator.runJogElevatorPosition(-0.001));

        auxController.rightBumper().onTrue(elevator.runToggleBouncing());

        auxController.rightBumper().onTrue(climber.runSetClimberVolts(10));
        auxController.rightBumper().onFalse(climber.runSetClimberVolts(0));
        auxController.rightTrigger().onTrue(climber.runSetClimberVolts(-10));
        auxController.rightTrigger().onFalse(climber.runSetClimberVolts(0));

        // auxController.a().onTrue(manipulator.runSetManipulatorPosition(ManipulatorPosition.HOME));
        // auxController.b().onTrue(manipulator.runSetManipulatorPosition(ManipulatorPosition.INTAKE));
        // auxController.x().onTrue(manipulator.runSetManipulatorPosition(ManipulatorPosition.CARRY));

        // Combo testCombo = new Combo("test combo", 1,
        //     driverController.a(),
        //     driverController.a().negate(),
        //     driverController.b(),
        //     driverController.b().negate(),
        //     driverController.x(),
        //     driverController.x().negate(),
        //     driverController.a(),
        //     driverController.a().negate(),
        //     driverController.y()
        // );
        // testCombo.getTrigger().onTrue(Commands.print("triggered"));
    }

    private void updateAutoCommand() {
        autoCommand = Autos.getAutoCommand(swerve, elevator);
    }

    public Command getAutonomousCommand() {
        updateAutoCommand();
        return autoCommand;
    }
}
