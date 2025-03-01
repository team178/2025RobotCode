// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
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
import frc.robot.subsystems.vision.LimelightLocations;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

public class RobotContainer {
    private CommandXboxController driverController;
    private CommandXboxController auxController;

    private SwerveDrive swerve;
    private Elevator elevator;
    private Manipulator manipulator;
    private Vision vision;

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
                // vision = new Vision(
                //     swerve::addVisionMeasurement,
                //     new VisionIOLimelight(LimelightLocations.FRONT3, swerve.getPose()::getRotation),
                //     new VisionIOLimelight(LimelightLocations.HIGH2PLUS, swerve.getPose()::getRotation),
                //     new VisionIOLimelight(LimelightLocations.SIDE2, swerve.getPose()::getRotation)
                // );
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
                // vision = new Vision((pose, timestamp, stdDevs) -> {}, new VisionIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        swerve.setToAimSuppliers(
            driverController.x()::getAsBoolean,
            driverController.b()::getAsBoolean,
            driverController.a()::getAsBoolean
        );
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX,
            driverController::getRightTriggerAxis,
            driverController.leftBumper()::getAsBoolean,
            driverController.rightBumper()::getAsBoolean
        ));

        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.back().onTrue(swerve.runToXPosition(true));
        

        // swerve.setDefaultCommand(swerve.runSimOdometryMoveBy(
        //     driverController::getLeftX,
        //     driverController::getLeftY,
        //     driverController::getRightX
        // ));

        // driverController.a().onTrue(swerve.runTestDrive());
        // driverController.a().onFalse(swerve.runStopDrive());
        // driverController.x().onTrue(swerve.runOpenTestDrive());
        // driverController.x().onFalse(swerve.runStopDrive());
        driverController.b().onTrue(swerve.runUpdateControlConstants().andThen(elevator.runUpdateControlConstants()));
        driverController.start().onTrue(swerve.runSetTempPose());

        driverController.povUp().onTrue(elevator.runEffectorPreferences());
        driverController.povUp().onFalse(elevator.runEffector(0, 0));
        driverController.povDown().onTrue(elevator.runaEffectorPreferences());
        driverController.povDown().onFalse(elevator.runEffector(0, 0));

        driverController.povLeft().onTrue(elevator.runElevatorOpenLoopPreferences());
        driverController.povLeft().onFalse(elevator.runElevatorOpenLoop(0));
        driverController.povRight().onTrue(elevator.runaElevatorOpenLoopPreferences());
        driverController.povRight().onFalse(elevator.runElevatorOpenLoop(0));

        // auxController.a().onTrue(manipulator.runSetManipulatorPosition(ManipulatorPosition.HOME));
        // auxController.b().onTrue(manipulator.runSetManipulatorPosition(ManipulatorPosition.INTAKE));
        // auxController.x().onTrue(manipulator.runSetManipulatorPosition(ManipulatorPosition.CARRY));

        auxController.b().onTrue(elevator.runToElevatorPosition(ElevatorPosition.HOME));
        auxController.x().onTrue(elevator.runToElevatorPosition(ElevatorPosition.L2));
        // auxController.y().onTrue(elevator.runToElevatorPosition(ElevatorPosition.L3));
        auxController.leftBumper().onTrue(elevator.runEffector(-4, 4));
        auxController.leftBumper().onFalse(elevator.runEffector(0, 0));
        auxController.leftTrigger().onTrue(elevator.runEffector(-8, 4));
        auxController.leftTrigger().onFalse(elevator.runEffector(0, 0));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
