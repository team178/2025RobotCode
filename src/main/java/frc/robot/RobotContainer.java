// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.Pigeon2IO;
import frc.robot.subsystems.swerve.SDSModuleIO;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private CommandXboxController driverController;
    private CommandXboxController auxController;

    private SwerveDrive swerve;
    private Elevator elevator;

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
                // elevator = new Elevator(new ElevatorIOSpark());
                break;
            default:
                swerve = new SwerveDrive(
                    new GyroIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {},
                    new SDSModuleIO() {});
                // elevator = new Elevator(new ElevatorIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX,
            driverController::getRightTriggerAxis,
            driverController.leftBumper()::getAsBoolean,
            driverController.rightBumper()::getAsBoolean
        ));

        // driverController.a().onTrue(swerve.runTestDrive());
        // driverController.a().onFalse(swerve.runStopDrive());
        // driverController.x().onTrue(swerve.runOpenTestDrive());
        // driverController.x().onFalse(swerve.runStopDrive());
        driverController.b().onTrue(swerve.runUpdateControlConstants());
        driverController.back().onTrue(swerve.runToXPosition(true));
        driverController.y().onTrue(swerve.runZeroGyro());
        driverController.start().onTrue(swerve.runSetTempPose());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
