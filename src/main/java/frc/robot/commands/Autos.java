// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.swerve.SwerveDrive;

/** 
 * Utility class for generating autonomous routines.
 */
public class Autos {
    // private static final SendableChooser<AutoCommand> autoChooser =  new SendableChooser<>();
    private static final SendableChooser<StartingPositions> startingPositionChooser =  new SendableChooser<>();
    private static final SendableChooser<ReefPositions> firstReefPositionChooser = new SendableChooser<>();
    private static final SendableChooser<ReefPositions> secondReefPositionChooser = new SendableChooser<>();

    private static final HashMap<String, Trajectory<SwerveSample>> trajectories = new HashMap<>();

    private enum StartingPositions {
        Left,
        Right,
        Center;
    }

    private enum ReefPositions  {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;
    }

    /**
     * Sets up options for SendableChoosers in Shuffleboard's "Auto" tab. Also calls loadTrajectories() to pre-load trajectories before auto begins.
     * @param swerve Robot's swerve drive subsystem instance.
     * @param elevator Robot's elevator subsystem instance.
     */
    public static void initAutos(SwerveDrive swerve, Elevator elevator) {
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        
        // Shuffleboard.getTab("Auto")
        //     .add("Auto", autoChooser)
        //     .withWidget(BuiltInWidgets.kSplitButtonChooser)
        //     .withSize(9, 1);

        tab.add("Starting Position", startingPositionChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(9, 1);
        for (StartingPositions position : StartingPositions.values()) {
            if (position.equals(StartingPositions.Left)) {
                startingPositionChooser.setDefaultOption(position.toString(), position);
            } else {
                startingPositionChooser.addOption(position.toString(), position);
            }
        }

        tab.add("Leg 1 Reef Position", firstReefPositionChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(9, 1);
        for (ReefPositions position : ReefPositions.values()) {
            if (position.equals(ReefPositions.A)) {
                firstReefPositionChooser.setDefaultOption(position.toString(), position);
            } else {
                firstReefPositionChooser.addOption(position.toString(), position);
            }
        }

        tab.add("Leg 2 Reef Position", secondReefPositionChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(9, 1);
        for (ReefPositions position : ReefPositions.values()) {
            if (position.equals(ReefPositions.A)) {
                secondReefPositionChooser.setDefaultOption(position.toString(), position);
            } else {
                secondReefPositionChooser.addOption(position.toString(), position);
            }
        }

        loadTrajectories();
    }

    /**
     * Loads all possible auto trajectories and stores them in a HashMap for later reference.
     */
    private static void loadTrajectories() {
        // First trajectory (starting position to first reef score)
        for (StartingPositions startingPosition : StartingPositions.values()) {
            for (ReefPositions reefPosition : ReefPositions.values()) {
                String trajectoryName1 = startingPosition.toString() + "To" + reefPosition.toString();
                Optional<Trajectory<SwerveSample>> trajectory1 = Choreo.loadTrajectory(trajectoryName1);

                if (trajectory1.isPresent()) trajectories.put(trajectoryName1, trajectory1.get());
            }
        }
        
        for (ReefPositions reefPosition : ReefPositions.values()) {
            // Second trajectory (first reef score to coral station)
            String trajectoryName2 = reefPosition.toString() + "ToCoralStation";
            Optional<Trajectory<SwerveSample>> trajectory2 = Choreo.loadTrajectory(trajectoryName2);

            if (trajectory2.isPresent()) trajectories.put(trajectoryName2, trajectory2.get());

            // Third trajectory (coral station to second reef score)
            String trajectoryName3 = "CoralStationTo" + reefPosition.toString();
            Optional<Trajectory<SwerveSample>> trajectory3 = Choreo.loadTrajectory(trajectoryName3);

            if (trajectory3.isPresent()) trajectories.put(trajectoryName3, trajectory3.get());
        }
    }

    /**
     * Selects correct trajectories using responses from Shuffleboard, generates Commands for each leg of auto, and combines them in a SequenceCommandGroup.
     * @param swerve Robot's swerve drive subystem instance.
     * @param elevator Robot's elevator subsystem instance.
     * @return SequenceCommandGroup representing autonomous generated based on Shuffleboard selections.
     */
    public static FullAutoCommand getAutoCommand(SwerveDrive swerve, Elevator elevator) {
        String trajectoryName1 = startingPositionChooser.getSelected().toString() + "To" + firstReefPositionChooser.getSelected().toString();
        String trajectoryName2 = firstReefPositionChooser.getSelected().toString() + "ToCoralStation";
        String trajectoryName3 = "CoralStationTo" + secondReefPositionChooser.getSelected().toString();

        if (trajectories.get(trajectoryName1).getInitialPose(false).isPresent()) {
            swerve.setPose(
                trajectories.get(trajectoryName1).getInitialPose(false).get()
            );
        }

        return (FullAutoCommand) runAutoToReef(trajectoryName1, ElevatorPosition.L3, swerve, elevator)
            .andThen(runAutoToCoralStation(trajectoryName2, 2.0, swerve, elevator))
            .andThen(runAutoToReef(trajectoryName3, ElevatorPosition.L3, swerve, elevator));
    }

    /**
     * Generates a command to drive to the reef and score.
     * @param trajectoryName Name of the drive trajectory for swerve base to follow.
     * @param scoringHeight Preferred reef scoring height (L1, L2, or L3).
     * @param swerve Robot's swerve drive subsystem instance.
     * @param elevator Robot's elevatory subystem instance.
     * @return Command to drive to and score on the reef.
     */
    private static AutoSegmentCommand runAutoToReef(String trajectoryName, ElevatorPosition scoringHeight, SwerveDrive swerve, Elevator elevator) {
        Trajectory<SwerveSample> trajectory = trajectories.get(trajectoryName);
        double trajectoryTime = trajectory.getTotalTime();
        
        return (AutoSegmentCommand) swerve.runFollowTrajectory(trajectory)
            .alongWith(
                new WaitCommand(trajectoryTime - 0.5) // TUNE THIS NUMBER TO FIND THE BEST TIME TO RAISE THE ELEVATOR BEFORE REACHING THE REEF
                .andThen(elevator.runScoreToElevatorPosition(scoringHeight))
            );
    }

    /**
     * Generates a command to drive to the coral station and intake.
     * @param trajectoryName Name of the drive trajectory for swerve base to follow.
     * @param coralStationLingerTime Amount of time to linger at the Coral Station in seconds.
     * @param swerve Robot's swerve drive subsystem instance.
     * @param elevator Robot's elevator subsystem instance.
     * @return Command to drive to and intake on the reef.
     */
    private static AutoSegmentCommand runAutoToCoralStation(String trajectoryName, double coralStationLingerTime, SwerveDrive swerve, Elevator elevator) {
        Trajectory<SwerveSample> trajectory = trajectories.get(trajectoryName);
        double trajectoryTime = trajectory.getTotalTime();
        
        return (AutoSegmentCommand) swerve.runFollowTrajectory(trajectory)
            .alongWith(
                new WaitCommand(trajectoryTime-0.5) // TUNE THIS NUMBER TO FIND THE BEST TIME TO START INTAKING BEFORE REACHING THE CORAL STATION
                .andThen(elevator.runIntakeFromCoralStation())
            );
    }
}
