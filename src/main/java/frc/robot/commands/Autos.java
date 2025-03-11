// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.swerve.SwerveDrive;

/** 
 * Utility class for generating autonomous routines.
 */
public class Autos {
    private static final SendableChooser<StartingPositions> startingPositionChooser =  new SendableChooser<>();
    private static GenericEntry autoStringFormEntry;

    private static final double preemptiveElevatorInterval = 0.2; // seconds before reaching target for raising elevator (currently no delay implemented before preemptive intake)
    private static final double elevatorAllowMovementTolerance = 0.1; // meters above home tolerated before allowing movement

    // all trajectories should be: (S-AA through S-AL, with S-AX through S-GL, then AY through LZ)
    // 12 * 7 + 2 * 2 * 12
    private static final HashMap<String, Trajectory<SwerveSample>> blueTrajectories = new HashMap<>();

    private enum StartingPositions { // left to right
        ALLYCAGELEFT(      "Ally Cage Left",       "S_A", 7.266),
        ALLYCAGECENTER(    "Ally Cage Center",     "S_B", 6.171),
        ALLYCAGERIGHT(     "Ally Cage Right",      "S_C", 5.075),
        CENTER(            "Center",               "S_D", FieldConstants.fieldHeight / 2), // 4.021
        OPPOSINGCAGELEFT(  "Opposing Cage Left",   "S_E", 3.002),
        OPPOSINGCAGECENTER("Opposing Cage Center", "S_F", 1.906),
        OPPOSINGCAGERIGHT( "Opposing Cage Right",  "S_G", 0.811),
        ;

        public final String name;
        public final String fileKey;
        public final double y;

        // starting robot with center of robot on starting line, centered with the cage
        public static final double blueX = 7.582;
        public static final double redX = FieldConstants.fieldWidth - blueX; // 9.966

        private StartingPositions(String name, String fileKey, double y) {
            this.name = name;
            this.fileKey = fileKey;
            this.y = y;
        }
    }

    private enum AutoPositions { 
        // A is close left reef, then go CCW
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
        L,
        
        // (from the perspective of driver station)
        Y, // left coral station
        Z, // right coral station
        ;
    }

    static {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous ");
        autoStringFormEntry = tab.add("Auto String", "")
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();

        for(StartingPositions position : StartingPositions.values()) {
            if(position.equals(StartingPositions.CENTER)) {
                startingPositionChooser.setDefaultOption(position.name, position);
            } else {
                startingPositionChooser.addOption(position.name, position);
            }
        }
        tab.add("Starting Position", startingPositionChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withPosition(0, 0)
            .withSize(7, 1);

        loadTrajectories();
    }

    /**
     * Loads all possible auto trajectories and stores them in a HashMap for later reference.
     */
    private static void loadTrajectories() {
        // S-AA through S-GL
        for(StartingPositions startingPosition : StartingPositions.values()) {
            for(AutoPositions reefPosition : AutoPositions.values()) {
                if(reefPosition.equals(AutoPositions.Y) || reefPosition.equals(AutoPositions.Z)) continue;

                String trajectoryName = startingPosition.fileKey + reefPosition.name();
                Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);

                if(trajectory.isPresent()) {
                    blueTrajectories.put(trajectoryName, trajectory.get());
                } else {
                    System.out.println("Trajectory " + trajectoryName + " not found");
                }
            }
        }
        
        // AY through LZ, with calculated YA through ZL
        for(AutoPositions reefPosition : AutoPositions.values()) {
            if(reefPosition.equals(AutoPositions.Y) || reefPosition.equals(AutoPositions.Z)) continue;
            
            String trajectoryNameY = reefPosition.name() + "Y";
            Optional<Trajectory<SwerveSample>> trajectoryY = Choreo.loadTrajectory(trajectoryNameY);
            
            if(trajectoryY.isPresent()) {
                blueTrajectories.put(trajectoryNameY, trajectoryY.get());
                blueTrajectories.put("Y" + reefPosition.name(), getReversedTrajectory(trajectoryY.get()));
            } else {
                System.out.println("Trajectory " + trajectoryNameY + " not found");
            }

            String trajectoryNameZ = reefPosition.name() + "Z";
            Optional<Trajectory<SwerveSample>> trajectoryZ = Choreo.loadTrajectory(trajectoryNameZ);
            
            if(trajectoryZ.isPresent()) {
                blueTrajectories.put(trajectoryNameZ, trajectoryZ.get());
                blueTrajectories.put("Z" + reefPosition.name(), getReversedTrajectory(trajectoryZ.get()));
            } else {
                System.out.println("Trajectory " + trajectoryNameZ + " not found");
            }
        }
    }

    /**
     * Selects correct trajectories using responses from Shuffleboard, generates Commands for each leg of auto, and combines them in a SequenceCommandGroup.
     * @param swerve Robot's swerve drive subystem instance.
     * @param elevator Robot's elevator subsystem instance.
     * @return SequenceCommandGroup representing autonomous generated based on Shuffleboard selections.
     */
    public static Command getAutoCommand(SwerveDrive swerve, Elevator elevator) {
        Command resetPoseCommand = Commands.run(() -> swerve.setPose(new Pose2d(
            Constants.isRed() ? StartingPositions.redX : StartingPositions.blueX,
            startingPositionChooser.getSelected().y,
            Constants.isRed() ? Rotation2d.kZero : Rotation2d.k180deg)
        ), swerve);

        String rawAutoString = autoStringFormEntry.getString("");
        if(rawAutoString.length() == 0) {
            return resetPoseCommand;
        }

        Trajectory<SwerveSample> startingTrajectory = blueTrajectories.get(startingPositionChooser.getSelected().fileKey + rawAutoString.substring(0, 1));
        if(startingTrajectory == null) return resetPoseCommand.andThen(Commands.print("Starting trajectory not found"));
        Command autoCommand = new FollowTrajectory(swerve, Constants.isRed() ? getRedTrajectory(startingTrajectory) : startingTrajectory);
        String previousPosition = rawAutoString.substring(0, 1);

        for(int i = 1; i < rawAutoString.length(); i++) {
            String character = rawAutoString.substring(i, i + 1);
            switch(character) {
                case "0":
                    autoCommand = autoCommand
                        .andThen(elevator.runIntakeFromCoralStation());
                    System.out.println("That's a weird auto routine... index " + i);
                    break;
                case "1":
                    autoCommand = autoCommand
                        .andThen(elevator.runScoreToElevatorPosition(ElevatorPosition.L1))
                        .andThen(elevator.runWaitUntilSafeToMove(elevatorAllowMovementTolerance));
                    System.out.println("That's a weird auto routine... index " + i);
                    break;
                case "2":
                    autoCommand = autoCommand
                        .andThen(elevator.runScoreToElevatorPosition(ElevatorPosition.L2))
                        .andThen(elevator.runWaitUntilSafeToMove(elevatorAllowMovementTolerance));
                    System.out.println("That's a weird auto routine... index " + i);
                    break;
                case "3":
                    autoCommand = autoCommand
                        .andThen(elevator.runScoreToElevatorPosition(ElevatorPosition.L3))
                        .andThen(elevator.runWaitUntilSafeToMove(elevatorAllowMovementTolerance));
                    System.out.println("That's a weird auto routine... index " + i);
                    break;
                case "A":
                case "B":
                case "C":
                case "D":
                case "E":
                case "F":
                case "G":
                case "H":
                case "I":
                case "J":
                case "K":
                case "L":
                    Trajectory<SwerveSample> trajectory = blueTrajectories.get(previousPosition + character);
                    if(trajectory == null) return resetPoseCommand.andThen(Commands.print("Trajectory not found going to index " + i + ", trajectory " + previousPosition + character));
                    Command followTrajectory = new FollowTrajectory(swerve, Constants.isRed() ? getRedTrajectory(trajectory) : trajectory);
                    String next = i < rawAutoString.length() + 1 ? rawAutoString.substring(i + 1, i + 2) : "";
                    switch(next) {
                        case "0": // no delay with preemptive intake
                            autoCommand = autoCommand
                                .andThen(followTrajectory.alongWith(elevator.runIntakeFromCoralStation()));
                            i++;
                            break;
                        case "1":
                            autoCommand = autoCommand
                                .andThen(followTrajectory.alongWith(
                                    new WaitCommand(Math.max(trajectory.getTotalTime() - preemptiveElevatorInterval, 0))
                                        .andThen(elevator.runScoreToElevatorPosition(ElevatorPosition.L1))
                                )).andThen(elevator.runWaitUntilSafeToMove(elevatorAllowMovementTolerance));
                            i++;
                            break;
                        case "2":
                            autoCommand = autoCommand
                                .andThen(followTrajectory.alongWith(
                                    new WaitCommand(Math.max(trajectory.getTotalTime() - preemptiveElevatorInterval, 0))
                                        .andThen(elevator.runScoreToElevatorPosition(ElevatorPosition.L2))
                                )).andThen(elevator.runWaitUntilSafeToMove(elevatorAllowMovementTolerance));
                            i++;
                            break;
                        case "3":
                            autoCommand = autoCommand
                                .andThen(followTrajectory.alongWith(
                                    new WaitCommand(Math.max(trajectory.getTotalTime() - preemptiveElevatorInterval, 0))
                                        .andThen(elevator.runScoreToElevatorPosition(ElevatorPosition.L3))
                                )).andThen(elevator.runWaitUntilSafeToMove(elevatorAllowMovementTolerance));
                            i++;
                            break;
                        default:
                            autoCommand = autoCommand.andThen(followTrajectory);
                            break;
                    }
                    previousPosition = character;
                    break;
                default:
                    return resetPoseCommand.andThen(Commands.print("Invalid character in auto string: " + character));
            }
        }

        return resetPoseCommand.andThen(autoCommand);
    }

    public static Trajectory<SwerveSample> getRedTrajectory(Trajectory<SwerveSample> blueTrajectory) {
        List<SwerveSample> redSwerveSamples = blueTrajectory.samples().stream().map(sample -> {
            Pose2d pose = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));
            pose = pose.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
            double[] fx = Arrays.stream(sample.moduleForcesX()).map(force -> -force).toArray();
            double[] fy = Arrays.stream(sample.moduleForcesY()).map(force -> -force).toArray();
            return new SwerveSample(
                sample.t,
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians(),
                -sample.vx,
                -sample.vy,
                sample.omega,
                -sample.ax,
                -sample.ay,
                sample.alpha,
                fx,
                fy
            );
        }).toList();

        return new Trajectory<>(
            blueTrajectory.name() + "Red",
            redSwerveSamples,
            blueTrajectory.splits(),
            blueTrajectory.events()
        );
    }

    public static Trajectory<SwerveSample> getReversedTrajectory(Trajectory<SwerveSample> trajectory) {
        double trajectoryDuration = trajectory.getFinalSample(false).isPresent() ? trajectory.getFinalSample(false).get().t : 0;
        List<SwerveSample> reversedSwerveSamples = trajectory.samples().stream().map(sample -> {
            double[] fx = Arrays.stream(sample.moduleForcesX()).map(force -> -force).toArray();
            double[] fy = Arrays.stream(sample.moduleForcesY()).map(force -> -force).toArray();
            return new SwerveSample(
                trajectoryDuration - sample.t,
                sample.x,
                sample.y,
                sample.heading,
                -sample.vx,
                -sample.vy,
                -sample.omega,
                -sample.ax,
                -sample.ay,
                -sample.alpha,
                fx,
                fy
            );
        }).toList();
        Collections.reverse(new LinkedList<>(reversedSwerveSamples));

        return new Trajectory<>(
            trajectory.name() + "Reversed",
            reversedSwerveSamples,
            List.of(0),
            List.of()
        );
    }
}
