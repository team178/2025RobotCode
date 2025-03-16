package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FollowTrajectory extends Command {
    private Timer timer;
    private SwerveDrive swerveDrive;
    private Trajectory<SwerveSample> trajectory;
    
    public FollowTrajectory(SwerveDrive swerveDrive, Trajectory<SwerveSample> trajectory) {
        timer = new Timer();
        this.swerveDrive = swerveDrive;
        this.trajectory = Constants.isRed() ? Autos.getRedTrajectory(trajectory) : trajectory;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        Optional<SwerveSample> optionalSample = trajectory.sampleAt(timer.get(), false);
        if(optionalSample.isPresent()) swerveDrive.followSwerveSample(optionalSample.get());
        else swerveDrive.runChassisSpeeds(new ChassisSpeeds(0, 0, 0), true, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.runChassisSpeeds(new ChassisSpeeds(0, 0, 0), true, true);
    }

    @Override
    public boolean isFinished() {
        // Optional<SwerveSample> finalSample = trajectory.getFinalSample(false);
        // if(finalSample.isPresent()) {
        //     return (timer.get() > trajectory.getTotalTime() + 5 ||
        //         (timer.get() > trajectory.getTotalTime() + 1 &&
        //         Math.abs(finalSample.get().x - swerveDrive.getPose().getX()) < 0.05 &&
        //         Math.abs(finalSample.get().y - swerveDrive.getPose().getY()) < 0.05 &&
        //         Math.abs(finalSample.get().heading - swerveDrive.getPose().getRotation().getRadians()) < 0.1));
        // }
        return timer.get() > trajectory.getTotalTime() + 2;
    }
}
