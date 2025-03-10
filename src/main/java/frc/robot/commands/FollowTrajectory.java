package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
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
        return timer.get() > trajectory.getTotalTime();
    }
}
