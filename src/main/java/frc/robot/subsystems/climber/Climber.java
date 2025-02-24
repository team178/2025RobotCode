package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private ClimberIO climberIO;

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;  
    }

    public Command toggleClimber(ClimberPosition climberPosition) {
        return runOnce(null);
    }

    public Command runVolts(double volts) {
        return runOnce(() -> climberIO.setVolts(volts));
    }

    @Override
    public void periodic() {

    }
}
