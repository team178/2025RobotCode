package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    private ClimberIO climberIO;

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;  
    }

    public Command runClimber(BooleanSupplier low, BooleanSupplier high) {
        return runOnce(() -> climberIO.setClimberVolts(low.getAsBoolean(), high.getAsBoolean()));
    }
    
    @Override
    public void periodic() {

    }
}
