package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    private ClimberIO climber;

    public Climber(ClimberIO climberIO) {
        Preferences.initDouble("volts",0);
        Preferences.initDouble("kG", 0);
        Preferences.initDouble("P",0.1);
        this.climber = climberIO;  
    }

    public Command runClimber(BooleanSupplier set, BooleanSupplier up, BooleanSupplier down, BooleanSupplier key) {
        // return runOnce(() -> climberIO.setClimberVolts(low.getAsBoolean(), high.getAsBoolean()));
        boolean bKey = key.getAsBoolean();
        boolean bSet = set.getAsBoolean() && bKey;
        boolean bUp = up.getAsBoolean() && bKey;
        boolean bDown = down.getAsBoolean() && bKey;
        
        return run(() -> climber.climberInputs(bSet, bUp, bDown));
    }

    public Command runUp() {
        return runOnce( () -> climber.setVolts(Preferences.getDouble("volts",0)));
    }

    public Command runDown() {
        return runOnce(() -> climber.setVolts(-Preferences.getDouble("volts",0)));
    }

    public Command stop() {
        return runOnce(() -> climber.setVolts(0));
    }

    public Command gravity() {
        return runOnce(() -> climber.setVolts(Preferences.getDouble("kG",0)));
    }

    public Command pid() {
        return runOnce(() -> climber.setReference(Preferences.getDouble("P", 0)));
    }
    
    @Override
    public void periodic() {
        climber.update();
        SmartDashboard.putNumber("encoder position", climber.getPos());

    }
}
