package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberIOInputs;

    public Climber(ClimberIO io) {
        climberIO = io;
        climberIOInputs = new ClimberIOInputsAutoLogged();
    }

    public Command runSetClimberVolts(double volts) {
        return runOnce(() -> {
            climberIO.setClimberVolts(volts);
        });
    }

    public Command runSetClimberVoltsWithLimit(double volts) {
        return runOnce(() -> {
            if(volts > 0 && climberIOInputs.encoderPosition > ClimberConstants.kMaxEncoderPosition ||
                volts < 0 && climberIOInputs.encoderPosition > ClimberConstants.kMinEncoderPosition) return;
            climberIO.setClimberVolts(volts);
        });
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberIOInputs);
        Logger.processInputs("Climber", climberIOInputs);

        // periodic with limiting movement
    }
}
