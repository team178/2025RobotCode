package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private ManipulatorIO io;
    private ManipulatorIOInputsAutoLogged inputs;

    public Manipulator(ManipulatorIO io) {
        this.io = io;
    }

    public Command runSetManipulatorPosition(ManipulatorPosition position) {
        return runOnce(() -> {
            io.setManipulatorPosition(position);
        });
    }

    public Command runSetRollerVolts(double volts) {
        return runOnce(() -> {
            io.setRollerVolts(volts);
        });
    }

    public Command runUpdateControlConstants() {
        return runOnce(() -> {
            io.updateControlConstants();
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Manipulator", inputs);
    }
}
