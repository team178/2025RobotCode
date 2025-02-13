package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private ManipulatorIO manipulatorIO;

    public Manipulator(ManipulatorIO io) {
        manipulatorIO = io;
    }

    public Command setManipulatorPosition(ManipulatorPosition position) {
        return runOnce(() -> {
            manipulatorIO.setManipulatorPosition(position);
        });
    }
}
