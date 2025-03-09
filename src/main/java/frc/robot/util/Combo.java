package frc.robot.util;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Combo {
    private String key;
    private double timeout;
    private BooleanSupplier[] comboRequirements;

    private int stage;
    private Timer timer;

    private ComboUpdateMessage comboUpdateMessage;

    private Trigger trigger;

    public Combo(String name, double timeoutSeconds, BooleanSupplier... requirementsSuppliers) {
        timeout = timeoutSeconds;
        comboRequirements = requirementsSuppliers;
        key = "Combos/" + name + comboRequirements.length + "/";
        
        stage = 0;
        timer = new Timer();
        timer.start();

        comboUpdateMessage = ComboUpdateMessage.TIMEOUT;

        trigger = new Trigger(() -> poll());
    }

    private void update() {
        if(stage == comboRequirements.length) {
            if(!comboRequirements[comboRequirements.length - 1].getAsBoolean()) {
                stage = 0;
                timer.reset();
                comboUpdateMessage = ComboUpdateMessage.ENDED;
            }
        } else if(stage == 0) {
            if(comboRequirements[0].getAsBoolean()) {
                stage = 1;
                timer.reset();
                comboUpdateMessage = ComboUpdateMessage.INITIATED;
            }
        } else if(timer.hasElapsed(timeout)) {
            stage = 0;
            comboUpdateMessage = ComboUpdateMessage.TIMEOUT;
        } else if(comboRequirements[stage].getAsBoolean()) {
            stage++;
            timer.reset();
            comboUpdateMessage = stage == comboRequirements.length ? ComboUpdateMessage.TRIGGERED : ComboUpdateMessage.NEXT;
        }
        Logger.recordOutput(key + "stage", stage);
        Logger.recordOutput(key + "message", comboUpdateMessage);
        Logger.recordOutput(key + "triggered", stage == comboRequirements.length);
        Logger.recordOutput(key + "timer", timer.get());
    }

    private boolean poll() {
        update();
        return stage == comboRequirements.length;
    }

    public Trigger getTrigger() {
        return trigger;
    }

    private enum ComboUpdateMessage {
        INITIATED, NEXT, TRIGGERED, TIMEOUT, ENDED;
    }
}
