package frc.robot.subsystems.lights;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LightsConstants;

public class Lights extends SubsystemBase{
    private int stripLength;

    private AddressableLED lights;
    private AddressableLEDBuffer lightsBuffer;
    private LEDPattern pattern;

    private BooleanSupplier isTranslAligned;
    private BooleanSupplier isRotAligned;

    private boolean prevTranslAl;
    private boolean prevRotAl;
    // none - solid alliance color
    // rotation only - alliance color flash T = 0.75 sec
    // position only - solid alternating yellow/blue
    // is aligned - alternating yellow/blue flash T = 0.2 sec

    // top of the lights: override above pattern on the left/right side with green if aiming left/right reef position respectively
    //if fully alligned - green flashes opposite to other flashing

    public Lights(BooleanSupplier isTranslAligned, BooleanSupplier isRotAligned) {
        stripLength = LightsConstants.kLength;
        lights = new AddressableLED(LightsConstants.kPort);
        lightsBuffer = new AddressableLEDBuffer(stripLength);
        setDefColor();

        this.isTranslAligned = isTranslAligned;
        this.isRotAligned = isRotAligned;
        
        prevTranslAl = false;
        prevRotAl = false;
    }

    private int getSignalCode(boolean isTranslAligned, boolean isRotAligned) {
        int rtrn = 0;
        if(isTranslAligned) rtrn += 1;
        if(isRotAligned) rtrn += 2;
        return rtrn;
    }

    private void setDefColor() {
        pattern = Constants.isRed() ?
            LightsConstants.kFRCRedPattern :
            LightsConstants.kFRCBluePattern;
        
        pattern.applyTo(lightsBuffer);
        lights.setData(lightsBuffer);
    }

    private void setTColor() {

    }

    @Override
    public void periodic() {
        // boolean currColorCond = colorCond.getAsBoolean();
        // if(currColorCond != prevColorCond) {
        //     pattern = colorCond.getAsBoolean() ? 
        //         LightsConstants.kTeamYellowPattern : 
        //         LightsConstants.kTeamBluePattern;

        //     pattern.applyTo(lightsBuffer);
        //     lights.setData(lightsBuffer);

        //     prevColorCond = currColorCond;
        // }
    }
}
