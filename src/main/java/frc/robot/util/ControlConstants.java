package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;

public class ControlConstants {
    public static final NetworkTable constantPreferences = NetworkTableInstance.getDefault().getTable("Custom Robot Preferences");
    public static boolean enableNT = true;

    public final String name;

    // PID
    public final double kDefaultP;
    public final double kDefaultI;
    public final double kDefaultD;

    // FF
    public final double kDefaultS; //kS * sign(theta)
    public final double kDefaultG; //kG * cos(theta)
    public final double kDefaultV; //kV * dtheta/dt

    public ControlConstants(String name, double kDefaultP, double kDefaultI, double kDefaultD, double kDefaultV, double kDefaultG, double kDefaultS) {
        this.name = name;
        this.kDefaultP = kDefaultP;
        this.kDefaultI = kDefaultI;
        this.kDefaultD = kDefaultD;
        this.kDefaultS = kDefaultS;
        this.kDefaultG = kDefaultG;
        this.kDefaultV = kDefaultV;
        
        if(!enableNT) return;
        Preferences.initDouble(name + "/kP", this.kDefaultP);
        Preferences.initDouble(name + "/kI", this.kDefaultI);
        Preferences.initDouble(name + "/kD", this.kDefaultD);
        Preferences.initDouble(name + "/kS", this.kDefaultS);
        Preferences.initDouble(name + "/kG", this.kDefaultG);
        Preferences.initDouble(name + "/kV", this.kDefaultV);

        System.out.println(name + " preferences have been initialized");
    }

    public void resetPreferences() {
        if(!enableNT) return;
        Preferences.initDouble(name + "/kP", this.kDefaultP);
        Preferences.initDouble(name + "/kI", this.kDefaultI);
        Preferences.initDouble(name + "/kD", this.kDefaultD);
        Preferences.initDouble(name + "/kS", this.kDefaultS);
        Preferences.initDouble(name + "/kG", this.kDefaultG);
        Preferences.initDouble(name + "/kV", this.kDefaultV);
    }

    public double kP() {
        if(!enableNT) return kDefaultP;
        return Preferences.getDouble(name + "/kP", kDefaultP);
    }

    public double kI() {
        if(!enableNT) return kDefaultI;
        return Preferences.getDouble(name + "/kI", kDefaultI);
    }

    public double kD() {
        if(!enableNT) return kDefaultD;
        return Preferences.getDouble(name + "/kD", kDefaultD);
    }

    public double kS() {
        if(!enableNT) return kDefaultS;
        return Preferences.getDouble(name + "/kS", kDefaultS);
    }

    public double kG() {
        if(!enableNT) return kDefaultG;
        return Preferences.getDouble(name + "/kG", kDefaultV);
    }

    public double kV() {
        if(!enableNT) return kDefaultV;
        return Preferences.getDouble(name + "/kV", kDefaultV);
    }
}
