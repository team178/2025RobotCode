package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorIOSpark implements ManipulatorIO {
    private SparkMax deployMotor;
    private SparkMax rollerMotor;

    private SparkAbsoluteEncoder deployEncoder;
    private SparkClosedLoopController deployController;

    private ManipulatorPosition desiredPosition;
    private double desiredDeployVoltage;
    private double desiredRollerVoltage;

    public ManipulatorIOSpark() {
        deployMotor = new SparkMax(ManipulatorConstants.kDeployMotorCANID, MotorType.kBrushless);
        rollerMotor = new SparkMax(ManipulatorConstants.kRollerMotorCANID, MotorType.kBrushless);

        deployEncoder = deployMotor.getAbsoluteEncoder();
        deployController = deployMotor.getClosedLoopController();

        deployMotor.setCANTimeout(0);
        rollerMotor.setCANTimeout(0);

        deployMotor.configure(ManipulatorConstants.deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double applyDeployAngleOffset(double angleInRadians) {
        return angleInRadians + ManipulatorConstants.kDeployEncoderOffset;
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.desiredPosition = desiredPosition;
        inputs.desiredDeployVoltage = desiredDeployVoltage;
        inputs.desiredRollerVoltage = desiredRollerVoltage;

        inputs.manipulatorAngle = deployEncoder.getPosition();
        inputs.manipulatorVelocity = deployEncoder.getVelocity();
    }

    @Override
    public void setManipulatorPosition(ManipulatorPosition position) {
        desiredPosition = position;
        double ffVolts = ManipulatorConstants.deployControlConstants.kG() * Math.cos(position.angle);
        deployController.setReference(position.angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setDeployOpenLoop(double output) {
        desiredDeployVoltage = output;
        deployMotor.setVoltage(output);
    }

    @Override
    public void setRollerVolts(double volts) {
        desiredRollerVoltage = volts;
        rollerMotor.setVoltage(volts);
    }
}