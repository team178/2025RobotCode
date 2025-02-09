package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSpark implements ElevatorIO {
    private SparkMax elevatorLeaderMotor;
    private SparkMax elevatorFollowerMotor;
    private SparkMax leftEffectorMotor;
    private SparkMax rightEffectorMotor;
    private SparkMax funnelMotor;

    private SparkAbsoluteEncoder elevatorEncoder;
    private RelativeEncoder leftEffectorEncoder;
    private RelativeEncoder rightEffectorEncoder;
    private RelativeEncoder funnelEncoder;

    private SparkClosedLoopController elevatorController;

    private DigitalInput highLimit;
    private DigitalInput lowLimit;
    private DigitalInput upperPhotosensor;
    private DigitalInput lowerPhotosensor;

    private ElevatorPosition desiredPosition;
    private double leftEffectorDesiredVolts;
    private double rightEffectorDesiredVolts;
    private double funnelDesiredVolts;

    public ElevatorIOSpark() {
        elevatorLeaderMotor = new SparkMax(ElevatorConstants.kElevatorLeaderCANID, MotorType.kBrushless);
        elevatorFollowerMotor = new SparkMax(ElevatorConstants.kElevatorFollowerCANID, MotorType.kBrushless);
        leftEffectorMotor = new SparkMax(ElevatorConstants.kLeftEffectorCANID, MotorType.kBrushless);
        rightEffectorMotor = new SparkMax(ElevatorConstants.kRightEffectorCANID, MotorType.kBrushless);
        funnelMotor = new SparkMax(ElevatorConstants.kFunnelMotorCANID, MotorType.kBrushless);

        elevatorLeaderMotor.setCANTimeout(0);
        elevatorFollowerMotor.setCANTimeout(0);
        leftEffectorMotor.setCANTimeout(0);
        rightEffectorMotor.setCANTimeout(0);
        funnelMotor.setCANTimeout(0);

        elevatorEncoder = elevatorLeaderMotor.getAbsoluteEncoder();
        leftEffectorEncoder = leftEffectorMotor.getEncoder();
        rightEffectorEncoder = rightEffectorMotor.getEncoder();
        funnelEncoder = funnelMotor.getEncoder();

        elevatorController = elevatorLeaderMotor.getClosedLoopController();

        elevatorLeaderMotor.configure(ElevatorConstants.elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorFollowerMotor.configure(ElevatorConstants.elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftEffectorMotor.configure(ElevatorConstants.effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightEffectorMotor.configure(ElevatorConstants.effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        funnelMotor.configure(ElevatorConstants.effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        highLimit = new DigitalInput(ElevatorConstants.kHighLimitDIO);
        lowLimit = new DigitalInput(ElevatorConstants.kLowLimitDIO);
        upperPhotosensor = new DigitalInput(ElevatorConstants.kUpperPhotosensorDIO);
        lowerPhotosensor = new DigitalInput(ElevatorConstants.kLowerPhotosensorDIO);

        desiredPosition = ElevatorPosition.HOME;
        leftEffectorDesiredVolts = 0;
        rightEffectorDesiredVolts = 0;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.desiredPosition = desiredPosition;
        inputs.leftEffectorDesiredVolts = leftEffectorDesiredVolts;
        inputs.rightEffectorDesiredVolts = rightEffectorDesiredVolts;
        inputs.funnelMotorDesiredVolts = funnelDesiredVolts;

        inputs.elevatorHeight = elevatorEncoder.getPosition();
        inputs.elevatorVelocity = elevatorEncoder.getVelocity();
        inputs.leftEffectorVelocity = leftEffectorEncoder.getVelocity();
        inputs.rightEffectorVelocity = rightEffectorEncoder.getVelocity();
        inputs.funnelMotorVelocity = funnelEncoder.getVelocity();
        
        inputs.elevatorAppliedVolts = elevatorLeaderMotor.getAppliedOutput() * elevatorLeaderMotor.getBusVoltage();
        inputs.elevatorCurrentAmps = elevatorLeaderMotor.getOutputCurrent();
        inputs.leftEffectorAppliedVolts = leftEffectorMotor.getAppliedOutput() * leftEffectorMotor.getBusVoltage();
        inputs.rightEffectorAppliedVolts = rightEffectorMotor.getAppliedOutput() * rightEffectorMotor.getBusVoltage();
        inputs.leftEffectorCurrentAmps = leftEffectorMotor.getOutputCurrent();
        inputs.rightEffectorCurrentAmps = rightEffectorMotor.getOutputCurrent();
        inputs.funnelMotorAppliedVolts = funnelMotor.getAppliedOutput() * funnelMotor.getBusVoltage();
        inputs.funnelMotorCurrentAmps = funnelMotor.getOutputCurrent();

        inputs.highLimit = highLimit.get();
        inputs.lowLimit = lowLimit.get();
        inputs.upperPhotosensor = upperPhotosensor.get();
        inputs.lowerPhotosensor = lowerPhotosensor.get();
    }

    @Override
    public void setElevatorPosition(ElevatorPosition position) {
        desiredPosition = position;
        double ffVolts = ElevatorConstants.elevatorControlConstants.kG();
        elevatorController.setReference(
            position.height, ControlType.kPosition,
            (lowLimit.get()) ? ClosedLoopSlot.kSlot1 : ((highLimit.get()) ? ClosedLoopSlot.kSlot2 : ClosedLoopSlot.kSlot0),
            ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setElevatorOpenLoop(double volts) {
        elevatorLeaderMotor.setVoltage(volts);
    }
    
    @Override
    public void setLeftEffectorVolts(double volts) {
        leftEffectorDesiredVolts = volts;
        leftEffectorMotor.setVoltage(volts);
    }
    
    @Override
    public void setRightEffectorVolts(double volts) {
        rightEffectorDesiredVolts = volts;
        rightEffectorMotor.setVoltage(volts);
    }
    
    @Override
    public void setEffectorVolts(double left, double right) {
        setLeftEffectorVolts(left);
        setRightEffectorVolts(right);
    }

    @Override
    public void setFunnelMotorVolts(double volts) {
        funnelDesiredVolts = volts;
        funnelMotor.setVoltage(volts);
    }
}
