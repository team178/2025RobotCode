// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this projec
package frc.robot.subsystems.swerve;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;
public class SDSModuleIOSim implements SDSModuleIO {
    private final SparkMax turnMotor;
    private final SparkMax driveMotor;

    private final SparkMaxSim turnMotorSim;
    private final SparkMaxSim driveMotorSim;

    private final SparkAbsoluteEncoderSim turnEncoderSim;
    private final SparkRelativeEncoderSim driveEncoderSim;

    private final SparkClosedLoopController turnController;
    private final SparkClosedLoopController driveController;

    private final DCMotorSim turnSim;
    private final DCMotorSim driveSim;

    public SDSModuleIOSim(int module) {
        turnMotor = new SparkMax(
            switch(module) {
                case 0 -> SwerveConstants.kFLTurnCANID;
                case 1 -> SwerveConstants.kFRTurnCANID;
                case 2 -> SwerveConstants.kBLTurnCANID;
                case 3 -> SwerveConstants.kBRTurnCANID;
                default -> 0;
            },
            MotorType.kBrushless);
        driveMotor = new SparkMax(
            switch(module) {
                case 0 -> SwerveConstants.kFLDriveCANID;
                case 1 -> SwerveConstants.kFRDriveCANID;
                case 2 -> SwerveConstants.kBLDriveCANID;
                case 3 -> SwerveConstants.kBRDriveCANID;
                default -> 0;
            },
            MotorType.kBrushless
        );
        
        SparkMaxConfig turnConfig = SwerveModuleConstants.turnConfig;
        SparkMaxConfig driveConfig = SwerveModuleConstants.driveConfig;

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        updateControlConstants();

        turnMotorSim = new SparkMaxSim(turnMotor, DCMotor.getNEO(1));
        driveMotorSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));

        turnEncoderSim = turnMotorSim.getAbsoluteEncoderSim();
        driveEncoderSim = driveMotorSim.getRelativeEncoderSim();
        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        turnEncoderSim.setZeroOffset(0);

        // TODO: Find the actual moment of inertia of the motor (incl. gearing I believe)
        turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 12.8*100),
            DCMotor.getNEO(1));
        driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 6.12*100),
            DCMotor.getNEO(1));
    }
   
    @Override
    public void updateInputs(SDSModuleIOInputs inputs) {
        turnMotorSim.iterate(turnSim.getAngularVelocityRadPerSec(), 12, 0.02);
        driveMotorSim.iterate(driveSim.getAngularVelocityRadPerSec(), 12, 0.02);
        
        turnSim.setInputVoltage(turnMotorSim.getAppliedOutput() * turnMotorSim.getBusVoltage());
        driveSim.setInputVoltage(driveMotorSim.getAppliedOutput() * driveMotorSim.getBusVoltage());

        turnSim.update(0.02);
        driveSim.update(0.02);

        inputs.turnPosition = new Rotation2d(turnEncoderSim.getPosition());
        inputs.turnVelocityRadPerSec = turnEncoderSim.getVelocity();
        inputs.turnAppliedVolts = turnMotorSim.getAppliedOutput() * turnMotorSim.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent()     ;
        inputs.drivePositionRad = driveEncoderSim.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoderSim.getVelocity();
        inputs.driveVelocityWheelMetersPerSec = inputs.driveVelocityRadPerSec * SwerveConstants.kWheelRadiusMeters;
        inputs.driveAppliedVolts = driveMotorSim.getAppliedOutput() * driveMotorSim.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
    }
   
    @Override
    public void setTurnPosition(Rotation2d position) {
        double setpoint = MathUtil.inputModulus(position.getRadians(), 0, 2 * Math.PI);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
   
    @Override
    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        double ffVolts = Math.signum(velocityRadPerSec) * SwerveModuleConstants.driveControlConstants.kS();
        driveController.setReference(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }
   
    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }
   
    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }
   
    @Override
    public void updateControlConstants() { // don't spam run
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        turnConfig.closedLoop
            .p(SwerveModuleConstants.turnControlConstants.kP())
            .i(SwerveModuleConstants.turnControlConstants.kI())
            .d(SwerveModuleConstants.turnControlConstants.kD())
        ; driveConfig.closedLoop
            .p(SwerveModuleConstants.driveControlConstants.kP())
            .i(SwerveModuleConstants.driveControlConstants.kI())
            .d(SwerveModuleConstants.driveControlConstants.kD())
            .velocityFF(SwerveModuleConstants.driveControlConstants.kV())
        ;
       
        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}






// // Copyright 2021-2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot.subsystems.swerve;

// import static frc.robot.Constants.*;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// /** Physics sim implementation of module IO. */
// public class SDSModuleIOSim implements SDSModuleIO {
//   private final DCMotorSim driveSim;
//   private final DCMotorSim turnSim;

//   private static final double driveSimP = 0.05;
//   private static final double driveSimD = 0.0;
//   private static final double turnSimP = 8.0;
//   private static final double turnSimD = 0.0;

//   private boolean driveClosedLoop = false;
//   private boolean turnClosedLoop = false;
//   private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
//   private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
//   private double driveFFVolts = 0.0;
//   private double driveAppliedVolts = 0.0;
//   private double turnAppliedVolts = 0.0;

//   private static final DCMotor driveGearbox = DCMotor.getNEO(1);
//   private static final DCMotor turnGearbox = DCMotor.getNEO(1);
//   private static final double driveMotorReduction = 6.12;
//   private static final double turnMotorReduction = 12.8;

//   public SDSModuleIOSim() {
//     // Create drive and turn sim models
//     driveSim =
//         new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
//             driveGearbox);
//     turnSim =
//         new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
//             turnGearbox);

//     // Enable wrapping for turn PID
//     turnController.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   @Override
//   public void updateInputs(SDSModuleIOInputs inputs) {
//     // Run closed-loop control
//     if (driveClosedLoop) {
//       driveAppliedVolts =
//           driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
//     } else {
//       driveController.reset();
//     }
//     if (turnClosedLoop) {
//       turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
//     } else {
//       turnController.reset();
//     }

//     // Update simulation state
//     driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
//     turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
//     driveSim.update(0.02);
//     turnSim.update(0.02);

//     // Update drive inputs
//     //inputs.driveConnected = true;
//     inputs.drivePositionRad = driveSim.getAngularPositionRad();
//     inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
//     inputs.driveAppliedVolts = driveAppliedVolts;
//     inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

//     // Update turn inputs
//     //inputs.turnConnected = true;
//     inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
//     inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
//     inputs.turnAppliedVolts = turnAppliedVolts;
//     inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

//     // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
//     /*inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
//     inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
//     inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};*/
//   }

//   @Override
//   public void setDriveOpenLoop(double output) {
//     driveClosedLoop = false;
//     driveAppliedVolts = output;
//   }

//   @Override
//   public void setTurnOpenLoop(double output) {
//     turnClosedLoop = false;
//     turnAppliedVolts = output;
//   }

// //   @Override
// //   public void setDriveVelocity(double velocityRadPerSec) {
// //     driveClosedLoop = true;
// //     driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
// //     driveController.setSetpoint(velocityRadPerSec);
// //   }

//   @Override
//   public void setTurnPosition(Rotation2d rotation) {
//     turnClosedLoop = true;
//     turnController.setSetpoint(rotation.getRadians());
//   }
// }