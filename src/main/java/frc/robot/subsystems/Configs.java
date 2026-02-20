// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
// the api has changed
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class Configs extends SubsystemBase {
  public static final class SwerveModule {
        // Make these public so they can be accessed from SwerveModule.java
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig drivingInvertedConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        public static void setDriveMotorSettings(SparkMaxConfig driveConfig, boolean inverted) {
            driveConfig
                .inverted(inverted ? true: false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)  // Increased from 40A to 60A for more power
                .voltageCompensation(12.0)
                .openLoopRampRate(0.1)   // Reduced ramp rate for faster response
                .closedLoopRampRate(0.1);
            driveConfig.encoder
                .positionConversionFactor(ModuleConstants.ROTATIONS_TO_METERS)
                .velocityConversionFactor(ModuleConstants.RPM_TO_MPS); // rotations per minute to MPS
            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0, 0.02)  // Increased P from 0.04 to 0.1, D from 0.01 to 0.02
                .velocityFF(ModuleConstants.DRIVE_VELOCITY_FEEDFOWARD)
                .outputRange(-1, 1);
        }
        public static void setTurningMotorSettings(SparkMaxConfig turnConfig) {
            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)  // Increased from 20A to 30A for stronger steering
                .voltageCompensation(12.0);
            turnConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(ModuleConstants.ROTATIONS_TO_RADIANS)
                .velocityConversionFactor(ModuleConstants.RPM_TO_RADPS);
            turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1.0, 0, 0.03)  // Increased P from 0.7 to 1.0, D from 0.02 to 0.03
                .velocityFF(0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, ModuleConstants.ROTATIONS_TO_RADIANS);
        }
        static {
            setDriveMotorSettings(drivingConfig, false);
            setDriveMotorSettings(drivingInvertedConfig, true);
            setTurningMotorSettings(turningConfig);
        }
    }

  /** Creates a new Configs. */
  //public Configs() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
