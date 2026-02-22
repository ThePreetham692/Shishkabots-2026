// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.Logger;

public class SwerveModule {
  private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    // robot chasis is not angled perfectly with each module
    private double chasisAngularOffset;

    // mark the wanted driveSpeed, and turningPosition
    private double desiredSpeed;
    private double desiredAngle;
    
    // Module identifier
    private String moduleName;

    // motor and simulated versions of the drive and turning motors
    private final DCMotor driveDCMotor;
    private final DCMotor turningDCMotor; 
    private final SparkMaxSim driveMotorSim;
    private final SparkMaxSim turningMotorSim;
  
  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveMotorChannel,
            int turningMotorChannel,
            double angularOffset,
            boolean inverted, String moduleName) {

        driveMotor = new SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        driveClosedLoopController = driveMotor.getClosedLoopController();
        turningClosedLoopController = turningMotor.getClosedLoopController();
        
        // Configure encoders and motors
        driveMotor.configure(inverted ? Configs.SwerveModule.drivingInvertedConfig : Configs.SwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(Configs.SwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        chasisAngularOffset = angularOffset;
        driveEncoder.setPosition(0);
        
        // setup simulated motors
        driveDCMotor = DCMotor.getNEO(1);
        turningDCMotor = DCMotor.getNEO(1);

        driveMotorSim = new SparkMaxSim(driveMotor, driveDCMotor);
        turningMotorSim = new SparkMaxSim(turningMotor, turningDCMotor);

        this.moduleName = moduleName;
  }

public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(getTurningPosition())
        );
    }
    /**
     * Returns the current position of the module
     */
    public SwerveModulePosition getPosition() {
        // apply angular offset to encoder position to get position relative to chasis
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chasisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees
        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        // Store desired values
        desiredSpeed = correctedDesiredState.speedMetersPerSecond;
        desiredAngle = correctedDesiredState.angle.getRadians();

        // Set motor references - drive uses velocity PID, turning uses position PID
        driveClosedLoopController.setReference(desiredSpeed, ControlType.kVelocity);
        turningClosedLoopController.setReference(desiredAngle, ControlType.kPosition);
    }

    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    // gives the robot relative turning position (gives 0 degrees if robot moving 0 degrees)
    private double getTurningPosition() {
        return turningEncoder.getPosition() - chasisAngularOffset;
    }

    public double getDriveSpeed() {
        return getDriveVelocity();
    }

    public double getSteerAngle() {
        return getTurningPosition();
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();

        desiredSpeed = 0;
        desiredAngle = 0;
    }

    /**
     * Returns the current position of the drive encoder in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    // updates the states of the simulated motors (velocity, and pos), which automatically updates the encoders of the actual motors
    public void updateSimulatorState() {
        driveMotorSim.iterate(desiredSpeed, driveMotor.getBusVoltage(), 0.02);
        
        double positionError = desiredAngle - turningEncoder.getPosition();
        double velocityRadPerSec = positionError / 0.02;

        turningMotorSim.iterate(velocityRadPerSec, turningMotor.getBusVoltage(), 0.02);
    } 

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getDesiredSpeed() {
        return desiredSpeed;
    }
    
    public String getModuleName() {
        return moduleName;
    }

    /**
     * Returns the raw absolute encoder position (for calibration)
     */
    public double getRawAbsoluteEncoderPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * Returns the applied voltage to the drive motor
     * Calculated as bus voltage * applied output (duty cycle)
     */
    public double getDriveVoltage() {
        return driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
    }

    /**
     * Returns the applied voltage to the turning motor
     * Calculated as bus voltage * applied output (duty cycle)
     */
    public double getTurningVoltage() {
        return turningMotor.getBusVoltage() * turningMotor.getAppliedOutput();
    }

    /**
     * Set drive motor power directly (for testing)
     * @param power Power from -1.0 to 1.0
     */
    public void setDrivePower(double power) {
        driveMotor.set(power);
    }

    /**
     * Set turning motor power directly (for testing)
     * @param power Power from -1.0 to 1.0
     */
    public void setTurningPower(double power) {
        turningMotor.set(power);
    }

    /**
     * Set turning motor to a specific angle using PID (for testing)
     * @param angleRadians Target angle in radians
     */
    public void setTurningAngle(double angleRadians) {
        double targetWithOffset = angleRadians + chasisAngularOffset;
        turningClosedLoopController.setReference(targetWithOffset, ControlType.kPosition);
    }
}
