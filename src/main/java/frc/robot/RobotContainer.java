// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.YagslSwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final YagslSwerveSubsystem yagslSwerveSubsystem = new YagslSwerveSubsystem();
  // Shooter: Left CAN ID, Right CAN ID | Tower CAN ID | Conveyor CAN ID | Intake CAN ID
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(11, 9, 12, 13, 16);

  // The driver's controllers
  // Primary controller (port 0) is for the main driver
  // Secondary controller (port 1) is for the operator/co-pilot
  // Both controllers have the same button mappings for redundancy
  private final XboxController driveController = new XboxController(DRIVER_CONTROLLER_PORT); // Primary controller
  private final XboxController mechanismController = new XboxController(OPERATOR_CONTROLLER_PORT); // Secondary controller

  private static final double DEADBAND = 0.07;
  private static final double BRAKE_DRIVE_SCALE = 0.25;
  private static final double NORMAL_DRIVE_SCALE = 0.80;
  private static final double SPRINT_DRIVE_SCALE = 0.95;
  private static final double BRAKE_TURN_SCALE = 0.20;
  private static final double NORMAL_TURN_SCALE = 0.40;
  private static final double SPRINT_TURN_SCALE = 0.60;
  private static final boolean TELEOP_FIELD_RELATIVE = false; // Keep false while validating gyro sign.

  // setup the AutoBuilder with all pathplanner paths in place
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }

  private double applyDeadbandAndCurve(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }

    double sign = Math.signum(value);
    double adjusted = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    // Blend linear and cubic to keep fine control near center without feeling sluggish at mid-stick.
    double curved = 0.5 * adjusted + 0.5 * adjusted * adjusted * adjusted;
    return sign * curved;
  }

  private double applyTurnDeadbandAndCurve(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }

    double sign = Math.signum(value);
    double adjusted = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    // Heavier cubic weighting makes rotation much finer near center.
    double curved = 0.2 * adjusted + 0.8 * adjusted * adjusted * adjusted;
    return sign * curved;
  }

  private double getDriveScale() {
    double brakeAmount = driveController.getLeftTriggerAxis();
    double sprintAmount = driveController.getRightTriggerAxis();

    double scale = NORMAL_DRIVE_SCALE;
    scale -= brakeAmount * (NORMAL_DRIVE_SCALE - BRAKE_DRIVE_SCALE);
    scale += sprintAmount * (SPRINT_DRIVE_SCALE - NORMAL_DRIVE_SCALE);

    return MathUtil.clamp(scale, BRAKE_DRIVE_SCALE, SPRINT_DRIVE_SCALE);
  }

  private double getTurnScale() {
    double brakeAmount = driveController.getLeftTriggerAxis();
    double sprintAmount = driveController.getRightTriggerAxis();

    double scale = NORMAL_TURN_SCALE;
    scale -= brakeAmount * (NORMAL_TURN_SCALE - BRAKE_TURN_SCALE);
    scale += sprintAmount * (SPRINT_TURN_SCALE - NORMAL_TURN_SCALE);

    return MathUtil.clamp(scale, BRAKE_TURN_SCALE, SPRINT_TURN_SCALE);
  }

  private double getForwardInput() {
    double rawForward = -driveController.getLeftY();
    return applyDeadbandAndCurve(rawForward);
  }

  private double getStrafeInput() {
    double rawStrafe = -driveController.getLeftX();
    return applyDeadbandAndCurve(rawStrafe);
  }

  private double getTurnInput() {
    double rawTurn = -driveController.getRightY();
    return MathUtil.clamp(applyTurnDeadbandAndCurve(rawTurn) * getTurnScale(), -1.0, 1.0);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    yagslSwerveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            yagslSwerveSubsystem,
            () -> getForwardInput() * getDriveScale(),
            () -> getStrafeInput() * getDriveScale(),
            () -> getTurnInput(),
            TELEOP_FIELD_RELATIVE));
    SmartDashboard.putBoolean("Drive/UsingYAGSL", true);
    SmartDashboard.putBoolean("YAGSL/ConfigLoaded", yagslSwerveSubsystem.isConfigLoaded());

    // Connect Limelight to robot pose for simulation
    limelightSubsystem.setRobotPoseSupplier(() -> yagslSwerveSubsystem.getPose());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // A: zero gyro heading for YAGSL drivebase.
    new JoystickButton(driveController, XboxController.Button.kA.value)
        .onTrue(
            edu.wpi.first.wpilibj2.command.Commands.runOnce(
                yagslSwerveSubsystem::zeroGyro, yagslSwerveSubsystem));

    // Y (hold): continuously print FL/FR/BL/BR raw absolute encoder values to DS log.
    // Calibration workflow:
    // 1) Put robot on blocks and disable outputs.
    // 2) Point all wheels straight forward by hand.
    // 3) Hold Y and read the four ABS values in DS log/console.
    // 4) Copy values into absoluteEncoderOffset in:
    //    src/main/deploy/swerve/modules/fl.json
    //    src/main/deploy/swerve/modules/fr.json
    //    src/main/deploy/swerve/modules/bl.json
    //    src/main/deploy/swerve/modules/br.json
    // 5) Redeploy code and re-test wheel alignment.
    new JoystickButton(driveController, XboxController.Button.kY.value)
        .whileTrue(
            edu.wpi.first.wpilibj2.command.Commands.run(
                yagslSwerveSubsystem::printCalibrationAbsoluteEncoders, yagslSwerveSubsystem));

    // Toggle B: press once to run shooter + tower + conveyor, press again to stop.
    // Use steady open-loop output to avoid velocity-PID oscillation (red/green flicker).
    new JoystickButton(driveController, XboxController.Button.kB.value)
        .toggleOnTrue(
            edu.wpi.first.wpilibj2.command.Commands.startEnd(
                shooterSubsystem::shootWithPID,
                () -> shooterSubsystem.stop(),
                shooterSubsystem));

    // Press X to toggle intake motor (CAN 16) at 0.6 power.
    new JoystickButton(driveController, XboxController.Button.kX.value)
        .onTrue(
            edu.wpi.first.wpilibj2.command.Commands.runOnce(
                () -> shooterSubsystem.toggleIntakeOnly(0.6), shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return selected auto command, or null if none selected
    return autoChooser.getSelected();
  }

  public YagslSwerveSubsystem getDriveSubsystem() {
    return yagslSwerveSubsystem;
  }

  public void zeroDriveHeading() {
    yagslSwerveSubsystem.zeroGyro();
  }
}
