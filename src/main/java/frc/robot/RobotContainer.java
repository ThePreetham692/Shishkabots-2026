// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  // Shooter: Left CAN ID, Right CAN ID | Tower CAN ID | Conveyor CAN ID | Intake CAN ID
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(11, 9, 12, 13, 16);

  // The driver's controllers
  // Primary controller (port 0) is for the main driver
  // Secondary controller (port 1) is for the operator/co-pilot
  // Both controllers have the same button mappings for redundancy
  private final XboxController driveController = new XboxController(DRIVER_CONTROLLER_PORT); // Primary controller
  private final XboxController mechanismController = new XboxController(OPERATOR_CONTROLLER_PORT); // Secondary controller

  private static final double DEADBAND = 0.07;
  private static final double NORMAL_DRIVE_SCALE = 0.95;
  private static final double SPRINT_DRIVE_SCALE = 1.20;
  private static final double NORMAL_TURN_SCALE = 1.00;
  private static final double SPRINT_TURN_SCALE = 1.25;
  private static final double OUTWARD_TURN_ASSIST = 0.35;
  private static final boolean TELEOP_FIELD_RELATIVE = false;

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

  private double getDriveScale() {
    return driveController.getRightBumperButton() ? SPRINT_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
  }

  private double getTurnScale() {
    return driveController.getRightBumperButton() ? SPRINT_TURN_SCALE : NORMAL_TURN_SCALE;
  }

  private double getForwardInput() {
    double rawForward = -driveController.getLeftY();
    return applyDeadbandAndCurve(rawForward);
  }

  private double getTurnInput() {
    double rawTurn = -driveController.getRightX();
    return applyDeadbandAndCurve(rawTurn) * getTurnScale();
  }

  private double getOutwardTurnStrafeAssist() {
    double assist = getForwardInput() * getTurnInput() * OUTWARD_TURN_ASSIST;
    return Math.max(-1.0, Math.min(1.0, assist));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
     // Set up the default command for the drive subsystem (60% speed default)
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * getDriveScale(),
            () -> getOutwardTurnStrafeAssist(),
            () -> getTurnInput(),
            TELEOP_FIELD_RELATIVE
        )
    );

    // Connect Limelight to robot pose for simulation
    limelightSubsystem.setRobotPoseSupplier(() -> driveSubsystem.getPose());
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
    // Toggle B: press once to run shooter + tower + conveyor, press again to stop.
    // Use steady open-loop output to avoid velocity-PID oscillation (red/green flicker).
    new JoystickButton(driveController, XboxController.Button.kB.value)
        .toggleOnTrue(Commands.startEnd(
            () -> shooterSubsystem.setShooterPower(0.85),
            () -> shooterSubsystem.stop(),
            shooterSubsystem));

    // Press X to toggle intake motor (CAN 16) at 0.6 power.
    new JoystickButton(driveController, XboxController.Button.kX.value)
        .onTrue(Commands.runOnce(
            () -> shooterSubsystem.toggleIntakeOnly(0.6),
            shooterSubsystem));
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

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}
