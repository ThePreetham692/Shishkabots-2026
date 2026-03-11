// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private static final double DEADBAND = 0.10;
  private static final double TELEOP_TRANSLATION_SCALE = 0.25;
  private static final boolean TELEOP_FIELD_RELATIVE = false;

  // setup the AutoBuilder with all pathplanner paths in place
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }

  private XboxController getActiveController() {
    if (DriverStation.isJoystickConnected(DRIVER_CONTROLLER_PORT)) {
      return driveController;
    }
    if (DriverStation.isJoystickConnected(OPERATOR_CONTROLLER_PORT)) {
      return mechanismController;
    }
    return driveController;
  }

  private Trigger button(int buttonId) {
    return new Trigger(
        () -> driveController.getRawButton(buttonId) || mechanismController.getRawButton(buttonId));
  }

  private double getForwardInput() {
    return MathUtil.applyDeadband(-getActiveController().getLeftY(), DEADBAND);
  }

  private double getStrafeInput() {
    // Left stick X commands pure left/right translation.
    return MathUtil.applyDeadband(-getActiveController().getLeftX(), DEADBAND);
  }

  private double getTurnInput() {
    // Disable turning: translation-only calibration mode.
    return 0.0;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
     // Set up the default command for the drive subsystem (60% speed default)
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * TELEOP_TRANSLATION_SCALE,
            () -> getStrafeInput() * TELEOP_TRANSLATION_SCALE,
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
    button(XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(driveSubsystem::zeroHeading, driveSubsystem));
    button(XboxController.Button.kY.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.setAllWheelAngles(0.0), driveSubsystem));
    button(XboxController.Button.kBack.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.setAllWheelAngles(0.0), driveSubsystem));
    // Right stick click sets/holds the initial straight wheel position.
    button(XboxController.Button.kRightStick.value)
        .whileTrue(Commands.run(() -> driveSubsystem.setAllWheelAngles(0.0), driveSubsystem));

    // Toggle B: press once to run shooter + tower + conveyor, press again to stop.
    // Use steady open-loop output to avoid velocity-PID oscillation (red/green flicker).
    button(XboxController.Button.kB.value)
        .toggleOnTrue(Commands.startEnd(
            () -> shooterSubsystem.setShooterPower(0.85),
            () -> shooterSubsystem.stop(),
            shooterSubsystem));

    // Press X to toggle intake motor (CAN 16) at 0.6 power.
    button(XboxController.Button.kX.value)
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
