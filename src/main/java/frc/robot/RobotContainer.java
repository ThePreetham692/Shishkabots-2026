// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;

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
  // Shooter: Left CAN ID, Right CAN ID | Tower CAN ID | Conveyor CAN ID
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(11, 9, 12, 13);

  // The driver's controllers
  // Primary controller (port 0) is for the main driver
  // Secondary controller (port 1) is for the operator/co-pilot
  // Both controllers have the same button mappings for redundancy
  private final XboxController driveController = new XboxController(0); // Primary controller on port 0
  private final XboxController mechanismController = new XboxController(1); // Secondary controller on port 1

  private static final double DEADBAND = 0.1;
  private static final double ROTATION_MULTIPLIER = 0.7; // Rotation is 70% of translation speed
  private static final double NORMAL_TRANSLATION_SCALE = 0.70;
  private static final double NORMAL_ROTATION_SCALE = 0.65;
  private static final double SPRINT_TRANSLATION_SCALE = 1.00;
  private static final double SPRINT_ROTATION_SCALE = 0.90;
  private static final boolean TELEOP_FIELD_RELATIVE = false;

  // setup the AutoBuilder with all pathplanner paths in place
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }

  /**
   * Apply deadband and a blended cubic response for smoother, less twitchy driving.
   */
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

  /**
   * Radial deadband for the translation stick so tiny stick noise does not cause sideways drift.
   * Returns [forward, strafe] in robot-centric axes.
   */
  private double[] getTranslationInputs() {
    double rawForward = -driveController.getLeftY();
    double rawStrafe = -driveController.getLeftX();

    double magnitude = Math.hypot(rawForward, rawStrafe);
    if (magnitude < DEADBAND) {
      return new double[] {0.0, 0.0};
    }

    double adjustedMagnitude = (magnitude - DEADBAND) / (1.0 - DEADBAND);
    double curvedMagnitude = 0.5 * adjustedMagnitude + 0.5 * adjustedMagnitude * adjustedMagnitude * adjustedMagnitude;

    double unitForward = rawForward / magnitude;
    double unitStrafe = rawStrafe / magnitude;

    return new double[] {unitForward * curvedMagnitude, unitStrafe * curvedMagnitude};
  }

  private double getTranslationScale() {
    return driveController.getRightBumperButton() ? SPRINT_TRANSLATION_SCALE : NORMAL_TRANSLATION_SCALE;
  }

  private double getRotationScale() {
    return driveController.getRightBumperButton() ? SPRINT_ROTATION_SCALE : NORMAL_ROTATION_SCALE;
  }

  private double getForwardInput() {
    return getTranslationInputs()[0];
  }

  private double getRotationInput() {
    double primaryInput = -driveController.getRightX();
    // Rotation uses curve + reduced sensitivity for precision turning
    return applyDeadbandAndCurve(primaryInput) * ROTATION_MULTIPLIER;
  }

  private double getStrafeInput() {
    return getTranslationInputs()[1];
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
     // Set up the default command for the drive subsystem (60% speed default)
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * getTranslationScale(),
            () -> getStrafeInput() * getTranslationScale(),
            () -> getRotationInput() * getRotationScale(),
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
    // No command bindings needed for drive. Sprint is handled directly from Right Bumper in suppliers.
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
