// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.AutoAllign;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.EmergencyStopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private static final double SHOOTER_DEADBAND = 0.06;
  private static final double ROTATION_MULTIPLIER = 0.7; // Rotation is 70% of translation speed

  // setup the AutoBuilder with all pathplanner paths in place
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }

  /**
   * Apply deadband and square the input for smoother control
   * Squaring preserves the sign but gives more precision at low speeds
   */
  private double applyDeadbandAndCurve(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }
    // Remove deadband from the range, then square for smoother curve
    double sign = Math.signum(value);
    double adjusted = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    return sign * adjusted * adjusted; // Squared input for fine control
  }

  /**
   * Apply deadband without curve (linear response)
   */
  private double applyDeadband(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }
    return value;
  }

  private double getForwardInput() {
    double primaryInput = -driveController.getLeftY();
    return applyDeadbandAndCurve(primaryInput);
  }

  private double getStrafeInput() {
    double primaryInput = -driveController.getLeftX();
    return applyDeadbandAndCurve(primaryInput);
  }

  private double getRotationInput() {
    double primaryInput = -driveController.getRightX();
    // Rotation uses curve + reduced sensitivity for precision turning
    return applyDeadbandAndCurve(primaryInput) * ROTATION_MULTIPLIER;
  }

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
     // Set up the default command for the drive subsystem (60% speed default)
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * 0.6,  // Forward/backward
            () -> getStrafeInput() * 0.6,   // Left/right
            () -> getRotationInput() * 0.6  // Rotation
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
    // Configure button bindings for both controllers
    // Both controllers have identical bindings for redundancy and flexibility
    // This allows either the driver or operator to control any function if needed
    
    // Primary Xbox Controller Bindings (port 0)

    // Emergency stop for all subsystems (Back + Start buttons together)
    new JoystickButton(driveController, XboxController.Button.kBack.value)
        .and(new JoystickButton(driveController, XboxController.Button.kStart.value))
        .onTrue(new EmergencyStopCommand(driveSubsystem));

      // Slow driving mode for primary controller
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
    .whileTrue(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * 0.45,
            () -> getStrafeInput() * 0.45,
            () -> getRotationInput() * 0.45
        )
    );

    // Auto-align to AprilTag using Limelight (A button)
    new JoystickButton(driveController, XboxController.Button.kA.value)
        .whileTrue(new AutoAllign(limelightSubsystem, driveSubsystem));

    // Shooter - toggle on/off with B button (feedforward control at 5000 RPM)
    new JoystickButton(driveController, XboxController.Button.kB.value)
        .toggleOnTrue(new RunCommand(() -> shooterSubsystem.setShooterVelocityFF(5000), shooterSubsystem)
            .finallyDo(() -> shooterSubsystem.stop()));

    // ==================== WHEEL TEST BUTTONS ====================
    // X button: Test drive motors (hold to spin wheels at 20% power)
    new JoystickButton(driveController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> driveSubsystem.testDriveMotors(0.2), driveSubsystem)
            .finallyDo(() -> driveSubsystem.stop()));

    // Y button: Test turning motors (hold to rotate wheels at 20% power)
    new JoystickButton(driveController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> driveSubsystem.testTurningMotors(0.2), driveSubsystem)
            .finallyDo(() -> driveSubsystem.stop()));

    // Left bumper: Point all wheels forward (0 degrees)
    new JoystickButton(driveController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> driveSubsystem.setAllWheelAngles(0), driveSubsystem)
            .finallyDo(() -> driveSubsystem.stop()));

    // RT (Right Trigger): Sprint mode - full speed (4.5 m/s)
    new Trigger(() -> driveController.getRightTriggerAxis() > 0.5)
        .whileTrue(new DefaultDriveCommand(
            driveSubsystem,
            () -> getForwardInput() * 1.67,  // 1.67 * 0.6 = 1.0 (full speed)
            () -> getStrafeInput() * 1.67,
            () -> getRotationInput() * 1.43   // Slightly faster rotation in sprint
        ));

    // Start button: Reset gyro (resets field-relative forward direction)
    new JoystickButton(driveController, XboxController.Button.kStart.value)
        .onTrue(new RunCommand(() -> {
            driveSubsystem.zeroHeading();
            System.out.println("Gyro reset - current heading is now forward");
        }, driveSubsystem).withTimeout(0.1));

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
