package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.YagslSwerveSubsystem;
import util.Logger;

public class AutoAllign extends Command {
  private final LimelightSubsystem limelight;
  private final YagslSwerveSubsystem drive;

  // PID Controllers for alignment
  private final PIDController distancePID = new PIDController(0.8, 0, 0.05);
  private final PIDController strafePID = new PIDController(0.05, 0, 0.005);
  private final PIDController rotationPID = new PIDController(0.03, 0, 0.002);

  // Target distance from AprilTag in meters
  private final double targetDistance;

  // Tolerances
  private static final double DISTANCE_TOLERANCE = 0.1;  // ±10 cm
  private static final double STRAFE_TOLERANCE = 2.0;    // ±2 degrees
  private static final double ROTATION_TOLERANCE = 3.0;  // ±3 degrees

  // Speed limits
  private static final double MAX_DRIVE_SPEED = 1.5;     // m/s
  private static final double MAX_ROTATION_SPEED = 2.0;  // rad/s

  // AprilTag height for distance calculation (meters)
  private static final double TAG_HEIGHT = 1.45;  // Adjust for 2025 Reefscape tags

  /**
   * Creates a new AutoAlign command with default 1.0m target distance.
   */
  public AutoAllign(LimelightSubsystem limelight, YagslSwerveSubsystem drive) {
    this(limelight, drive, 1.0);
  }

  /**
   * Creates a new AutoAlign command with specified target distance.
   * @param limelight The limelight subsystem
   * @param drive The drive subsystem
   * @param targetDistanceMeters Distance to maintain from AprilTag (meters)
   */
  public AutoAllign(
      LimelightSubsystem limelight,
      YagslSwerveSubsystem drive,
      double targetDistanceMeters) {
    this.limelight = limelight;
    this.drive = drive;
    this.targetDistance = targetDistanceMeters;

    // Set tolerances
    distancePID.setTolerance(DISTANCE_TOLERANCE);
    strafePID.setTolerance(STRAFE_TOLERANCE);
    rotationPID.setTolerance(ROTATION_TOLERANCE);

    // Declare subsystem dependencies
    addRequirements(limelight, drive);
  }

  @Override
  public void initialize() {
    // Reset PIDs when command starts
    distancePID.reset();
    strafePID.reset();
    rotationPID.reset();
    Logger.log("AutoAlign started - target distance: " + targetDistance + "m");
  }

  @Override
  public void execute() {
    // Check if we have a valid AprilTag target
    if (!limelight.hasValidTarget()) {
      drive.stop();
      Logger.log("AutoAlign: No valid target detected");
      return;
    }

    // Get horizontal angle (tx) - how far left/right the tag is
    double tx = limelight.getX();

    // Get vertical angle (ty) - used for distance calculation
    double ty = limelight.getY();

    // Calculate distance to tag using trigonometry
    // distance = (tagHeight - cameraHeight) / tan(cameraAngle + ty)
    double currentDistance = calculateDistanceFromTag(ty);

    // Calculate errors
    double distanceError = currentDistance - targetDistance;  // positive = too far
    double strafeError = tx;                                   // positive = tag is to the right
    double rotationError = tx;                                 // rotate to face the tag

    // Calculate PID outputs
    double vx = -distancePID.calculate(distanceError, 0);  // negative because forward reduces distance
    double vy = -strafePID.calculate(strafeError, 0);      // strafe to center the tag
    double omega = -rotationPID.calculate(rotationError, 0); // rotate to face tag

    // Clamp speeds for safety
    vx = MathUtil.clamp(vx, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    vy = MathUtil.clamp(vy, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    omega = MathUtil.clamp(omega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Log for debugging
    Logger.log(String.format("AutoAlign: dist=%.2fm (err=%.2f), tx=%.1f°, vx=%.2f, vy=%.2f, omega=%.2f",
        currentDistance, distanceError, tx, vx, vy, omega));

    // Drive the robot
    drive.driveMetersPerSecond(vx, vy, omega, true);
  }

  /**
   * Calculate distance from the AprilTag using vertical angle
   * @param ty Vertical angle to target in degrees
   * @return Distance in meters
   */
  private double calculateDistanceFromTag(double ty) {
    // Camera mounting angle (degrees above horizontal) - adjust for your robot
    double cameraAngleDegrees = 0.0;  // TODO: Set your camera mount angle
    double cameraHeightMeters = 0.5;   // TODO: Set your camera height

    double totalAngleRadians = Math.toRadians(cameraAngleDegrees + ty);

    // Avoid division by zero
    if (Math.abs(totalAngleRadians) < 0.01) {
      return targetDistance;  // Return target as fallback
    }

    double distance = (TAG_HEIGHT - cameraHeightMeters) / Math.tan(totalAngleRadians);
    return Math.abs(distance);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.log("AutoAlign ended - interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    // Only finish if we have a target and all PIDs are at setpoint
    if (!limelight.hasValidTarget()) {
      return false;  // Keep trying if no target
    }

    boolean aligned = distancePID.atSetpoint() &&
                      strafePID.atSetpoint() &&
                      rotationPID.atSetpoint();

    if (aligned) {
      Logger.log("AutoAlign: Alignment complete!");
    }

    return aligned;
  }
}
