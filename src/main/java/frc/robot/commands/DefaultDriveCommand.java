// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.YagslSwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCommand extends Command {
  private final YagslSwerveSubsystem driveSubsystem;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotationSupplier;
    private final boolean fieldRelative;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command will run on
     * @param xSpeed The forward/backward speed supplier
     * @param ySpeed The left/right speed supplier
     * @param rotation The rotation speed supplier
     */
    public DefaultDriveCommand(
            YagslSwerveSubsystem subsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotation) {
        this(subsystem, xSpeed, ySpeed, rotation, true);
    }

    public DefaultDriveCommand(
            YagslSwerveSubsystem subsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotation,
            boolean useFieldRelative) {
        driveSubsystem = subsystem;
        xSpeedSupplier = xSpeed;
        ySpeedSupplier = ySpeed;
        rotationSupplier = rotation;
        fieldRelative = useFieldRelative;
        addRequirements(subsystem); // ensures command has exclusive use of the drive subsystem.
            }
        /** Creates a new DefaultDriveCommand. */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(
            xSpeedSupplier.getAsDouble(),
            ySpeedSupplier.getAsDouble(),
            rotationSupplier.getAsDouble(),
            fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command will run forever as we want to keep the robot moving
    return false;
  }
}
