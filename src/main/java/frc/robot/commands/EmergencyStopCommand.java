// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.YagslSwerveSubsystem;
import util.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EmergencyStopCommand extends Command {
  private final YagslSwerveSubsystem driveSubsystem;
  /** Creates a new EmergencyStopCommand. */
  public EmergencyStopCommand(YagslSwerveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        
        // Add all subsystems as requirements to ensure no other commands run on them
        addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("EMERGENCY STOP ACTIVATED");
    
    // Stop all subsystems
    driveSubsystem.stop();
    
    // Add visual indicator to dashboard
    SmartDashboard.putBoolean("EmergencyStop", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("Emergency stop complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This is a one-shot command that finishes immediately after stopping everything
    return true;
  }
}
