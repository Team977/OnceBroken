// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetOdometryAdjustment extends CommandBase {
  private DrivetrainSubsystem m_drive;
  private double m_x;
  private double m_y;
  private double m_angle;
  /** Creates a new SetOdometryAdjustment. */
  public SetOdometryAdjustment(DrivetrainSubsystem subsystem, double x, double y, double angle) {
    m_drive = subsystem;
    m_x = x;
    m_y = y;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setOdometryOffset(m_x, m_y, m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
