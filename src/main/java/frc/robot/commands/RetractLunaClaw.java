// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LunaClimb;

public class RetractLunaClaw extends CommandBase {

  private LunaClimb m_LunaClimb;
  /** Creates a new OpenLunaClaw. */
  public RetractLunaClaw(LunaClimb subsystem) {
    m_LunaClimb = subsystem;
    addRequirements(m_LunaClimb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LunaClimb.retractLuna();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LunaClimb.retractLuna();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
