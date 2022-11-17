// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberDownToPosition extends CommandBase {
  private Climber m_climber;
  private double m_percent;
  /** Creates a new ClimberDown. */
  public ClimberDownToPosition(Climber subsystem, double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = subsystem;
    m_percent = percent;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.ClimberDownToPosition(m_percent);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.moveSol(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.ClimberDownAtPosition(m_percent);
  }
}
