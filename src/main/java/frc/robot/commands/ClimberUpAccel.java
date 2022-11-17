// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ClimberUpAccel extends CommandBase {
  private Climber m_climber;
  private DrivetrainSubsystem m_drive;
  /** Creates a new ClimberDown. */
  public ClimberUpAccel(Climber subsystem, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = subsystem;
    m_drive = drive;
    addRequirements(m_climber, m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drive.getAccelX()>0){
      m_climber.moveSol(1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.moveSol(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.atTop();
  }
}
