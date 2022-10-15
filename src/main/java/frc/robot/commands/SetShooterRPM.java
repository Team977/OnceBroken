// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {
  private Shooter m_Shooter;
  private Limelight m_Limelight;
  private boolean m_HighorLow;
  /** Creates a new SetShooterRPM. */
  public SetShooterRPM(Shooter subsystem, Limelight limelight, boolean HighorLow) {
    m_Shooter = subsystem;
    m_Limelight = limelight;
    m_HighorLow = HighorLow;
    
    addRequirements(m_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //m_Shooter.setRPM(m_Limelight.getRPM());
    SmartDashboard.putBoolean("Limelight Target", m_Limelight.hasTarget());
    if (m_Limelight.hasTarget()){
      if (m_HighorLow){
        m_Shooter.setRPM(m_Limelight.getHighRPM());
        m_Shooter.setHoodPosition(m_Limelight.getHighHoodPosition());
        SmartDashboard.putNumber("Limelight RPM", m_Limelight.getHighRPM());
        SmartDashboard.putNumber("Limelight HOOD", m_Limelight.getHighHoodPosition());
      } 
      else {
        m_Shooter.setRPM(m_Limelight.getLowRPM());
        m_Shooter.setHoodPosition(m_Limelight.getLowHoodPosition());
        SmartDashboard.putNumber("Limelight RPM", m_Limelight.getLowRPM());
        SmartDashboard.putNumber("Limelight HOOD", m_Limelight.getLowHoodPosition());
      }
    } 
    else {
      m_Shooter.setRPM(RobotContainer.getRPM());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.setRPM(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
