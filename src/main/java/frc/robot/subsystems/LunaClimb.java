// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LunaClimb extends SubsystemBase {

  private Solenoid m_LunaClawL = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private Solenoid m_LunaClawR = new Solenoid(PneumaticsModuleType.REVPH, 2);
  private Solenoid m_LunaRetract = new Solenoid(PneumaticsModuleType.REVPH, 3);
  /** Creates a new LunaClimb. */
  public LunaClimb() {
    
  }

  public void setLuna(boolean bool){
    m_LunaRetract.set(bool);
  }

  public void retractLuna(){
    m_LunaRetract.set(true);
  }

  public void raiseLuna(){
    m_LunaRetract.set(false);
  }

  public void setLunaClaw(boolean bool){
    m_LunaClawL.set(bool);
    m_LunaClawR.set(bool);
  }

  public void closeLunaClaw(){
    m_LunaClawL.set(true);
    m_LunaClawR.set(true);
  }

  public void openLunaClaw(){
    m_LunaClawL.set(false);
    m_LunaClawR.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
