// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  //private Solenoid m_Kicker = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private Solenoid m_intakeDeploy = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private VictorSPX m_KickerMotor = new VictorSPX(1);
  private VictorSPX m_intakeMotor = new VictorSPX(2);

  private double m_intakeDefaultSpeed = 1.0;

  /** Creates a new Intake. */
  public Intake() {

    
  }



  public void RunIntake(){
    m_intakeMotor.set(ControlMode.PercentOutput, -m_intakeDefaultSpeed);
  }

  public void ReverseIntake(){
    m_intakeMotor.set(ControlMode.PercentOutput, m_intakeDefaultSpeed);
  }

  public void setIntakeSpeed(double speed){
    m_intakeMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void DeployIntake(){
    m_intakeDeploy.set(true);
  }

  public void RetractIntake(){
    m_intakeDeploy.set(false);
  }

  public void setIntake(boolean bool){
    m_intakeDeploy.set(bool);
  }

public void engageKicker(){
  m_KickerMotor.set(ControlMode.PercentOutput, -1.0);
}

public void engageKicker(double speed){
  m_KickerMotor.set(ControlMode.PercentOutput, -speed);
}

public void reverseKicker(){
  m_KickerMotor.set(ControlMode.PercentOutput, 1.0);
}

public void disengageKicker(){
  m_KickerMotor.set(ControlMode.PercentOutput, 0.0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
