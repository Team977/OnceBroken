// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  // private Solenoid m_LunaClaw = new Solenoid(PneumaticsModuleType.REVPH, 2);
  // private Solenoid m_LunaRetract = new Solenoid(PneumaticsModuleType.REVPH, 3);
  private CANSparkMax m_SolLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax m_SolRightMotor = new CANSparkMax(4, MotorType.kBrushless);

  private DigitalInput m_LeftClimbDownLimit = new DigitalInput(2);
  private DigitalInput m_RightClimbDownLimit = new DigitalInput(1);
  
  //private DigitalInput m_LeftClimbUpLimit = new DigitalInput(4);
  //private DigitalInput m_RightClimbUpLimit = new DigitalInput(3);

  //Wire low/high limit switches into each Spark MAX

  private double m_SolMotorDefaultSpeed = 0.5;

  /** Creates a new Climber. */
  public Climber() {

    //m_SolRightMotor.follow(m_SolLeftMotor, false);
    //m_SolLeftMotor.follow(m_SolRightMotor, false);
    //m_SolRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
     m_SolRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
     m_SolRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    
     m_SolLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
     m_SolLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

     
    m_SolLeftMotor.getEncoder().setPosition(0.0);
    m_SolRightMotor.getEncoder().setPosition(0.0);

    m_SolLeftMotor.setSoftLimit(SoftLimitDirection.kForward, 179);
    
    m_SolRightMotor.setSoftLimit(SoftLimitDirection.kForward, 179);
    
    m_SolLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_SolRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

  }

  public void moveSol(double speed){


    if ((!m_LeftClimbDownLimit.get()&&speed<0) ){
    m_SolLeftMotor.set(0.0);
    m_SolLeftMotor.getEncoder().setPosition(0.0);
    } else {
      if (m_SolLeftMotor.getEncoder().getPosition()<40){
        m_SolLeftMotor.set(speed/4);
      } else {
        m_SolLeftMotor.set(speed);
      }
     
    }
    if ((!m_RightClimbDownLimit.get()&&speed<0) ){
      m_SolRightMotor.set(0.0);
      m_SolRightMotor.getEncoder().setPosition(0.0);
      } else {      
        if (m_SolLeftMotor.getEncoder().getPosition()<40){
       
        m_SolRightMotor.set(speed/4);
      } else {
        m_SolRightMotor.set(speed);
      }
    }

  }

  public boolean atBottom(){
    return (!m_LeftClimbDownLimit.get() && !m_RightClimbDownLimit.get());
  }


  public boolean atTop(){
    return (m_SolLeftMotor.getEncoder().getPosition()>=168 && m_SolRightMotor.getEncoder().getPosition()>=178);
  }

  public void extendSol(){
    
    m_SolLeftMotor.set(m_SolMotorDefaultSpeed);
    m_SolRightMotor.set(m_SolMotorDefaultSpeed);
    /*
    if (!m_LeftClimbUpLimit.get()){
      m_SolLeftMotor.set(m_SolMotorDefaultSpeed);ks
      } else {
        m_SolLeftMotor.set(0.0);
      }
      if (!m_RightClimbUpLimit.get()){
        m_SolRightMotor.set(m_SolMotorDefaultSpeed);
        } else {
          m_SolRightMotor.set(0.0);
        }
        */
   }

  public void retractSol(){

    m_SolLeftMotor.set(-m_SolMotorDefaultSpeed);
    m_SolRightMotor.set(-m_SolMotorDefaultSpeed);
    /*
    if (!m_LeftClimbDownLimit.get()){
      m_SolLeftMotor.set(-m_SolMotorDefaultSpeed);
      } else {
        m_SolLeftMotor.set(0.0);
      }
      if (!m_RightClimbDownLimit.get()){
        m_SolRightMotor.set(-m_SolMotorDefaultSpeed);
        } else {
          m_SolRightMotor.set(0.0);
        }
*/
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb Pos", m_SolLeftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Climb Pos", m_SolRightMotor.getEncoder().getPosition());
    //SmartDashboard.putBoolean("Left Down Limit", m_LeftClimbDownLimit.get());
    //SmartDashboard.putBoolean("Right Down Limit", m_RightClimbDownLimit.get());
    //SmartDashboard.putBoolean("At Bottom", atBottom());
    //SmartDashboard.putBoolean("At Top", atTop());
  
  }
}
