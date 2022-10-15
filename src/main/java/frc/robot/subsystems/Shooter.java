// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

 
  private CANSparkMax m_LeftShooterMotor = new CANSparkMax(5,MotorType.kBrushless);
  private CANSparkMax m_RightShooterMotor = new CANSparkMax(1,MotorType.kBrushless);
  private CANSparkMax m_HoodMotor = new CANSparkMax(2,MotorType.kBrushed);

  
  //private DutyCycleEncoder m_HoodEncoder = new DutyCycleEncoder(0);
  
  private DutyCycleEncoder m_HoodEncoder = new DutyCycleEncoder(0);

  private SparkMaxPIDController mPIDcontroller;
  private edu.wpi.first.math.controller.PIDController mHoodPIDController;

  /** Creates a new Shooter. */
  public Shooter() {

    m_LeftShooterMotor.restoreFactoryDefaults();
    m_RightShooterMotor.restoreFactoryDefaults();
    m_LeftShooterMotor.setClosedLoopRampRate(0.25);
    m_LeftShooterMotor.setInverted(false);
    m_RightShooterMotor.follow(m_LeftShooterMotor, true);

    m_RightShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    m_RightShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    m_RightShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    m_LeftShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    m_HoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_HoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    m_HoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    
    m_HoodMotor.setInverted(true);
    mHoodPIDController = new edu.wpi.first.math.controller.PIDController(-20, 0.0, 0.0);
    
    mPIDcontroller = m_LeftShooterMotor.getPIDController();

    mPIDcontroller.setP(0.0001);
    mPIDcontroller.setI(0.0);
    mPIDcontroller.setD(0.0);
    mPIDcontroller.setIZone(0.0);
    mPIDcontroller.setFF(0.0002);
    mPIDcontroller.setOutputRange(0.0, 0.99); //BRB 0.90

    m_LeftShooterMotor.burnFlash();
  }

  public void setRPM(double rpm){
    mPIDcontroller.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseShooter(){
    m_LeftShooterMotor.set(-0.25);
  }
  
  public void publishEncoderValues(){
    SmartDashboard.putNumber("Shooter RPM", m_LeftShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Hood Position", getHoodPosition());
    //SmartDashboard.putNumber("Raw Hood Position", m_HoodEncoder.getDistance());
    //SmartDashboard.putNumber("Shooter L Temp", m_LeftShooterMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Shooter R Temp", m_RightShooterMotor.getMotorTemperature());
  }
  
  public void moveHood(double speed){
    if(speed > 0 && getHoodPosition() <= Constants.kHoodMinValue){
      speed = 0;
    }
    if(speed < 0 && getHoodPosition() >= Constants.kHoodMaxValue){
      speed = 0;
    }
    m_HoodMotor.set(speed);
  }
  public void moveHoodNoLimit(double speed){
    m_HoodMotor.set(speed);
  }
  public void setHoodPosition(double targetPosition){
    moveHood(mHoodPIDController.calculate(getHoodPosition(), targetPosition));
  }
  
  public double getHoodPosition(){
    double temp = (m_HoodEncoder.getDistance());
    if (temp<0){
      temp =0;
    }
    return temp;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    publishEncoderValues();
  }
}
