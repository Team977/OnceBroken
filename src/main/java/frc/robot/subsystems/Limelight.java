// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
    
    NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("pipeline").setNumber(0);
    turnOffLEDs();
  
  }

  public boolean hasTarget(){
    if (NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("tv").getDouble(0)==1){
      return true;
    } else{
      return false;
    }
  }

  public double getHeading(){
    return NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("tx").getDouble(0);
  }

  public double getDistance(){
    return NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("ty").getDouble(0);
  }


  public double getHighRPM(){
      //return (-83.3*(getDistance()-20.5)+1283.3);
      //return (-87*getDistance()+2872);
      //return (-42*getDistance()+2750);
      return (-40*getDistance()+2770);
  }
  
  public double getLowRPM(){
    return 2400;
    //return (-83.3*(getDistance()-20.5)+1283.3);
    //return (-83.3*(getDistance()-20.5)+1283.3);
}

  public double getHighHoodPosition(){
   
    return (0.0065*getDistance()+.48);
   //return 0.27;
    //return -.72*getDistance()+.953;
   //return -getDistance()+80;
  }

  public double getLowHoodPosition(){
    return 0.4;
    //return -getDistance()+80;
  }
  public void turnOffLEDs(){
    NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("ledMode").setNumber(1);
  }

  public void turnOnLEDs(){
    NetworkTableInstance.getDefault().getTable("limelight-comets").getEntry("ledMode").setNumber(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LL TARGET", hasTarget());
    SmartDashboard.putNumber("LL heading", getHeading());
    SmartDashboard.putNumber("LL distance", getDistance());
  }
}
