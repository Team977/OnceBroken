// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class OdometryDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private double m_X;
  private double m_Y;
  private double m_rotationPose;
  private double x_move =0.0;
  private double y_move = 0.0;
  private double rot_move = 0.0;
  private PIDController xController;
  private PIDController yController;
  private PIDController rController;
  /** Creates a new OdometryDriveCommand. */
  public OdometryDriveCommand(DrivetrainSubsystem subsystem, double x, double y, double rot) {
    m_drivetrainSubsystem = subsystem;
    m_X = x;
    m_Y = y;
    m_rotationPose = rot;
    addRequirements(m_drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new PIDController(2.4, 0.08, 0);
    yController = new PIDController(2.4, 0.08, 0);
    rController = new PIDController(0.05, 0.001, 0);

    rController.enableContinuousInput(-180, 180);

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    rController.setTolerance(.2);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    x_move = xController.calculate(m_drivetrainSubsystem.getOdometryX(), m_X);
    y_move = yController.calculate(m_drivetrainSubsystem.getOdometryY(), m_Y);
    rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_rotationPose);
    //new idea: calculate, and then modify that value if it is abs>180
/*
    double temp = m_rotationPose - m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();

    if (temp<-180){
      rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()+360, m_rotationPose);
    }

    if (temp>180){
      rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()-360, m_rotationPose);
    }
/*
    rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_rotationPose);

    if (rot_move<-180){
      rot_move = rot_move+360;
    }
    if (rot_move>180){
      rot_move = rot_move-360;
    }

*/
    //need to do two calculates here: one for + and one for -?
    //OR: compare gyro degrees & rotationpose and decide which to modify before calculating
/*
    double currentAngle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
    if (Math.abs(currentAngle - m_rotationPose)>180){
      if(m_rotationPose>0){
        rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), 360+m_rotationPose);
      }else{
        rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), 360-m_rotationPose);
      }
    }else{
      
    rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_rotationPose);
    }

    //rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_rotationPose);
*/

    m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x_move,
                        y_move,
                        rot_move,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                ));

    SmartDashboard.putNumber("X Odo Error", xController.getPositionError());
    SmartDashboard.putNumber("Y Odo Error", yController.getPositionError());
    SmartDashboard.putNumber("R Odo Error", rController.getPositionError());

    SmartDashboard.putNumber("X Move", x_move);
    SmartDashboard.putNumber("Y Move", y_move);
    SmartDashboard.putNumber("R Move", rot_move);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
    //return Math.abs(xController.getPositionError())<0.1 && Math.abs(yController.getPositionError())<0.1 && Math.abs(rController.getPositionError())<2;
  }
}
