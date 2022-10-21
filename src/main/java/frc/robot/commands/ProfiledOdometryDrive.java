// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ProfiledOdometryDrive extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private double m_X;
  private double m_Y;
  private double m_rotationPose;
  private double x_move =0.0;
  private double y_move = 0.0;
  private double rot_move = 0.0;
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController rController;

  private Constraints m_Constraints = new TrapezoidProfile.Constraints(0.0025, 0.0025);
  /** Creates a new OdometryDriveCommand. */
  public ProfiledOdometryDrive(DrivetrainSubsystem subsystem, double x, double y, double rot) {
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
    xController = new ProfiledPIDController(.25, 0.05, 0, m_Constraints);
    yController = new ProfiledPIDController(.25, 0.05, 0, m_Constraints);
    rController = new ProfiledPIDController(.0150, 0.001, 0, m_Constraints);

    
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rController.setTolerance(2);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    x_move = xController.calculate(m_drivetrainSubsystem.getOdometryX(), m_X);
    y_move = yController.calculate(m_drivetrainSubsystem.getOdometryY(), m_Y);
    rot_move = rController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(), m_rotationPose);


    m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x_move,
                        y_move,
                        rot_move,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                ));

    //SmartDashboard.putNumber("X Odo Error", xController.getPositionError());
    //SmartDashboard.putNumber("Y Odo Error", yController.getPositionError());
    //SmartDashboard.putNumber("R Odo Error", rController.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && rController.atGoal();
  }
}
