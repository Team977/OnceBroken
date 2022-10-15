// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberControl extends CommandBase {

  private Climber m_climber;
  private double m_Joystick;
  //private Boolean m_active;

  /** Creates a new ClimberControl. */
  public ClimberControl(Climber subsystem) {

    m_climber = subsystem;
    
    //m_active = active;

    //m_active = m_Joystick.getRawButton(8);

    addRequirements(m_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //if (m_active){
    m_Joystick = RobotContainer.getOperatorJoy().getRawAxis(1);
    double value = m_Joystick;
    double deadband = 0.15;

    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        value = (value - deadband) / (1.0 - deadband);
      } else {
        value = (value + deadband) / (1.0 - deadband);
      }
    } else {
     value =  0.0;
    }
//SmartDashboard.putNumber("Climb value" , value);
    m_climber.moveSol(value);
  }
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.moveSol(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
