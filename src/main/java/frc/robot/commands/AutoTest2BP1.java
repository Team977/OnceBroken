// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest2BP1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public AutoTest2BP1(DrivetrainSubsystem m_drive, Shooter m_shooter, Limelight m_limelight, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new SequentialCommandGroup(
        new SetOdometryAdjustment(m_drive, Constants.pos1X, Constants.pos1Y, Constants.pos1theta),
        
            new OdometryDriveCommand(m_drive, Constants.ball1Xpos,Constants.ball1Ypos, Constants.pos1theta).withTimeout(2.0),
          
            new OdometryDriveCommand(m_drive, Constants.approach2X,Constants.approach2Y, Constants.approach2theta).withTimeout(2.0),
         
          new OdometryDriveCommand(m_drive, Constants.ball2Xpos,Constants.ball2Ypos, Constants.approach2theta).withTimeout(2.0),

          new OdometryDriveCommand(m_drive, Constants.return2X,Constants.return2Y, Constants.return2theta).withTimeout(2.0)
          
        )
      
    );
  }
}
