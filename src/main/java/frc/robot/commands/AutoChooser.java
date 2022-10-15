// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoChooser extends SequentialCommandGroup {
  /** Creates a new auto2. */
  public AutoChooser(DrivetrainSubsystem m_drive, Shooter m_shooter, Limelight m_limelight, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    if (RobotContainer.getPosChoice() == 3.0) {
      addCommands(
        new SetOdometryAdjustment(m_drive, Constants.pos3X, Constants.pos3Y, Constants.pos3theta),
        new Auto1BP3(m_drive, m_shooter, m_limelight, m_intake)
      );
    }

    if (RobotContainer.getPosChoice() == 2.0 && RobotContainer.getBallChoice() == 1.0) {
      addCommands(
        new SetOdometryAdjustment(m_drive, Constants.pos2X, Constants.pos2Y, Constants.pos2theta),
        new Auto1BP2(m_drive, m_shooter, m_limelight, m_intake)
      );
    }
   
    if (RobotContainer.getPosChoice() == 2.0 && RobotContainer.getBallChoice() == 2.0) {
      addCommands(
        new SetOdometryAdjustment(m_drive, Constants.pos2X, Constants.pos2Y, Constants.pos2theta),
        new Auto2BP2(m_drive, m_shooter, m_limelight, m_intake)
      );
    }
   
    if (RobotContainer.getPosChoice() == 1.0 && RobotContainer.getBallChoice() == 1.0) {
      addCommands(
        new SetOdometryAdjustment(m_drive,  Constants.pos1X, Constants.pos1Y, Constants.pos1theta),
        new Auto1BP1(m_drive, m_shooter, m_limelight, m_intake)
      );
    }

    if (RobotContainer.getPosChoice() == 1.0 && RobotContainer.getBallChoice() == 2.0) {
      addCommands(
        new SetOdometryAdjustment(m_drive, Constants.pos1X, Constants.pos1Y, Constants.pos1theta),
        new Auto2BP1(m_drive, m_shooter, m_limelight, m_intake)
      );
    }

    if (RobotContainer.getPosChoice() == 1.0 && RobotContainer.getBallChoice() == 3.0) {
      addCommands(
        new SetOdometryAdjustment(m_drive, Constants.pos1X, Constants.pos1Y, Constants.pos1theta),
        new Auto3BP1(m_drive, m_shooter, m_limelight, m_intake)
      );
    }

/*
      new SetGyroAdjustment(m_drive, 40),
      new SetOdometryAdjustment(m_drive, 2, 2, 40),
      new AutoShoot(m_shooter,m_limelight, m_intake),
      new ParallelRaceGroup(
        new AutoIntakeStart(m_intake),
        new ProfiledOdometryDrive(m_drive, 3, 3, 40)),
      new AutoIntakeEnd(m_intake),
      new AutoShoot(m_shooter,m_limelight, m_intake),
      new ProfiledOdometryDrive(m_drive, 6, 4, 120),
      new ParallelRaceGroup(
        new AutoIntakeStart(m_intake),
        new ProfiledOdometryDrive(m_drive, 6, 5, 120)),
      new AutoIntakeEnd(m_intake),
      new ProfiledOdometryDrive(m_drive, 6, 5, 0),
      new AutoShoot(m_shooter,m_limelight, m_intake));
      */
  }
}
