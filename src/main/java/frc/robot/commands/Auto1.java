// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public Auto1(DrivetrainSubsystem m_drive, Shooter m_shooter, Limelight m_limelight, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new AutoIntakeStart(m_intake),
            new OdometryDriveCommand(m_drive, 1.9,0.0, 0.0).withTimeout(2.0)
          ),
          new AutoIntakeEnd(m_intake),
          new LimelightActivate(m_limelight),
          new OdometryDriveCommand(m_drive, 1.2,0.0, 0.0).withTimeout(0.5),
          new LimeLightDriveCommand(m_drive, ()->0.0, ()->0.0, ()->0.0).withTimeout(1.0)
        ),
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new KickerReverse(m_intake).withTimeout(0.25),
            new ShooterReverse(m_shooter).withTimeout(0.25),
            new LimelightActivate(m_limelight)
          ), 
          new WaitCommand(0.1),
          new ParallelRaceGroup( 
            new SetShooterRPM(m_shooter, m_limelight, true).withTimeout(3.5), //BRB used SetShooterRPMTest here
            new SequentialCommandGroup(
              new WaitCommand(0.75),
              new KickerKickIt(m_intake), 
              new WaitCommand(3.0)
            )
          )
        ),
        new SequentialCommandGroup(
          new LimelightDeactivate(m_limelight), 
          new ShooterStop(m_shooter), 
          new KickerStop(m_intake), 
          new IntakeStop(m_intake)
        )
      )
    );
  }
}
