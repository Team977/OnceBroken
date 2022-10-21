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
public class Auto2BP2 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public Auto2BP2(DrivetrainSubsystem m_drive, Shooter m_shooter, Limelight m_limelight, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new SetOdometryAdjustment(m_drive, Constants.pos2X, Constants.pos2Y, Constants.pos2theta),
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new AutoIntakeStart(m_intake),
            new OdometryDriveCommand(m_drive, Constants.ball2Xpos,Constants.ball2Ypos, Constants.pos2theta).withTimeout(2.0)
          ),
          new AutoIntakeEnd(m_intake),
          new LimelightActivate(m_limelight),
          //new OdometryDriveCommand(m_drive, 1.2,0.0, 0.0).withTimeout(0.5),
          new LimeLightDriveCommand(m_drive, ()->0.0, ()->0.0, ()->0.0).withTimeout(1.0)
        ),
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new KickerReverse(m_intake).withTimeout(Constants.shooterReverseTime),
            new ShooterReverse(m_shooter).withTimeout(Constants.shooterReverseTime),
            new LimelightActivate(m_limelight)
          ), 
          new WaitCommand(0.1),
          new ParallelRaceGroup( 
            new SetShooterRPM(m_shooter, m_limelight, true).withTimeout(3.5), //BRB used SetShooterRPMTest here
            new SequentialCommandGroup(
              new WaitCommand(Constants.kickItWaitTime),
              new KickerKickIt(m_intake), 
              new WaitCommand(Constants.shooterAutoTime)
            )
          )
        ),
        new SequentialCommandGroup(
          new LimelightDeactivate(m_limelight), 
          new ShooterStop(m_shooter), 
          new KickerStop(m_intake), 
          new IntakeStop(m_intake)
        ),

        new SequentialCommandGroup(
          new OdometryDriveCommand(m_drive, Constants.approach4X,Constants.approach4Y, Constants.approach4theta).withTimeout(3.0),
          new ParallelRaceGroup(
            new AutoIntakeStart(m_intake),
            new OdometryDriveCommand(m_drive, Constants.ball4Xpos,Constants.ball4Ypos, Constants.approach4theta).withTimeout(2.0)
          ),
          new AutoIntakeEnd(m_intake),
          new OdometryDriveCommand(m_drive, Constants.return2X,Constants.return2Y, Constants.return2theta).withTimeout(2.0),
          new LimelightActivate(m_limelight),
          //new OdometryDriveCommand(m_drive, 1.2,0.0, 0.0).withTimeout(0.5),
          new LimeLightDriveCommand(m_drive, ()->0.0, ()->0.0, ()->0.0).withTimeout(1.0)
        ),
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new KickerReverse(m_intake).withTimeout(Constants.shooterReverseTime),
            new ShooterReverse(m_shooter).withTimeout(Constants.shooterReverseTime),
            new LimelightActivate(m_limelight)
          ), 
          new WaitCommand(0.1),
          new ParallelRaceGroup( 
            new SetShooterRPM(m_shooter, m_limelight, true).withTimeout(3.5), //BRB used SetShooterRPMTest here
            new SequentialCommandGroup(
              new WaitCommand(Constants.kickItWaitTime),
              new KickerKickIt(m_intake), 
              new WaitCommand(Constants.shooterAutoTime)
            )
          )
        ),
        new SequentialCommandGroup(
          new LimelightDeactivate(m_limelight), 
          new ShooterStop(m_shooter), 
          new KickerStop(m_intake), 
          new IntakeStop(m_intake)
         // new SetGyroAdjustmentTeleop(m_drive,Constants.pos2theta)
        )
      )
    );
  }
}
