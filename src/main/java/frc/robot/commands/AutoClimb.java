// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LunaClimb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb(Climber m_climber, LunaClimb m_lunaclimb, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new OpenLunaClaw(m_lunaclimb),
      //Run the climb down till hard limits
      new ClimberDown(m_climber, 0.8),
      //Close luna claws
      new CloseLunaClaw(m_lunaclimb),
      //Start raising climb &&  Retract luna
      new RetractLunaClaw(m_lunaclimb),
      //Raise climb til soft limit

      new ParallelRaceGroup(
        new DeployIntakeForClimb(m_intake),
      
     
        new ClimberUp(m_climber, 0.8)),


   

      //Extend luna
      new ExtendLunaClaw(m_lunaclimb),
      //run climb down, then release luna claw  (when to do this is the hard part -- manual control needed? or use position control?)
      new IncClimbStage()
    );
  }
}
