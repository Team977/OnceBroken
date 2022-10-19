// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto1BP1;
import frc.robot.commands.Auto1BP2;
import frc.robot.commands.Auto1BP3;
import frc.robot.commands.Auto2BP1;
import frc.robot.commands.Auto2BP2;
import frc.robot.commands.Auto3BP1;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutoIntakeEnd;
import frc.robot.commands.AutoIntakeStart;
import frc.robot.commands.ClimberControl;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.CloseLunaClaw;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.ExtendLunaClaw;
import frc.robot.commands.IntakeLoad;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.KickerKickIt;
import frc.robot.commands.KickerReverse;
import frc.robot.commands.KickerStop;
import frc.robot.commands.LimeLightDriveCommand;
import frc.robot.commands.LimelightActivate;
import frc.robot.commands.LimelightDeactivate;
import frc.robot.commands.ManualHoodMove;
import frc.robot.commands.OdometryDriveCommand;
import frc.robot.commands.OpenLunaClaw;
import frc.robot.commands.OpenLunaClawNoAdv;
import frc.robot.commands.RetractLunaClaw;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.SetShooterRPMLow;
import frc.robot.commands.SetShooterRPMTest;
import frc.robot.commands.SettleDown;
import frc.robot.commands.ShooterReverse;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LunaClimb;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private static double m_driveSpeedModifier = 0.5;
  private static boolean m_jukeMode = false;

  private static int m_climbStage = 0;

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  public static final Limelight m_Limelight = new Limelight();
  private final Shooter m_shooter = new Shooter();
  private final LunaClimb m_LunaClimb = new LunaClimb();
  private final static PneumaticHub m_pHub = new PneumaticHub();

  private final XboxController m_driver = new XboxController(0);
  private final static Joystick m_operator = new Joystick(1);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command m_auto1 = new Auto1(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  private final Command m_Auto1BP1 = new Auto1BP1(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  private final Command m_Auto1BP2 = new Auto1BP2(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  private final Command m_Auto1BP3 = new Auto1BP3(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  private final Command m_Auto2BP1 = new Auto2BP1(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  private final Command m_Auto2BP2 = new Auto2BP2(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  private final Command m_Auto3BP1 = new Auto3BP1(m_drivetrainSubsystem, m_shooter,m_Limelight,m_intake);
  

  
  //private final BooleanSupplier m_climbActive =  ()->m_operator.getRawButton(8);

  public RobotContainer() {

    m_chooser.setDefaultOption("Old Auto", m_auto1 );
    m_chooser.addOption("1BP1", m_Auto1BP1);
    m_chooser.addOption("1BP2", m_Auto1BP2);
    m_chooser.addOption("1BP3", m_Auto1BP3);
    m_chooser.addOption("2BP1", m_Auto2BP1);
    m_chooser.addOption("2BP2", m_Auto2BP2);
    m_chooser.addOption("3BP1", m_Auto3BP1);
    SmartDashboard.putData(m_chooser);

   // CameraServer.startAutomaticCapture();

    m_pHub.enableCompressorAnalog(100, 120);
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_climber.setDefaultCommand(new ClimberControl(m_climber));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    SmartDashboard.putNumber("HIGH RPM", 3000);
    SmartDashboard.putNumber("LOW RPM", 1500);

    // Configure the button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //DRIVER CONTROLS
    // Back button zeros the gyroscope
    new Button(m_driver::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    //intake
    new Button(m_driver::getRightBumper)
    .whenPressed(()->m_intake.DeployIntake())
    .whileHeld(new IntakeLoad(m_intake))
    .whenReleased(()->m_intake.RetractIntake())
    .whenReleased(new IntakeStop(m_intake));
    
    //speed mode?
    new Button(m_driver::getLeftBumper)
    .whenPressed(()->setDriveSpeedModifier(1.0))
    .whenReleased(()->setDriveSpeedModifier(0.5));


    //A button for testing?
//    new Button(m_driver::getAButton)

    //Manual Move hood up/down
    new Button(m_driver::getBButton)
    .whenPressed(new ManualHoodMove(m_shooter, 0.5))
    .whenReleased(new ManualHoodMove(m_shooter, 0.0));

    new Button(m_driver::getYButton)
    .whenPressed(new ManualHoodMove(m_shooter, -0.5))
    .whenReleased(new ManualHoodMove(m_shooter, 0.0));

    //High goal shot
      new RShooterTrigger()
        .whenActive(
          new SequentialCommandGroup(
            new ParallelCommandGroup(
              new KickerReverse(m_intake).withTimeout(Constants.shooterReverseTime),
              new ShooterReverse(m_shooter).withTimeout(Constants.shooterReverseTime)
            ),
            new SequentialCommandGroup(
              new LimelightActivate(m_Limelight),
              new WaitCommand(0.05),
              new ParallelCommandGroup(
                new LimeLightDriveCommand(
                  m_drivetrainSubsystem, 
                  () -> -modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                  () -> -modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                  () -> -modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                new SetShooterRPM(m_shooter, m_Limelight, true), 
                new SequentialCommandGroup(
                  new WaitCommand(Constants.kickItWaitTime),
                  new KickerKickIt(m_intake)
                )
              )
            )
          )
        )

        .whenInactive(
          new SequentialCommandGroup(
            new LimelightDeactivate(m_Limelight), 
            new ShooterStop(m_shooter), 
            new KickerStop(m_intake), 
            new IntakeStop(m_intake),
            new DefaultDriveCommand(m_drivetrainSubsystem,
              () -> -modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
          )
        );

      //low goal shot
      new LShooterTrigger()
        .whenActive(
          new SequentialCommandGroup(
            new ParallelCommandGroup(
              new KickerReverse(m_intake).withTimeout(Constants.shooterReverseTime),
              new ShooterReverse(m_shooter).withTimeout(Constants.shooterReverseTime)
            ),
            new ParallelCommandGroup(
              new SetShooterRPMLow(m_shooter), 
              new SequentialCommandGroup(
                new WaitCommand(Constants.kickItWaitTime),
                new KickerKickIt(m_intake)
              )
            )
          )
        )

        .whenInactive(
          new SequentialCommandGroup(
            new LimelightDeactivate(m_Limelight), 
            new ShooterStop(m_shooter), 
            new KickerStop(m_intake), 
            new IntakeStop(m_intake),
            new DefaultDriveCommand(m_drivetrainSubsystem,
              () -> -modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
          )
        );
        
      //Manual shot - doesn't use limelight to set hood & rpm, takes rpm from smartdashboard "High RPM"
      new Button(m_driver::getXButton)
        .whenPressed(
            new SequentialCommandGroup( 
              new ParallelCommandGroup(
                new KickerReverse(m_intake).withTimeout(Constants.shooterReverseTime),
                new ShooterReverse(m_shooter).withTimeout(Constants.shooterReverseTime)
              ),
              new ParallelCommandGroup(
                new SetShooterRPMTest(m_shooter, m_Limelight),
                new SequentialCommandGroup(
                  new WaitCommand(Constants.kickItWaitTime),
                  new KickerKickIt(m_intake)
            )
              )
            )
        )
        .whenReleased(
          new SequentialCommandGroup(
            new LimelightDeactivate(m_Limelight), 
            new ShooterStop(m_shooter), 
            new KickerStop(m_intake), 
            new IntakeStop(m_intake)
          )
        );

    //OPERATOR CONTROLS

    //Intake
    new JoystickButton(m_operator,1)
    .whenPressed(()->m_intake.DeployIntake())
    .whileHeld(new IntakeLoad(m_intake))
    .whenReleased(()->m_intake.RetractIntake())
    .whenReleased(new IntakeStop(m_intake));

    //Reverse Intake - spit ball out
    new JoystickButton(m_operator, 2)
    .whenPressed(()->m_intake.DeployIntake())
    .whileHeld(new IntakeReverse(m_intake))
    .whenReleased(()->m_intake.RetractIntake())
    .whenReleased(new IntakeStop(m_intake));
  
    //CLIMB SEQUENCE
    //Send climber to top, and open claw 
    new JoystickButton(m_operator, 6)
    .whenPressed(new ConditionalCommand(new ParallelCommandGroup(new ClimberUp(m_climber, 0.8),new OpenLunaClawNoAdv(m_LunaClimb)), new DoNothing(),()->RobotContainer.getClimbStage()==0));

    //AutoClimb, elevator down, grab, angle and extend
    new JoystickButton(m_operator, 5)
    .whenPressed(new ConditionalCommand(new AutoClimb(m_climber,m_LunaClimb, m_intake), new DoNothing(), ()->RobotContainer.getClimbStage()==1));

    //Opens the claw, swings down
    new JoystickButton(m_operator, 4)
    .whenPressed(new ConditionalCommand(new OpenLunaClaw(m_LunaClimb), new DoNothing(), ()->RobotContainer.getClimbStage()==2));

    //Once robot is settled, press to enable Autoclimb again
    new JoystickButton(m_operator, 7)
    .whenPressed(new ConditionalCommand(new SettleDown(), new DoNothing(), ()->RobotContainer.getClimbStage()==3));


    //Manual climb buttons
    //Close claw
    /*
    new JoystickButton(m_operator, 7)
    .whenPressed(new CloseLunaClaw(m_LunaClimb));
  
    //Angle the claw
    new JoystickButton(m_operator, 3)
    .whenPressed(new RetractLunaClaw(m_LunaClimb))
    .whenReleased(new ExtendLunaClaw(m_LunaClimb));
*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_chooser.getSelected();
    

  }

  public class RShooterTrigger extends Trigger{
      public boolean get(){
        return m_driver.getRightTriggerAxis()>0.5;
      }
  }

  public class LShooterTrigger extends Trigger{
    public boolean get(){
      return m_driver.getLeftTriggerAxis()>0.5;
    }
}

  public static Joystick getOperatorJoy(){
    return m_operator;
  }

  public static  void showAnalogPressure(){
    SmartDashboard.putNumber("Analog Pressure", m_pHub.getPressure(0));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static double getDriveSpeedModifier(){
    return m_driveSpeedModifier;
  }

  public static void setDriveSpeedModifier(double value){
    m_driveSpeedModifier = value;
  }

  public static boolean getJukeMode(){
    return m_jukeMode;
  }

  public static void setJukeMode(boolean bool){
    m_jukeMode = bool;
  }

  public static double getRPM(){
    return(SmartDashboard.getNumber("HIGH RPM", 3000));
  }
    
  public static double getLowRPM(){
    return(SmartDashboard.getNumber("LOW RPM", 2000));
  }

  public static int getClimbStage(){
    SmartDashboard.putNumber("Climb Stage", m_climbStage);
    return m_climbStage;
  }

  public static void incClimbStage(){
    m_climbStage++;
  }

  public static void setClimbStage(int number){
    m_climbStage = number;
  }
  public static void resetClimbStage(){
    m_climbStage = 0;
  }
}
