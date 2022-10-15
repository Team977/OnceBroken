package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.jukeLocations;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        double x = m_translationXSupplier.getAsDouble()*RobotContainer.getDriveSpeedModifier();
        double y = m_translationYSupplier.getAsDouble()*RobotContainer.getDriveSpeedModifier();
        double rot = m_rotationSupplier.getAsDouble()*Constants.rotationModifier;


        if (RobotContainer.getJukeMode()){
            if (x > 0  && y > 0){
                m_drivetrainSubsystem.setJukeLocation(jukeLocations.FRONTLEFT);
            } else if (x < 0  && y > 0){
                m_drivetrainSubsystem.setJukeLocation(jukeLocations.FRONTRIGHT);
            } else if (x < 0  && y > 0){
                m_drivetrainSubsystem.setJukeLocation(jukeLocations.BACKLEFT);
            } else if (x < 0  && y < 0){
                m_drivetrainSubsystem.setJukeLocation(jukeLocations.BACKRIGHT);
            } else{
                m_drivetrainSubsystem.setJukeLocation(jukeLocations.CENTER);
            }
        }


        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        rot,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );

        //SmartDashboard.putNumber("X input", x);     
        //SmartDashboard.putNumber("Y input", y);
        //SmartDashboard.putNumber("rot input", rot);
        //SmartDashboard.putBoolean("Juke Mode", RobotContainer.getJukeMode());
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
