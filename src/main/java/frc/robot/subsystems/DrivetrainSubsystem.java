// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private static double gyro_angle_adjust = 0.0;
        /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = Constants.maxDriveVoltage;//BRB 6.0;//12
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private static final Translation2d m_frontLeftLocation =  new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  private static final Translation2d m_frontRightLocation =  new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0);
  private static final Translation2d m_backLeftLocation =  new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  private static final Translation2d m_backRightLocation =  new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0);


  private static Translation2d m_rotationCenter = new Translation2d(0.0,0.0);




  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          m_frontLeftLocation,
          // Front right
          m_frontRightLocation,
          // Back left
          m_backLeftLocation,
          // Back right
          m_backRightLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0)); //getGyroscopeRotation());



  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.

 
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    DrivetrainSubsystem.gyro_angle_adjust = 0.0;
    m_navx.zeroYaw();
    //m_navx.setAngleAdjustment(0.0);
  }

  public void publishAccel(){
          SmartDashboard.putNumber("AccelX", m_navx.getRawAccelX());
          SmartDashboard.putNumber("AccelY", m_navx.getRawAccelY());
          SmartDashboard.putNumber("AccelZ", m_navx.getRawAccelZ());
  }

  public double getAccelX(){
          return m_navx.getRawAccelX();
  }

  public Rotation2d getGyroscopeRotation() {

        //return Rotation2d.fromDegrees(-m_navx.getYaw());
        
        double tempAngle = -m_navx.getYaw()+DrivetrainSubsystem.gyro_angle_adjust;
        if (tempAngle < -180) tempAngle = tempAngle+360;
        if (tempAngle > 180) tempAngle = tempAngle-360;

        //return Rotation2d.fromDegrees(-m_navx.getYaw()+DrivetrainSubsystem.gyro_angle_adjust);
        
        return Rotation2d.fromDegrees(tempAngle);
        
        /*
    if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
        //return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        double tempAngle = -m_navx.getYaw()-DrivetrainSubsystem.gyro_angle_adjust;
        if (tempAngle < -180) tempAngle = tempAngle+360;
        if (tempAngle > 180) tempAngle = tempAngle-360;
        return Rotation2d.fromDegrees(tempAngle);
        //return Rotation2d.fromDegrees(-m_navx.getAngle());
        //return Rotation2d.fromDegrees(-m_navx.getYaw());
   }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
     //return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
     double tempAngle = -m_navx.getYaw()-DrivetrainSubsystem.gyro_angle_adjust;
     if (tempAngle < -180) tempAngle = tempAngle+360;
     if (tempAngle > 180) tempAngle = tempAngle-360;
     return Rotation2d.fromDegrees(tempAngle);
     //return Rotation2d.fromDegrees(-m_navx.getYaw());//-DrivetrainSubsystem.gyro_angle_adjust);
     //return Rotation2d.fromDegrees(-m_navx.getYaw());

     */
  }
  public Rotation2d getGyroscopeRotationManual() {

        return Rotation2d.fromDegrees(-m_navx.getYaw());

  }
  public void setGyroStart(double angle){
        DrivetrainSubsystem.gyro_angle_adjust = angle; //was negative
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }


  public double getGoalHeading(){
        return Math.atan2( m_odometry.getPoseMeters().getY(),  m_odometry.getPoseMeters().getX())/Math.PI*180; 
        
  }

  public double getGoalRotation(){
          return getGyroscopeRotation().getDegrees()-getGoalHeading();
  }
  
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
        m_odometry.update(
            getGyroscopeRotationManual(),
            m_frontLeftModule.getOdoState(),
            m_frontRightModule.getOdoState(),
            m_backLeftModule.getOdoState(),
            m_backRightModule.getOdoState());
      }

      public double getOdometryX(){
              return m_odometry.getPoseMeters().getX();
      }

      public double getOdometryY(){
        return m_odometry.getPoseMeters().getY();
}

        public void setOdometryOffset(double x, double y, double angle){
                m_odometry.resetPosition(new Pose2d(x,y, Rotation2d.fromDegrees(angle)), getGyroscopeRotation());
                //m_odometry.resetPosition(new Pose2d(x,y, Rotation2d.fromDegrees(angle)), getGyroscopeRotation());
                
        }


   public void setJukeLocation(jukeLocations loc){
        if (loc == jukeLocations.FRONTLEFT){
                m_rotationCenter = m_frontLeftLocation;
        } else if (loc==jukeLocations.FRONTRIGHT){
                m_rotationCenter = m_frontRightLocation;
        } else if (loc == jukeLocations.BACKLEFT){
                m_rotationCenter = m_backLeftLocation;
        } else if (loc == jukeLocations.BACKRIGHT){
                m_rotationCenter = m_backRightLocation;
        } else {
                m_rotationCenter = new Translation2d(0,0);
        }

   }

   public enum jukeLocations{
           FRONTLEFT,
           FRONTRIGHT,
           BACKLEFT,
           BACKRIGHT,
           CENTER
   }

  @Override
  public void periodic() {
        //SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds, m_rotationCenter);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

        // SmartDashboard.putNumber("FL state drive", states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);
        // SmartDashboard.putNumber("FL state rotation", states[0].angle.getRadians());


    updateOdometry();

    SmartDashboard.putNumber("Gyro Degrees", getGyroscopeRotation().getDegrees());
    //SmartDashboard.putNumber("Gyro Radians", getGyroscopeRotation().getRadians());
    SmartDashboard.putNumber("odometry X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry Y", m_odometry.getPoseMeters().getY());
SmartDashboard.putNumber("Gyro Offset", gyro_angle_adjust);

publishAccel();
    //SmartDashboard.putString("Juke Location", m_rotationCenter.toString());

    //SmartDashboard.putNumber("Compass", m_navx.getCompassHeading());

  }
}
