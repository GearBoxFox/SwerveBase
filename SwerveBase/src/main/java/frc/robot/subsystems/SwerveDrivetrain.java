// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase{
  private SwerveModuleFalcon frModule;
  private SwerveModuleFalcon flModule;
  private SwerveModuleFalcon brModule;
  private SwerveModuleFalcon blModule;
  private PigeonIMU gyro;
  private int counter;

  private SwerveDriveOdometry odometry;
  /** Creates a new ExampleSubsystem. */
  public SwerveDrivetrain() {
    frModule = new SwerveModuleFalcon(0, Constants.kModFrOffset, Constants.kMod0Cans);
    flModule = new SwerveModuleFalcon(1, Constants.kModFlOffset, Constants.kMod1Cans);
    brModule = new SwerveModuleFalcon(2, Constants.kModBrOffset, Constants.kMod2Cans);
    blModule = new SwerveModuleFalcon(3, Constants.kModBlOffset, Constants.kMod3Cans);

    gyro = new PigeonIMU(15);
    odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, getGyro(), getModulePositions());
    counter = 0;
  }

  public void setModuleState(double xTranslation, double yTranslation, double zRotation, boolean fieldRelative){
    //Converts controller inputs to working chassis speeds, to working swerve module state array
    SwerveModuleState[] swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates( 
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      xTranslation, 
      yTranslation, 
      zRotation, 
      getGyro()
  )
  : new ChassisSpeeds(
      xTranslation, 
      yTranslation, 
      zRotation));

      //Pass respective values into module objects
      frModule.setDesiredState(swerveModuleStates[0]);
      flModule.setDesiredState(swerveModuleStates[1]);
      blModule.setDesiredState(swerveModuleStates[2]);
      brModule.setDesiredState(swerveModuleStates[3]);

      //Counter + logic for resetting to absolute
      counter++;
      if(counter == 200){
        setAbsoluteAngles();
        counter = 0;
      }
  }

  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePos = {new SwerveModulePosition(), 
      new SwerveModulePosition(), 
      new SwerveModulePosition(), 
      new SwerveModulePosition()};

    modulePos[0] = frModule.getPosition();
    modulePos[1] = flModule.getPosition();
    modulePos[2] = brModule.getPosition();
    modulePos[3] = blModule.getPosition();

    return modulePos;
  }

  public void setAbsoluteAngles(){
    frModule.resetToAbsolute();
    flModule.resetToAbsolute();
    blModule.resetToAbsolute();
    brModule.resetToAbsolute();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getGyro(), getModulePositions());

    SmartDashboard.putNumber("Fr Azimuth", frModule.getAzimuthAngle());
    SmartDashboard.putNumber("Fl Azimuth", flModule.getAzimuthAngle());
    SmartDashboard.putNumber("Br Azimuth", brModule.getAzimuthAngle());
    SmartDashboard.putNumber("Bl Azimuth", blModule.getAzimuthAngle());

    SmartDashboard.putNumber("Fr Setpoint", frModule.getTargetAngle());
    SmartDashboard.putNumber("Fl Setpoint", flModule.getTargetAngle());
    SmartDashboard.putNumber("Br Setpoint", brModule.getTargetAngle());
    SmartDashboard.putNumber("Bl Setpoint", blModule.getTargetAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
