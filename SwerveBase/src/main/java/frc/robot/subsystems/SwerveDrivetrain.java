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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase implements SwerveIO{
  private SwerveModuleFalcon frModule;
  private SwerveModuleFalcon flModule;
  private SwerveModuleFalcon brModule;
  private SwerveModuleFalcon blModule;
  private PigeonIMU gyro;

  private SwerveDriveOdometry odometry;
  /** Creates a new ExampleSubsystem. */
  public SwerveDrivetrain() {
    frModule = new SwerveModuleFalcon(0, Constants.kMod0Offset, Constants.kMod0Cans);
    flModule = new SwerveModuleFalcon(1, Constants.kMod1Offset, Constants.kMod1Cans);
    brModule = new SwerveModuleFalcon(2, Constants.kMod2Offset, Constants.kMod2Cans);
    blModule = new SwerveModuleFalcon(3, Constants.kMod3Offset, Constants.kMod3Cans);

    gyro = new PigeonIMU(15);
    odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, getGyro(), getModulePositions());
  }

  @Override
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
  }

  @Override
  public void updateInputs(SwerveIOInput inputs){
    //Updates modules states for AdvantageKit logging
    inputs.frState = frModule.getState();
    inputs.flState = flModule.getState();
    inputs.brState = brModule.getState();
    inputs.blState = blModule.getState();
    
    //Update gyro for logging
    inputs.gyroYaw = getGyro().getDegrees();
  }

  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePos = {new SwerveModulePosition()};

    modulePos[0] = frModule.getPosition();
    modulePos[1] = flModule.getPosition();
    modulePos[2] = brModule.getPosition();
    modulePos[3] = blModule.getPosition();

    return modulePos;
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
