package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Conversions;
public class SwerveModuleFalcon {
    private TalonFX driveFx;
    private TalonFX azimuthFx;
    private CANCoder encoder;
    private double lastAngle;

    private double kP;
    private double kI;
    private double kD;

    // private double cruiseVelocity = 4000.0;
    // private double maxAcceleration = 4000.0;

    public int moduleNumber;
    public double magnetOffset;
    private int count = 0;

    public SwerveModuleFalcon(int moduleNumber, double magnetOffset, int[] canIds){
        this.magnetOffset = magnetOffset;
        this.moduleNumber = moduleNumber;

        driveFx = new TalonFX(canIds[0]);
        azimuthFx = new TalonFX(canIds[1]);
        encoder = new CANCoder(canIds[2]);

        driveFx.setNeutralMode(NeutralMode.Brake);
        driveFx.configOpenloopRamp(0.75);

        kP = 0.16;
        kI = 0.00;
        kD = 3;

        azimuthFx.configFactoryDefault();
        azimuthFx.config_kP(0, kP);
        azimuthFx.config_kI(0, kI);
        azimuthFx.config_kD(0, kD);
        azimuthFx.setNeutralMode(NeutralMode.Brake);
        azimuthFx.setInverted(false);
        azimuthFx.setSensorPhase(false);
        azimuthFx.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        
        encoder.configFactoryDefault();
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configSensorDirection(true);
        encoder.configMagnetOffset(magnetOffset);
        
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState state){
        SwerveModuleState desiredState = CTREModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuthAngle()));
        // SwerveModuleState desiredState = state; //SwerveModuleState.optimize(state, getCanCoder());

        double percentOutput = desiredState.speedMetersPerSecond / 1.0 * 0.2; //This is swerve max speed , figure ths out
        

        double angle = Conversions.degreesToFalcon(desiredState.angle.getDegrees(), Constants.kTurningRatio); 

        if(desiredState.speedMetersPerSecond <= 0.1 && count == 200){
            resetToAbsolute();
        }

        if(desiredState.speedMetersPerSecond > 0.1){        
            azimuthFx.set(ControlMode.Position, angle); 
        }
        driveFx.set(ControlMode.PercentOutput, percentOutput);
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveFx.getSelectedSensorVelocity(), Constants.kWheelCircumfrance, Constants.kDriveGearRation);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(azimuthFx.getSelectedSensorPosition(), Constants.kTurningRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = (driveFx.getSelectedSensorPosition() / 4096) * Constants.kWheelCircumfrance;
        return new SwerveModulePosition(distance, getCanCoder());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(encoder.getAbsolutePosition(), Constants.kTurningRatio);
        azimuthFx.setSelectedSensorPosition(absolutePosition);  
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    public double getAzimuthAngle(){
        return Conversions.falconToDegrees(azimuthFx.getSelectedSensorPosition(), Constants.kTurningRatio);
    }

    public double getTargetAngle() {
        return lastAngle;
    }
}
