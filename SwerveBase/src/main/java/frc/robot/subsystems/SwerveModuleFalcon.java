package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public int moduleNumber;
    public double magnetOffset;

    public SwerveModuleFalcon(int moduleNumber, double magnetOffset, int[] canIds){
        this.magnetOffset = magnetOffset;
        this.moduleNumber = moduleNumber;

        driveFx = new TalonFX(canIds[0]);
        azimuthFx = new TalonFX(canIds[1]);
        encoder = new CANCoder(canIds[2]);

        driveFx.setNeutralMode(NeutralMode.Brake);
        driveFx.configOpenloopRamp(0.75);

        azimuthFx.config_kP(0, kP);
        azimuthFx.config_kI(0, kI);
        azimuthFx.config_kD(0, kD);
        azimuthFx.setNeutralMode(NeutralMode.Brake);
        azimuthFx.setInverted(true);
        resetToAbsolute();

        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configSensorDirection(true);
        encoder.configMagnetOffset(magnetOffset);
    }

    public void setDesiredState(SwerveModuleState state){
        SwerveModuleState desiredState = CTREModuleState.optimize(state, getState().angle);

        double percentOutput = desiredState.speedMetersPerSecond / 3.0; //This is swerve max speed , figure ths out
        driveFx.set(ControlMode.PercentOutput, percentOutput);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (3.0 * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        azimuthFx.set(ControlMode.Position, Conversions.degreesToFalcon(angle, 21.2)); 
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveFx.getSelectedSensorVelocity(), 1.5, 21.2);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(azimuthFx.getSelectedSensorPosition(), 21.2));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = (driveFx.getSelectedSensorPosition() / 4096) * Constants.kWheelCircumfrance;
        return new SwerveModulePosition(distance, getCanCoder());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - magnetOffset, 21.2);
        azimuthFx.setSelectedSensorPosition(absolutePosition);  
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    public double getTargetAngle() {
        return lastAngle;
    }
}
