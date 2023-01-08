package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.kinematics.SwerveModuleState;
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

        azimuthFx.config_kP(0, kP);
        azimuthFx.config_kI(0, kI);
        azimuthFx.config_kD(0, kD);

        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configSensorDirection(true);
        encoder.configMagnetOffset(magnetOffset);
    }

    public void setDesiredState(SwerveModuleState state){
        desiredState = CTREModule
    }
}
