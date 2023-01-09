package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {
    @AutoLog
    public class SwerveIOInput{
        public SwerveModuleState frState = new SwerveModuleState();
        public SwerveModuleState flState = new SwerveModuleState();
        public SwerveModuleState brState = new SwerveModuleState();
        public SwerveModuleState blState = new SwerveModuleState();

        public double gyroYaw = 0.0;
    }

    public default void updateInputs(SwerveIOInput inputs){
        /**update the drivetrains inputs */
    }

    public default void setModuleState(double xTranslation, double yTranslation, double zRotation, boolean fieldRelative){
        /**Update module states based on driver inpuy */
    }
}
