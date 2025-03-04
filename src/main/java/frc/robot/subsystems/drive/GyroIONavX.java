package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO
{
    private final AHRS _gyro = new AHRS(NavXComType.kUSB1);

    public GyroIONavX()
    {
        _gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs)
    {
        inputs.yawPosition          = Rotation2d.fromDegrees(-_gyro.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-_gyro.getRate());
        inputs.rollPosition         = Rotation2d.fromDegrees(_gyro.getRoll());
        inputs.pitchPosition        = Rotation2d.fromDegrees(_gyro.getPitch());
    }
}
