package frc.robot.subsystems.drive;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class GyroIONavX implements GyroIO
{
    private final AHRS          _navX = new AHRS(NavXComType.kMXP_SPI, (byte)Constants.Drive.ODOMETRY_FREQUENCY);
    private final Queue<Double> _yawPositionQueue;
    private final Queue<Double> _yawTimestampQueue;
    private final Queue<Double> _pitchPositionQueue;
    private final Queue<Double> _pitchTimestampQueue;
    private final Queue<Double> _rollPositionQueue;
    private final Queue<Double> _rollTimestampQueue;

    public GyroIONavX()
    {
        _yawPositionQueue    = OdometryThread.getInstance().registerSignal(_navX::getYaw);
        _yawTimestampQueue   = OdometryThread.getInstance().makeTimestampQueue();
        _pitchPositionQueue  = OdometryThread.getInstance().registerSignal(_navX::getPitch);
        _pitchTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        _rollPositionQueue   = OdometryThread.getInstance().registerSignal(_navX::getRoll);
        _rollTimestampQueue  = OdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs)
    {
        inputs.connected            = _navX.isConnected();
        inputs.yawPosition          = Rotation2d.fromDegrees(-_navX.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-_navX.getRawGyroZ());
        inputs.pitchPosition        = Rotation2d.fromDegrees(-_navX.getPitch());
        inputs.rollPosition         = Rotation2d.fromDegrees(-_navX.getRoll());

        inputs.odometryYawTimestamps   = _yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryPitchTimestamps = _pitchTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryRollTimestamps  = _rollTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

        inputs.odometryYawPositions   = _yawPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(-value)).toArray(Rotation2d[]::new);
        inputs.odometryPitchPositions = _pitchPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(-value)).toArray(Rotation2d[]::new);
        inputs.odometryRollPositions  = _rollPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(-value)).toArray(Rotation2d[]::new);

        _yawTimestampQueue.clear();
        _yawPositionQueue.clear();
        _pitchTimestampQueue.clear();
        _pitchPositionQueue.clear();
        _rollTimestampQueue.clear();
        _rollPositionQueue.clear();
    }
}
