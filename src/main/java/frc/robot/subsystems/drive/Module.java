package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static frc.robot.Constants.Drive.*;

public class Module
{
    private final ModuleIO                 _io;
    private final ModuleIOInputsAutoLogged _inputs            = new ModuleIOInputsAutoLogged();
    private final int                      _index;
    private final Alert                    _driveDisconnectedAlert;
    private final Alert                    _turnDisconnectedAlert;
    private SwerveModulePosition[]         _odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index)
    {
        _io                     = io;
        _index                  = index;
        _driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + Integer.toString(_index) + ".", AlertType.kError);
        _turnDisconnectedAlert  = new Alert("Disconnected turn motor on module " + Integer.toString(_index) + ".", AlertType.kError);
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(_index), _inputs);

        // Calculate positions for odometry
        int sampleCount = _inputs.odometryTimestamps.length;
        _odometryPositions = new SwerveModulePosition[sampleCount];

        for (int i = 0; i < sampleCount; i++)
        {
            double     positionMeters = _inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
            Rotation2d angle          = _inputs.odometryTurnPositions[i];
            _odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        _driveDisconnectedAlert.set(!_inputs.driveConnected);
        _turnDisconnectedAlert.set(!_inputs.turnConnected);
    }

    public void runSetpoint(SwerveModuleState state)
    {
        state.optimize(getAngle());
        state.cosineScale(_inputs.turnPosition);

        _io.setDriveVelocity(state.speedMetersPerSecond / WHEEL_RADIUS);
        _io.setTurnPosition(state.angle);
    }

    public void runCharacterization(double output)
    {
        _io.setDriveOpenLoop(output);
        _io.setTurnPosition(new Rotation2d());
    }

    public void stop()
    {
        _io.setDriveOpenLoop(0.0);
        _io.setTurnOpenLoop(0.0);
    }

    public Rotation2d getAngle()
    {
        return _inputs.turnPosition;
    }

    public double getPositionMeters()
    {
        return _inputs.drivePositionRad * WHEEL_RADIUS;
    }

    public double getVelocityMetersPerSec()
    {
        return _inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions()
    {
        return _odometryPositions;
    }

    public double[] getOdometryTimestamps()
    {
        return _inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharacterizationPosition()
    {
        return _inputs.drivePositionRad;
    }

    public double getFFCharacterizationVelocity()
    {
        return Units.radiansToRotations(_inputs.driveVelocityRadPerSec);
    }
}
