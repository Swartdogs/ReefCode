package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Module
{
    private final ModuleIO                 _io;
    private final ModuleIOInputsAutoLogged _inputs             = new ModuleIOInputsAutoLogged();
    private final int                      _index;
    private final SimpleMotorFeedforward   _driveFeedForward;
    private final PIDController            _driveFeedback;
    private final PIDController            _turnFeedback;
    private Rotation2d                     _angleSetpoint      = null;
    private Double                         _speedSetpoint      = null;
    private Rotation2d                     _turnRelativeOffset = null;
    private double                         _lastPositionMeters = 0.0;

    public Module(ModuleIO io, int index)
    {
        _io    = io;
        _index = index;

        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            case REAL:
            case REPLAY:
                _driveFeedForward = new SimpleMotorFeedforward(Constants.Drive.DRIVE_KS, Constants.Drive.DRIVE_KV);

                _driveFeedback = new PIDController(Constants.Drive.DRIVE_KP, 0, Constants.Drive.DRIVE_KD);
                _turnFeedback = new PIDController(Constants.Drive.TURN_KP, 0, Constants.Drive.TURN_KD);
                break;

            case SIM:
                _driveFeedForward = new SimpleMotorFeedforward(Constants.Drive.DRIVE_SIM_KS, Constants.Drive.DRIVE_SIM_KV);

                _driveFeedback = new PIDController(Constants.Drive.DRIVE_SIM_KP, 0, Constants.Drive.DRIVE_SIM_KD);
                _turnFeedback = new PIDController(Constants.Drive.TURN_SIM_KP, 0, Constants.Drive.TURN_SIM_KD);
                break;

            default:
                _driveFeedForward = new SimpleMotorFeedforward(0, 0);

                _driveFeedback = new PIDController(0, 0, 0);
                _turnFeedback = new PIDController(0, 0, 0);
                break;
        }

        _turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(_index), _inputs);

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (_turnRelativeOffset == null && _inputs.turnAbsolutePosition.getRadians() != 0.0)
        {
            _turnRelativeOffset = _inputs.turnAbsolutePosition.minus(_inputs.turnPosition);
        }

        // Run closed loop turn control
        if (_angleSetpoint != null)
        {
            _io.setTurnVolts(_turnFeedback.calculate(getAngle().getRadians(), _angleSetpoint.getRadians()));

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (_speedSetpoint != null)
            {
                // Scale velocity based on turn error
                // When the error is 90 degrees, the velocity setpoint should be 0. As the wheel
                // turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = _speedSetpoint * Math.cos(_turnFeedback.getError());

                // Run drive controller
                double velocityRadPerSec = adjustSpeedSetpoint / Constants.Drive.WHEEL_RADIUS;
                _io.setDriveVolts(_driveFeedForward.calculate(velocityRadPerSec) + _driveFeedback.calculate(_inputs.driveVelocityRadPerSec, velocityRadPerSec));
            }
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized
     * state.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state)
    {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        state.optimize(getAngle());

        // Update setpoints, controllers run in "periodic"
        _angleSetpoint = state.angle;
        _speedSetpoint = state.speedMetersPerSecond;

        return state;
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees
     */
    public void runCharacterization(double volts)
    {
        // Closed loop turn control
        _angleSetpoint = new Rotation2d();

        // Open loop drive control
        _io.setDriveVolts(volts);
        _speedSetpoint = null;
    }

    /** Disables all outputs to motors. */
    public void stop()
    {
        _io.setTurnVolts(0.0);
        _io.setDriveVolts(0.0);

        // Disable closed loop control for turn and drive
        _angleSetpoint = null;
        _speedSetpoint = null;
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled)
    {
        _io.setDriveBrakeMode(enabled);
        _io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle()
    {
        if (_turnRelativeOffset == null)
        {
            return new Rotation2d();
        }
        else
        {
            return _inputs.turnPosition.plus(_turnRelativeOffset);
        }
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters()
    {
        return _inputs.drivePositionRad * Constants.Drive.WHEEL_RADIUS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec()
    {
        return _inputs.driveVelocityRadPerSec * Constants.Drive.WHEEL_RADIUS;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module position delta since the last call to this method. */
    public SwerveModulePosition getPositionDelta()
    {
        var delta = new SwerveModulePosition(getPositionMeters() - _lastPositionMeters, getAngle());
        _lastPositionMeters = getPositionMeters();
        return delta;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity()
    {
        return _inputs.driveVelocityRadPerSec;
    }

    public void setAbsoluteEncoderOffset(Rotation2d moduleOffset)
    {
        _io.setAngleOffset(moduleOffset);
    }

    public void setDriveVolts(double volts)
    {
        _speedSetpoint = null;
        _io.setDriveVolts(volts);
    }
}
