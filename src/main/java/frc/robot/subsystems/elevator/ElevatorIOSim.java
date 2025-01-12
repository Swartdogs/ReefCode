package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO
{
    private final DCMotorSim _driveSim;
    private final DCMotorSim _turnSim;
    private boolean          _driveClosedLoop          = false;
    private boolean          _turnClosedLoop           = false;
    private PIDController    _driveController          = new PIDController(Constants.Drive.DRIVE_SIM_P, 0, Constants.Drive.DRIVE_SIM_D);
    private PIDController    _turnController           = new PIDController(Constants.Drive.TURN_SIM_P, 0, Constants.Drive.TURN_SIM_D);
    private double           _driveFFVolts             = 0.0;
    private double           _driveAppliedVolts        = 0.0;
    private double           _turnAppliedVolts         = 0.0;
    private Rotation2d       _turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

    public ElevatorIOSim()
    {
        _driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Drive.DRIVE_GEARBOX, 0.025, Constants.Drive.DRIVE_MOTOR_REDUCTION), Constants.Drive.DRIVE_GEARBOX);
        _turnSim  = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Drive.TURN_GEARBOX, 0.004, Constants.Drive.TURN_MOTOR_REDUCTION), Constants.Drive.TURN_GEARBOX);

        _turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        if (_driveClosedLoop)
        {
            _driveAppliedVolts = _driveFFVolts + _driveController.calculate(_driveSim.getAngularVelocityRadPerSec());
        }
        else
        {
            _driveController.reset();
        }

        if (_turnClosedLoop)
        {
            _turnAppliedVolts = _turnController.calculate(_turnSim.getAngularPositionRad());
        }
        else
        {
            _turnController.reset();
        }

        _driveSim.setInput(MathUtil.clamp(_driveAppliedVolts, -12.0, 12.0));
        _turnSim.setInput(MathUtil.clamp(_turnAppliedVolts, -12.0, 12.0));
        _driveSim.update(0.02);
        _turnSim.update(0.02);

        inputs.driveConnected         = true;
        inputs.drivePositionRad       = _driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = _driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts      = _driveAppliedVolts;
        inputs.driveCurrentAmps       = Math.abs(_driveSim.getCurrentDrawAmps());

        inputs.turnConnected         = true;
        inputs.turnAbsolutePosition  = new Rotation2d(_turnSim.getAngularPositionRad()).plus(_turnAbsoluteInitPosition);
        inputs.turnPosition          = new Rotation2d(_turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = _turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts      = _turnAppliedVolts;
        inputs.turnCurrentAmps       = Math.abs(_turnSim.getCurrentDrawAmps());

        inputs.odometryTimestamps        = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
        inputs.odometryTurnPositions     = new Rotation2d[] { inputs.turnPosition };
    }

    @Override
    public void setDriveOpenLoop(double output)
    {
        _driveClosedLoop   = false;
        _driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output)
    {
        _turnClosedLoop   = false;
        _turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec)
    {
        _driveClosedLoop = true;
        _driveFFVolts    = Constants.Drive.DRIVE_SIM_KS * Math.signum(velocityRadPerSec) + Constants.Drive.DRIVE_SIM_KV * velocityRadPerSec;
        _driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation)
    {
        _turnClosedLoop = true;
        _turnController.setSetpoint(rotation.getRadians());
    }
}
