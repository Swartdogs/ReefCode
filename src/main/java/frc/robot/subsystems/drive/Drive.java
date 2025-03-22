package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Camera;

public class Drive extends SubsystemBase
{
    private static Drive _instance;

    public static Drive getInstance()
    {
        if (_instance == null)
        {
            GyroIO   gyroIO;
            ModuleIO flIO, frIO, blIO, brIO;

            switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL:
                    gyroIO = new GyroIONavX();
                    flIO = new ModuleIOHardware(0);
                    frIO = new ModuleIOHardware(1);
                    blIO = new ModuleIOHardware(2);
                    brIO = new ModuleIOHardware(3);
                    break;

                case SIM:
                    gyroIO = new GyroIOSim(() -> _instance.getChassisSpeeds());
                    flIO = new ModuleIOSim();
                    frIO = new ModuleIOSim();
                    blIO = new ModuleIOSim();
                    brIO = new ModuleIOSim();
                    break;

                default:
                    gyroIO = new GyroIO() {};
                    flIO = new ModuleIO() {};
                    frIO = new ModuleIO() {};
                    blIO = new ModuleIO() {};
                    brIO = new ModuleIO() {};
                    break;
            }

            _instance = new Drive(gyroIO, flIO, frIO, blIO, brIO);
        }

        return _instance;
    }

    private final GyroIO                   _gyroIO;
    private final GyroIOInputsAutoLogged   _gyroInputs        = new GyroIOInputsAutoLogged();
    private final Module[]                 _modules           = new Module[4]; // FL, FR, BL, BR
    private final SwerveDrivePoseEstimator _poseEstimator;
    private final SwerveDrivePoseEstimator _localPoseEstimator;
    private final SwerveDriveKinematics    _kinematics        = new SwerveDriveKinematics(Constants.Drive.MODULE_TRANSLATIONS);
    private final PIDController            _headingController = new PIDController(Constants.Choreo.TURN_KP, 0, Constants.Choreo.TURN_KD);
    private final PIDController            _xController       = new PIDController(Constants.Choreo.DRIVE_KP, 0, Constants.Choreo.DRIVE_KD);
    private final PIDController            _yController       = new PIDController(Constants.Choreo.DRIVE_KP, 0, Constants.Choreo.DRIVE_KD);
    private final PIDController            _xDrivePID;
    private final PIDController            _yDrivePID;
    private final PIDController            _rotatePID;
    private double                         _xDriveMaxSpeed;
    private double                         _yDriveMaxSpeed;
    private double                         _rotateMaxSpeed;
    private double                         _speedMultiplier;
    private SysIdRoutine                   _sysId;

    private Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO)
    {
        _gyroIO = gyroIO;

        _modules[0] = new Module(flModuleIO, 0);
        _modules[1] = new Module(frModuleIO, 1);
        _modules[2] = new Module(blModuleIO, 2);
        _modules[3] = new Module(brModuleIO, 3);

        _headingController.enableContinuousInput(-Math.PI, Math.PI);

        _xDrivePID = new PIDController(Constants.Drive.TRANSLATE_KP, Constants.Drive.TRANSLATE_KI, Constants.Drive.TRANSLATE_KD);
        _yDrivePID = new PIDController(Constants.Drive.TRANSLATE_KP, Constants.Drive.TRANSLATE_KI, Constants.Drive.TRANSLATE_KD);

        SmartDashboard.putData("xDrive", _xDrivePID);
        SmartDashboard.putData("yDrive", _yDrivePID);

        _rotatePID = new PIDController(Constants.Drive.ROTATE_KP, 0, Constants.Drive.ROTATE_KD);

        _xDrivePID.setIZone(1);
        _yDrivePID.setIZone(1);

        _xDrivePID.setTolerance(Units.inchesToMeters(1.5));
        _yDrivePID.setTolerance(Units.inchesToMeters(1.5));
        _rotatePID.setTolerance(Units.degreesToRadians(3.0));

        _rotatePID.enableContinuousInput(-Math.PI, Math.PI);

        _speedMultiplier = 1;

        _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, new Rotation2d(), getModulePositions(), new Pose2d());
        _localPoseEstimator = new SwerveDrivePoseEstimator(_kinematics, new Rotation2d(), getModulePositions(), new Pose2d());

        _sysId = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())), new SysIdRoutine.Mechanism((voltage) -> runCharacterizationVolts(voltage.in(Volts)), null, this)
        );
    }

    @Override
    public void periodic()
    {
        _gyroIO.updateInputs(_gyroInputs);
        Logger.processInputs("Drive/Gyro", _gyroInputs);

        for (var module : _modules)
        {
            module.periodic();
        }

        if (DriverStation.isDisabled())
        {
            // Stop moving when disabled
            for (var module : _modules)
            {
                module.stop();
            }

            // Log empty setpoint states when disabled
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        _poseEstimator.update(_gyroInputs.yawPosition, getModulePositions());

        Logger.recordOutput("Odometry/Robot", _poseEstimator.getEstimatedPosition());
        Logger.recordOutput("SwerveStates/Measured", getModuleStates());
        Logger.recordOutput("AutoAlign/xAtSetpoint", xDriveIsFinished());
        Logger.recordOutput("AutoAlign/yAtSetpoint", yDriveIsFinished());
        Logger.recordOutput("AutoAlign/rotateAtSetpoint", rotateIsFinished());
    }

    /**
     * Runs the drive at the desired velocity
     * 
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds)
    {
        speeds = speeds.times(_speedMultiplier);

        // Calculate module setpoints
        ChassisSpeeds       discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.General.LOOP_PERIOD_SECS);
        SwerveModuleState[] setpointStates = _kinematics.toSwerveModuleStates(discreteSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Drive.MAX_LINEAR_SPEED);

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[_modules.length];

        for (int i = 0; i < _modules.length; i++)
        {
            if (Math.hypot(discreteSpeeds.vxMetersPerSecond, discreteSpeeds.vyMetersPerSecond) > Constants.Drive.SPEED_MOTION_THRESHOLD || Math.abs(discreteSpeeds.omegaRadiansPerSecond) > Constants.Drive.ROTATION_MOTION_THRESHOLD)
            {
                optimizedSetpointStates[i] = _modules[i].runSetpoint(setpointStates[i]);
            }
            else
            {
                _modules[i].stop();
                optimizedSetpointStates[i] = new SwerveModuleState();
            }
        }

        // Log optimized setpoint states
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    public Pose2d updateLocalPose(int tag)
    {
        double angle = 180 - Constants.Field.getTagAngle(tag).getDegrees();
        double angleTwo = angle + _gyroInputs.yawPosition.getDegrees();

        return _localPoseEstimator.update(Rotation2d.fromDegrees(angleTwo), getModulePositions());
    }

    public void followTrajectory(SwerveSample sample)
    {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + _xController.calculate(pose.getX(), sample.x), sample.vy + _yController.calculate(pose.getY(), sample.y), sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()));
    }

    public void runVolts(double volts)
    {
        for (int i = 0; i < _modules.length; i++)
        {
            var optimized = _modules[i].runSetpoint(new SwerveModuleState());

            var speed = volts;

            if (optimized.angle.getDegrees() != 0)
            {
                speed *= -1;
            }
            _modules[i].setDriveVolts(speed);
        }
    }

    /** Stops the drive. */
    public void stop()
    {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their normal orientations the next time a nonzero
     * velocity is requested.
     */
    public void stopWithX()
    {
        Rotation2d[] headings = new Rotation2d[_modules.length];

        for (int i = 0; i < _modules.length; i++)
        {
            headings[i] = Constants.Drive.MODULE_TRANSLATIONS[i].getAngle();
        }

        _kinematics.resetHeadings(headings);
        stop();
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts)
    {
        for (var module : _modules)
        {
            module.runCharacterization(volts);
        }
    }

    /** Returns the average drive velocity in radians/sec */
    public double getFFCharacterizationVelocity()
    {
        double driveVelocityAverage = 0.0;

        for (var module : _modules)
        {
            driveVelocityAverage += module.getFFCharacterizationVelocity();
        }

        return driveVelocityAverage / _modules.length;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp)
    {
        _poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules
     */
    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[_modules.length];

        for (int i = 0; i < _modules.length; i++)
        {
            states[i] = _modules[i].getState();
        }

        return states;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose()
    {
        return _poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return _poseEstimator.getEstimatedPosition().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose)
    {
        _poseEstimator.resetPosition(_gyroInputs.yawPosition, getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[_modules.length];

        for (int i = 0; i < _modules.length; i++)
        {
            wheelPositions[i] = _modules[i].getPosition();
        }

        return wheelPositions;
    }

    public ChassisSpeeds getChassisSpeeds()
    {
        return _kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions()
    {
        double[] values = new double[_modules.length];

        for (int i = 0; i < _modules.length; i++)
        {
            values[i] = _modules[i].getWheelRadiusCharacterizationPosition();
        }

        return values;
    }

    public void setModuleAbsoluteEncoderOffset(int moduleIndex, Rotation2d offset)
    {
        _modules[moduleIndex].setAbsoluteEncoderOffset(offset);
    }

    public void xDriveInit(double setpoint, double maxSpeed)
    {
        _xDriveMaxSpeed = Math.abs(maxSpeed);

        _xDrivePID.setSetpoint(setpoint);
    }

    public void yDriveInit(double setpoint, double maxSpeed)
    {
        _yDriveMaxSpeed = Math.abs(maxSpeed);

        _yDrivePID.setSetpoint(setpoint);
    }

    public void rotateInit(Rotation2d setpoint, double maxSpeed)
    {
        _rotateMaxSpeed = Math.abs(maxSpeed);

        _rotatePID.setSetpoint(setpoint.getRadians());
    }

    public double xDriveExecute()
    {
        Pose2d pose = getPose();

        if (Vision.getInstance(Camera.Front).hasTarget())
        {
            pose = Vision.getInstance(Camera.Front).getEstimatedPose();
        }

        return MathUtil.clamp(_xDrivePID.calculate(pose.getX()), -_xDriveMaxSpeed, _xDriveMaxSpeed);
    }

    public double yDriveExecute()
    {
        Pose2d pose = getPose();

        if (Vision.getInstance(Camera.Front).hasTarget())
        {
            pose = Vision.getInstance(Camera.Front).getEstimatedPose();
        }

        return MathUtil.clamp(_yDrivePID.calculate(pose.getY()), -_yDriveMaxSpeed, _yDriveMaxSpeed);
    }

    public double rotateExecute()
    {
        Rotation2d rotation = getRotation();

        if (Vision.getInstance(Camera.Front).hasTarget())
        {
            rotation = Vision.getInstance(Camera.Front).getEstimatedPose().getRotation();
        }

        return MathUtil.clamp(_rotatePID.calculate(rotation.getRadians()), -_rotateMaxSpeed, _rotateMaxSpeed);
    }

    public double rotateExecute(Rotation2d setpoint)
    {
        Rotation2d rotation = getRotation();

        if (Vision.getInstance(Camera.Front).hasTarget())
        {
            rotation = Vision.getInstance(Camera.Front).getEstimatedPose().getRotation();
        }

        return MathUtil.clamp(_rotatePID.calculate(rotation.getRadians(), setpoint.getRadians()), -_rotateMaxSpeed, _rotateMaxSpeed);
    }

    public double xDriveExecute(double setpoint)
    {
        Pose2d pose = getPose();

        if (Vision.getInstance(Camera.Front).hasTarget())
        {
            pose = Vision.getInstance(Camera.Front).getEstimatedPose();
        }

        return MathUtil.clamp(_xDrivePID.calculate(pose.getX(), setpoint), -_xDriveMaxSpeed, _xDriveMaxSpeed);
    }

    public double yDriveExecute(double setpoint)
    {
        Pose2d pose = getPose();

        if (Vision.getInstance(Camera.Front).hasTarget())
        {
            pose = Vision.getInstance(Camera.Front).getEstimatedPose();
        }

        return MathUtil.clamp(_yDrivePID.calculate(pose.getY(), setpoint), -_yDriveMaxSpeed, _yDriveMaxSpeed);
    }

    public boolean xDriveIsFinished()
    {
        return _xDrivePID.atSetpoint();
    }

    public boolean yDriveIsFinished()
    {
        return _yDrivePID.atSetpoint();
    }

    public boolean rotateIsFinished()
    {
        return _rotatePID.atSetpoint();
    }

    @AutoLogOutput
    public boolean isAligned()
    {
        return xDriveIsFinished() && yDriveIsFinished() && rotateIsFinished();
    }

    public void setSpeedMultiplier(double speedMultiplier)
    {
        _speedMultiplier = speedMultiplier;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return Commands.sequence(runOnce(() -> setPose(new Pose2d())), run(() -> runCharacterizationVolts(0.0)).withTimeout(1.0), _sysId.quasistatic(direction)).finallyDo(() -> stop());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return Commands.sequence(runOnce(() -> setPose(new Pose2d())), run(() -> runCharacterizationVolts(0.0)).withTimeout(1.0), _sysId.dynamic(direction)).finallyDo(() -> stop());
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs)
    {
        _poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }
}
