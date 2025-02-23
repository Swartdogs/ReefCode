package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Utilities;

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
    private final GyroIOInputsAutoLogged   _gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[]                 _modules    = new Module[4]; // FL, FR, BL, BR
    private final SwerveDrivePoseEstimator _poseEstimator;
    private final SwerveDriveKinematics    _kinematics = new SwerveDriveKinematics(Constants.Drive.MODULE_TRANSLATIONS);
    private PIDController                  _rotatePID;
    private double                         _maxSpeed;
    private double                         _speedMultiplier;

    private Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO)
    {
        _gyroIO = gyroIO;

        _modules[0] = new Module(flModuleIO, 0);
        _modules[1] = new Module(frModuleIO, 1);
        _modules[2] = new Module(blModuleIO, 2);
        _modules[3] = new Module(brModuleIO, 3);

        _rotatePID = new PIDController(Constants.Drive.ROTATE_KP, 0, Constants.Drive.ROTATE_KD);
        _rotatePID.enableContinuousInput(-Math.PI, Math.PI);

        _speedMultiplier = 1;

        RobotConfig config;

        try
        {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e)
        {
            config = Constants.PathPlanner.ROBOT_CONFIG;
        }

        AutoBuilder.configure(
                this::getPose, this::setPose, this::getChassisSpeeds, (speeds, feedforwards) -> runVelocity(speeds),
                new PPHolonomicDriveController(new PIDConstants(Constants.PathPlanner.DRIVE_KP, Constants.PathPlanner.DRIVE_KD), new PIDConstants(Constants.PathPlanner.TURN_KP, Constants.PathPlanner.TURN_KD)), config,
                Utilities::isRedAlliance, this
        );

        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback((activePath) ->
        {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
        {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, new Rotation2d(), getModulePositions(), new Pose2d());
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
    public double getCharacterizationVelocity()
    {
        double driveVelocityAverage = 0.0;

        for (var module : _modules)
        {
            driveVelocityAverage += module.getCharacterizationVelocity();
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

    public void setModuleAbsoluteEncoderOffset(int moduleIndex, Rotation2d offset)
    {
        _modules[moduleIndex].setAbsoluteEncoderOffset(offset);
    }

    public void rotateInit(Rotation2d setpoint, double maxSpeed)
    {
        _maxSpeed = Math.abs(maxSpeed);

        _rotatePID.setSetpoint(setpoint.getRadians());
    }

    public double rotateExecute()
    {
        return MathUtil.clamp(_rotatePID.calculate(getRotation().getRadians()), -_maxSpeed, _maxSpeed);
    }

    public double rotateExecute(Rotation2d setpoint)
    {
        return MathUtil.clamp(_rotatePID.calculate(getRotation().getRadians(), setpoint.getRadians()), -_maxSpeed, _maxSpeed);
    }

    public boolean rotateIsFinished()
    {
        return _rotatePID.atSetpoint();
    }

    public void setSpeedMultiplier(double speedMultiplier)
    {
        _speedMultiplier = speedMultiplier;
    }
}
