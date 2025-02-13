package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.util.LocalADStarAK;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.General.*;

public class Drive extends SubsystemBase
{
    static final Lock                    ODOMETRY_LOCK          = new ReentrantLock();
    private final GyroIO                 _gyroIO;
    private final GyroIOInputsAutoLogged _gyroInputs            = new GyroIOInputsAutoLogged();
    private final Module[]               _modules               = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine           _sysId;
    private final Alert                  _gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as a fallback.", AlertType.kError);
    private SwerveDriveKinematics        _kinematics            = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
    private Rotation2d                   _rawGyroRotation       = new Rotation2d();
    private SwerveModulePosition[]       _lastModulePositions   = new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    private SwerveDrivePoseEstimator     _poseEstimator         = new SwerveDrivePoseEstimator(_kinematics, _rawGyroRotation, _lastModulePositions, new Pose2d());

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO)
    {
        _gyroIO     = gyroIO;
        _modules[0] = new Module(flModuleIO, 0);
        _modules[1] = new Module(frModuleIO, 1);
        _modules[2] = new Module(blModuleIO, 2);
        _modules[3] = new Module(brModuleIO, 3);

        RobotConfig config;

        try
        {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e)
        {
            e.printStackTrace();
            config = new RobotConfig(ROBOT_MASS, ROBOT_MOI, Constants.Drive.MODULE_CONFIG, MODULE_TRANSLATIONS);
        }

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        OdometryThread.getInstance().start();

        AutoBuilder.configure(
                this::getPose, this::setPose, this::getChassisSpeeds, (speeds, feedforwards) -> runVelocity(speeds),
                new PPHolonomicDriveController(new PIDConstants(Constants.Drive.DRIVE_KP, Constants.Drive.DRIVE_KD), new PIDConstants(Constants.Drive.TURN_KP, Constants.Drive.TURN_KD)), config,

                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, this
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

        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic rotation.
        @SuppressWarnings("removal")
        List<Waypoint> bezierPoints = PathPlannerPath.bezierFromPoses(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)), new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)), new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)));

        _sysId = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())), new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic()
    {
        ODOMETRY_LOCK.lock();

        _gyroIO.updateInputs(_gyroInputs);
        Logger.processInputs("Drive/Gyro", _gyroInputs);

        for (var module : _modules)
        {
            module.periodic();
        }

        ODOMETRY_LOCK.unlock();

        if (DriverStation.isDisabled())
        {
            for (var module : _modules)
            {
                module.stop();
            }
        }

        if (DriverStation.isDisabled())
        {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = _modules[0].getOdometryTimestamps();
        int      sampleCount      = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++)
        {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas    = new SwerveModulePosition[4];

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
            {
                modulePositions[moduleIndex]      = _modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex]         = new SwerveModulePosition(modulePositions[moduleIndex].distanceMeters - _lastModulePositions[moduleIndex].distanceMeters, modulePositions[moduleIndex].angle);
                _lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            if (_gyroInputs.connected)
            {
                _rawGyroRotation = _gyroInputs.odometryYawPositions[i];
            }
            else
            {
                Twist2d twist = _kinematics.toTwist2d(moduleDeltas);
                _rawGyroRotation = _rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            _poseEstimator.updateWithTime(sampleTimestamps[i], _rawGyroRotation, modulePositions);
        }

        _gyroDisconnectedAlert.set(!_gyroInputs.connected && Constants.AdvantageKit.CURRENT_MODE != Constants.AdvantageKit.Mode.SIM);
    }

    public void runVelocity(ChassisSpeeds speeds)
    {
        ChassisSpeeds       discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.General.LOOP_PERIOD_SECS);
        SwerveModuleState[] setpointStates = _kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for (int i = 0; i < 4; i++)
        {
            _modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runCharacterization(double output)
    {
        for (int i = 0; i < 4; i++)
        {
            _modules[i].runCharacterization(output);
        }
    }

    public void setTurnOpenLoop(double volts)
    {
        for (int i = 0; i < 4; i++)
        {
            _modules[i].setTurnOpenLoop(volts);
        }
    }

    public void setTurnPosition(Rotation2d angle)
    {
        for (int i = 0; i < 4; i++)
        {
            _modules[i].setTurnPosition(angle);
        }
    }

    public void stop()
    {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX()
    {
        Rotation2d[] headings = new Rotation2d[4];

        for (int i = 0; i < 4; i++)
        {
            headings[i] = MODULE_TRANSLATIONS[i].getAngle();
        }

        _kinematics.resetHeadings(headings);

        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(_sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(_sysId.dynamic(direction));
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++)
        {
            states[i] = _modules[i].getState();
        }

        return states;
    }

    private SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++)
        {
            positions[i] = _modules[i].getPosition();
        }

        return positions;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds()
    {
        return _kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions()
    {
        double[] values = new double[4];

        for (int i = 0; i < 4; i++)
        {
            values[i] = _modules[i].getWheelRadiusCharacterizationPosition();
        }

        return values;
    }

    public double getFFCharacterizationVelocity()
    {
        double output = 0.0;

        for (int i = 0; i < 4; i++)
        {
            output += _modules[i].getFFCharacterizationVelocity() / 4.0;
        }

        return output;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose()
    {
        return _poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation()
    {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose)
    {
        _poseEstimator.resetPosition(_rawGyroRotation, getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs)
    {
        _poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public double getMaxLinearSpeedMetersPerSec()
    {
        return MAX_LINEAR_SPEED;
    }

    public double getMaxAngularSpeedRadPerSec()
    {
        return MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    }

    public void setTurnVolts(double volts)
    {
        for (int i = 0; i < 4; i++)
        {
            _modules[i].setTurnVolts(volts);
        }
    }
}
