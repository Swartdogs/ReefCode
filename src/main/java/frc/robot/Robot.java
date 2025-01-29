package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.dashboard.Dashboard;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
    private final RobotContainer _robotContainer;
    private Command              _autonomousCommand;

    private final Timer _canErrorTimer;
    private final Timer _canErrorTimerInitial;
    private final Alert _canError;
    private final Alert _funnelDrop;
    private final Alert _sensorDisconnect;
    private final Alert _batteryDrop;
    private final Alert _motorBrownout;

    public Robot()
    {
        _canErrorTimer = new Timer();
        _canErrorTimerInitial = new Timer();
        _canError = new Alert("CAN Error Detected", AlertType.kError);
        _funnelDrop = new Alert("The Funnel Has Been Dropped", AlertType.kInfo);
        _sensorDisconnect = new Alert("Sensor Has Been Disconnected", AlertType.kError);
        _batteryDrop = new Alert("Battery Has Dropped Below 11.5 Volts", AlertType.kWarning);
        _motorBrownout = new Alert("Motor Brownout Detected", AlertType.kError);

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY)
        {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Initialize URCL
        Logger.registerURCL(URCL.startExternal());

        // Start AdvantageKit logger
        Logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        _robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit()
    {
        _canErrorTimer.reset();
        _canErrorTimer.start();
        _canErrorTimerInitial.reset();
        _canErrorTimerInitial.start();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic()
    {
       
        Threads.setCurrentThreadPriority(true, 99);

        
        CommandScheduler.getInstance().run();

    
        Threads.setCurrentThreadPriority(false, 10);

        var canStatus = RobotController.getCANStatus();

        if(canStatus.receiveErrorCount>0 || canStatus.transmitErrorCount>0)
        {
            _canErrorTimer.reset();
        }

        _canError.set(!_canErrorTimer.hasElapsed(Constants.Dashboard.CAN_ERROR_TIME_THRESHOLD)&& _canErrorTimerInitial.hasElapsed(Constants.Dashboard.CAN_ERROR_TIME_THRESHOLD)); //less than 2 seconds since the error has been detected but more than 2 secons since robot was started

        if (RobotController.getBatteryVoltage() < Constants.Dashboard.LOW_BATTERY_VOLTAGE)
        {
            _batteryDrop.set(true);
        }

        _funnelDrop.set(_robotContainer.getFunnelIsDropped());
       
        if(RobotController.isBrownedOut())
        {
            _motorBrownout.set(true);
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit()
    {
        _autonomousCommand = _robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (_autonomousCommand != null)
        {
            _autonomousCommand.schedule();
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (_autonomousCommand != null)
        {
            _autonomousCommand.cancel();
        }
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
