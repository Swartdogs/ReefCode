package frc.robot;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;

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
    private final Alert          _batteryLowVoltage;
    private final Alert          _rioBrownout;
    private final MedianFilter   _voltageFilter;
    private Command              _autonomousCommand;

    public Robot()
    {
        _batteryLowVoltage = new Alert("Battery Has Dropped Below 11.5 Volts", AlertType.kWarning);
        _rioBrownout       = new Alert("RoboRIO brownout detected", AlertType.kError);

        _voltageFilter = new MedianFilter((int)(Constants.Dashboard.LOW_BATTERY_TIME_THRESHOLD / Constants.General.LOOP_PERIOD_SECS));

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

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic()
    {
        Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();

        Threads.setCurrentThreadPriority(false, 10);

        _batteryLowVoltage.set(_voltageFilter.calculate(RobotController.getBatteryVoltage()) < Constants.Dashboard.LOW_BATTERY_VOLTAGE);
        _rioBrownout.set(RobotController.isBrownedOut());
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
        // if (DriverStation.isFMSAttached())
        // {
        //     Elastic.selectTab("Teleoperated");
        // }

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
