package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public Robot()
    {
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
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
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
