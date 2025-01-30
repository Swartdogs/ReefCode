package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.FunnelCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIO;
import frc.robot.subsystems.dashboard.DashboardIONetwork;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOHardware;
import frc.robot.subsystems.funnel.FunnelIOSim;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive       _drive;
    private final Elevator    _elevator;
    private final Manipulator _manipulator;
    private final Funnel      _funnel;
    private final Dashboard   _dashboard;

    // Controller
    private final CommandXboxController _controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<String> _autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                _drive = new Drive(new GyroIONavX(), new ModuleIOHardware(0), new ModuleIOHardware(1), new ModuleIOHardware(2), new ModuleIOHardware(3));
                _elevator = new Elevator(new ElevatorIOHardware());
                _manipulator = new Manipulator(new ManipulatorIOHardware());
                _funnel = new Funnel(new FunnelIOHardware());
                _dashboard = new Dashboard(new DashboardIONetwork());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _elevator = new Elevator(new ElevatorIOSim());
                _manipulator = new Manipulator(new ManipulatorIOSim(() -> _controller.leftTrigger().getAsBoolean()));
                _funnel = new Funnel(new FunnelIOSim());
                _dashboard = new Dashboard(new DashboardIONetwork());
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _elevator = new Elevator(new ElevatorIO() {});
                _manipulator = new Manipulator(new ManipulatorIO() {});
                _funnel = new Funnel(new FunnelIO() {});
                _dashboard = new Dashboard(new DashboardIO() {});
                break;
        }

        // Set up auto routines
        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());

        // Set up SysId routines
        /*
         * _autoChooser.addOption("Drive Wheel Radius Characterization",
         * DriveCommands.wheelRadiusCharacterization(_drive));
         * _autoChooser.addOption("Drive Simple FF Characterization",
         * DriveCommands.feedforwardCharacterization(_drive));
         * _autoChooser.addOption("Drive SysId (Quasistatic Forward)",
         * _drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
         * _autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
         * _drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
         * _autoChooser.addOption("Drive SysId (Dynamic Forward)",
         * _drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
         * _autoChooser.addOption("Drive SysId (Dynamic Reverse)",
         * _drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
         */
        try
        {
            _autoChooser.addDefaultOption("1CoralAuto", "1CoralAuto");
            _autoChooser.addOption("2CoralAuto", "2CoralAuto");
        }
        catch (Exception e)
        {
            // TODO: handle exception
        }

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        // Default command, normal field-relative drive
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX()));

        // Lock to 0° when A button is held
        _controller.a().whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        _controller.x().onTrue(Commands.runOnce(_drive::stopWithX, _drive));

        // Reset gyro to 0° when B button is pressed
        _controller.b().onTrue(Commands.runOnce(() -> _drive.setPose(new Pose2d(_drive.getPose().getTranslation(), new Rotation2d())), _drive).ignoringDisable(true));

        _controller.y().onTrue(FunnelCommands.drop(_funnel));
        _controller.rightTrigger().whileTrue(ElevatorCommands.setVolts(_elevator, () -> -_controller.getRightY() * Constants.General.MOTOR_VOLTAGE));

        _controller.povUp().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level1));
        _controller.povRight().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level2));
        _controller.povDown().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level3));
        _controller.povLeft().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level4));

        _controller.leftTrigger().onTrue(ManipulatorCommands.intake(_manipulator));
        _controller.start().onTrue(ManipulatorCommands.output(_manipulator));
        _controller.rightStick().onTrue(ManipulatorCommands.stop(_manipulator));
    }

    public boolean getFunnelIsDropped()
    {
        return _funnel.isDropped();
    }

    public void periodic()
    {
        var cedricAuto = _dashboard.getPaths(_autoChooser.get());
        _dashboard.setAuto(cedricAuto);
        _dashboard.setRobotPosition(cedricAuto.get(0).getStartingHolonomicPose().get());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}
