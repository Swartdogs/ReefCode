package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ManipulatorCommands;
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
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;

import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive       _drive;
    private final Elevator    _elevator;
    private final Manipulator _manipulator;

    // Controller
    private final CommandXboxController _controller = new CommandXboxController(0);
    private final ArrayList<JoystickButton> _driverButtons = new ArrayList<JoystickButton>();

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> _autoChooser;

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
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _elevator = new Elevator(new ElevatorIOSim());
                _manipulator = new Manipulator(new ManipulatorIOSim(() -> _controller.leftTrigger().getAsBoolean()));
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _elevator = new Elevator(new ElevatorIO() {});
                _manipulator = new Manipulator(new ManipulatorIO() {});
                break;
        }

        Joystick _driverButtonBox = new Joystick(1);

        for(int i = 0; i < 9; i++)
        {
            _driverButtons.add(new JoystickButton(_driverButtonBox, i));
        }

        // Set up auto routines
        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        _autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(_drive));
        _autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(_drive));
        _autoChooser.addOption("Drive SysId (Quasistatic Forward)", _drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        _autoChooser.addOption("Drive SysId (Quasistatic Reverse)", _drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        _autoChooser.addOption("Drive SysId (Dynamic Forward)", _drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        _autoChooser.addOption("Drive SysId (Dynamic Reverse)", _drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        // Default command, normal field-relative drive
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX()));

        _driverButtons.get(0).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_REEF_ANGLE_ONE : Constants.Field.BLUE_REEF_ANGLE_ONE));
        _driverButtons.get(1).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_REEF_ANGLE_TWO : Constants.Field.BLUE_REEF_ANGLE_TWO));
        _driverButtons.get(2).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_REEF_ANGLE_THREE : Constants.Field.BLUE_REEF_ANGLE_THREE));
        _driverButtons.get(3).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_REEF_ANGLE_FOUR : Constants.Field.BLUE_REEF_ANGLE_FOUR));
        _driverButtons.get(4).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_REEF_ANGLE_FIVE : Constants.Field.BLUE_REEF_ANGLE_FIVE));
        _driverButtons.get(5).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_REEF_ANGLE_SIX : Constants.Field.BLUE_REEF_ANGLE_SIX));
        _driverButtons.get(6).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_LEFT_STATION_ANGLE : Constants.Field.BLUE_LEFT_STATION_ANGLE));
        _driverButtons.get(7).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_RIGHT_STATION_ANGLE : Constants.Field.BLUE_RIGHT_STATION_ANGLE));
        _driverButtons.get(8).whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Constants.Field.RED_PROCESSOR_ANGLE : Constants.Field.BLUE_PROCESSOR_ANGLE));
        

        _controller.rightTrigger().whileTrue(ElevatorCommands.setVolts(_elevator, () -> -_controller.getRightY() * Constants.General.MOTOR_VOLTAGE));

        _controller.povUp().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level1));
        _controller.povRight().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level2));
        _controller.povDown().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level3));
        _controller.povLeft().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level4));

        _controller.leftTrigger().onTrue(ManipulatorCommands.intake(_manipulator));
        _controller.start().onTrue(ManipulatorCommands.output(_manipulator));
        _controller.rightStick().onTrue(ManipulatorCommands.stop(_manipulator));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return _autoChooser.get();
    }
}
