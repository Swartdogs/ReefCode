package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public interface DashboardIO
{
    @AutoLog
    public static class DashboardIOInputs
    {
        // Elevator
        public double elevatorMinHeight               = Constants.Elevator.MIN_EXTENSION;
        public double elevatorMaxHeight               = Constants.Elevator.MAX_EXTENSION;
        public double elevatorStowHeight              = Constants.Elevator.STOW_HEIGHT;
        public double elevatorL1Height                = Constants.Elevator.L1_HEIGHT;
        public double elevatorL2Height                = Constants.Elevator.L2_HEIGHT;
        public double elevatorL3Height                = Constants.Elevator.L3_HEIGHT;
        public double elevatorL4Height                = Constants.Elevator.L4_HEIGHT;
        public double elevatorHangHeight              = Constants.Elevator.HANG_HEIGHT;
        public double elevatorKP                      = Constants.Elevator.EXTENSION_KP;
        public double elevatorKD                      = Constants.Elevator.EXTENSION_KD;
        public double elevatorMaxUpwardPercentSpeed   = Constants.Elevator.MAX_UPWARDS_SPEED;
        public double elevatorMaxDownwardPercentSpeed = Constants.Elevator.MAX_DOWNWARDS_SPEED;
        public double elevatorHangSpeed               = Constants.Elevator.HANG_SPEED;

        // Manipulator
        public double manipulatorIntakePercentSpeed = Constants.Manipulator.INTAKE_SPEED;
        public double manipulatorOutputPercentSpeed = Constants.Manipulator.OUTPUT_SPEED;
        public double manipulatorL1SpeedMultiplier  = Constants.Manipulator.L1_SPEED_MULTIPLIER;

        // Funnel
        public double funnelRetractPercentSpeed = Constants.Funnel.RETRACT_SPEED;
        public double funnelRetractTime         = Constants.Funnel.DROP_TIME_SECS;

        // Drive
        public Rotation2d driveFLOffset = Constants.Drive.FL_ZERO_ROTATION;
        public Rotation2d driveFROffset = Constants.Drive.FR_ZERO_ROTATION;
        public Rotation2d driveBLOffset = Constants.Drive.BL_ZERO_ROTATION;
        public Rotation2d driveBROffset = Constants.Drive.BR_ZERO_ROTATION;

        // Buttons
        public boolean elevatorZeroMinHeightPressed  = false;
        public boolean elevatorZeroMaxHeightPressed  = false;
        public boolean elevatorZeroStowHeightPressed = false;
        public boolean elevatorZeroL1HeightPressed   = false;
        public boolean elevatorZeroL2HeightPressed   = false;
        public boolean elevatorZeroL3HeightPressed   = false;
        public boolean elevatorZeroL4HeightPressed   = false;
        public boolean elevatorZeroHangHeightPressed = false;
        public boolean driveZeroFLModulePressed      = false;
        public boolean driveZeroFRModulePressed      = false;
        public boolean driveZeroBLModulePressed      = false;
        public boolean driveZeroBRModulePressed      = false;
        public boolean driveZeroModulesPressed       = false;

        // Auto Selectors
        public int    autoDelay         = 0;
        public String autoStartPosition = "";
        public String autoFirstCoral    = "";
        public String autoSecondCoral   = "";
        public String autoThirdCoral    = "";
        public String autoPath          = "";
    }

    public default void updateInputs(DashboardIOInputs inputs)
    {

    }

    public default void setElevatorHeight(double height)
    {
    }

    public default void setElevatorSetpoint(Double setpoint)
    {
    }

    public default void setManipulatorLeftMotorOutputPercentSpeed(double speed)
    {
    }

    public default void setManipulatorRightMotorOutputPercentSpeed(double speed)
    {
    }

    public default void setFunnelIsDropped(boolean dropped)
    {
    }

    public default void setDriveFLAngle(Rotation2d angle)
    {
    }

    public default void setDriveFLVelocity(double velocity)
    {
    }

    public default void setDriveFRAngle(Rotation2d angle)
    {
    }

    public default void setDriveFRVelocity(double velocity)
    {
    }

    public default void setDriveBLAngle(Rotation2d angle)
    {
    }

    public default void setDriveBLVelocity(double velocity)
    {
    }

    public default void setDriveBRAngle(Rotation2d angle)
    {
    }

    public default void setDriveBRVelocity(double velocity)
    {
    }

    public default void setDriveHeading(Rotation2d heading)
    {
    }

    public default void releaseElevatorMinHeightZeroButton()
    {
    }

    public default void releaseElevatorMaxHeightZeroButton()
    {
    }

    public default void releaseElevatorStowHeightZeroButton()
    {
    }

    public default void releaseElevatorL1HeightZeroButton()
    {
    }

    public default void releaseElevatorL2HeightZeroButton()
    {
    }

    public default void releaseElevatorL3HeightZeroButton()
    {
    }

    public default void releaseElevatorL4HeightZeroButton()
    {
    }

    public default void releaseElevatorHangHeightZeroButton()
    {
    }

    public default void releaseDriveFLOffsetZeroButton()
    {
    }

    public default void releaseDriveFROffsetZeroButton()
    {
    }

    public default void releaseDriveBLOffsetZeroButton()
    {
    }

    public default void releaseDriveBROffsetZeroButton()
    {
    }

    public default void releaseDriveModuleOffsetZeroButton()
    {
    }
}
