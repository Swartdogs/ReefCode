package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public interface DashboardIO
{
    /**
     * Dashboard Settings
     *  Elevator
     *      Min Height
     *      Max Height
     *      Stow Height
     *      L1 Height
     *      L2 Height
     *      L3 Height
     *      L4 Height
     *      KP
     *      KD
     *      Max Down Percent
     *      Max Up Percent
     *  Manipulator
     *      Intake Percent
     *      Output Percent
     *      L1 Speed Multiplier
     *  Funnel
     *      Retract Percent
     *      Retract Time
     *  Drive
     *      FL Offset
     *      FR Offset
     *      BL Offset
     *      BR Offset
     * 
     * Robot Values
     *  Elevator
     *      Current Height
     *      Current Setpoint?
     *  Manipulator
     *      Left Output Percent
     *      Right Output Percent
     *  Funnel
     *      Is Dropped
     *  Drive
     *      Swerve Drive Widget (https://www.chiefdelphi.com/t/how-to-display-swerve-module-states-in-dashboard/474019/2)
     *          FL Angle
     *          FL Velocity
     *          FR Angle
     *          FR Velocity
     *          BL Angle
     *          BL Velocity
     *          BR Angle
     *          BR Velocity
     *          Gyro Heading
     * 
     * Buttons
     *  Elevator
     *      Zero Min Height
     *      Zero Max Height
     *      Zero Stow Height
     *      Zero L1 Height
     *      Zero L2 Height
     *      Zero L3 Height
     *      Zero L4 Height
     *  Drive
     *      Zero Modules
     * 
     * Field stuff
     *  Auto Starting Position
     *  Auto Coral 1
     *  Auto Coral 2
     *  Auto Coral 3
     *  Display Path
     * 
     */

    @AutoLog
    public static class DashboardIOInputs
    {
        // Elevator
        public double elevatorMinHeight = Constants.Elevator.MIN_EXTENSION;
        public double elevatorMaxHeight = Constants.Elevator.MAX_EXTENSION;
        public double elevatorStowHeight = Constants.Elevator.STOW_HEIGHT;
        public double elevatorL1Height = Constants.Elevator.L1_HEIGHT;
        public double elevatorL2Height = Constants.Elevator.L2_HEIGHT;
        public double elevatorL3Height = Constants.Elevator.L3_HEIGHT;
        public double elevatorL4Height = Constants.Elevator.L4_HEIGHT;
        public double elevatorKP = Constants.Elevator.EXTENSION_KP;
        public double elevatorKD = Constants.Elevator.EXTENSION_KD;
        public double elevatorMaxUpwardPercentSpeed = Constants.Elevator.MAX_UPWARDS_SPEED;
        public double elevatorMaxDownwardPercentSpeed = Constants.Elevator.MAX_DOWNWARDS_SPEED;

        // Manipulator
        public double manipulatorIntakePercentSpeed = Constants.Manipulator.INTAKE_SPEED;
        public double manipulatorOutputPercentSpeed = Constants.Manipulator.OUTPUT_SPEED;
        public double manipulatorL1SpeedMultiplier = Constants.Manipulator.L1_SPEED_MULTIPLIER;

        // Funnel
        public double funnelRetractPercentSpeed = Constants.Funnel.RETRACT_SPEED;
        public double funnelRetractTime = Constants.Funnel.DROP_TIME_SECS;

        // Drive
        public Rotation2d driveFLOffset = Constants.Drive.FL_ZERO_ROTATION;
        public Rotation2d driveFROffset = Constants.Drive.FR_ZERO_ROTATION;
        public Rotation2d driveBLOffset = Constants.Drive.BL_ZERO_ROTATION;
        public Rotation2d driveBROffset = Constants.Drive.BR_ZERO_ROTATION;

        // Buttons
        public boolean elevatorZeroMinHeightPressed = false;
        public boolean elevatorZeroMaxHeightPressed = false;
        public boolean elevatorZeroStowHeightPressed = false;
        public boolean elevatorZeroL1HeightPressed = false;
        public boolean elevatorZeroL2HeightPressed = false;
        public boolean elevatorZeroL3HeightPressed = false;
        public boolean elevatorZeroL4HeightPressed = false;
        public boolean driveZeroModulesPressed = false;
    }

    public default void updateInputs(DashboardIOInputs inputs)
    {

    }

    public default void setElevatorHeight(double height) {}

    public default void setElevatorSetpoint(Double setpoint) {}

    public default void setManipulatorLeftMotorOutputPercentSpeed(double speed) {}

    public default void setManipulatorRightMotorOutputPercentSpeed(double speed) {}

    public default void setFunnelIsDropped(boolean dropped) {}

    public default void setDriveFLAngle(Rotation2d angle) {}

    public default void setDriveFLVelocity(Rotation2d velocity) {}

    public default void setDriveFRAngle(Rotation2d angle) {}

    public default void setDriveFRVelocity(Rotation2d velocity) {}

    public default void setDriveBLAngle(Rotation2d angle) {}

    public default void setDriveBLVelocity(Rotation2d velocity) {}

    public default void setDriveBRAngle(Rotation2d angle) {}

    public default void setDriveBRVelocity(Rotation2d velocity) {}

    public default void setDriveHeading(Rotation2d heading) {}
}
