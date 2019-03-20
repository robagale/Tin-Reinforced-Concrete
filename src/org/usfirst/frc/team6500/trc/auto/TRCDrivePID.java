/*
 *  TRCDrivePID
 *      static class to control the Robot Drive
 */

package org.usfirst.frc.team6500.trc.auto;

import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCEncoderSet;
import org.usfirst.frc.team6500.trc.wrappers.sensors.TRCGyroBase;
import org.usfirst.frc.team6500.trc.wrappers.systems.drives.TRCDifferentialDrive;
import org.usfirst.frc.team6500.trc.wrappers.systems.drives.TRCMecanumDrive;

import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.util.TRCSpeed;
import org.usfirst.frc.team6500.trc.util.TRCTypes.Direction;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DirectionType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DriveType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.RobotSide;
import org.usfirst.frc.team6500.trc.util.TRCTypes.UnitType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.VerbosityType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class TRCDrivePID
{
    protected static final double deadband = 2.0;
    protected static final int verificationMin = 25;

    private static TRCEncoderSet encoders;
    private static TRCGyroBase gyro;
    private static Object drive;

    private static boolean driving;
    private static boolean autoAuthorized = false;
    protected static double maxSpeed = 0.0;
    private static DriveType driveType;

    /**
     * Set up the necessary elements to be able to drive the robot in autonomous
     * with a PID control loop for accurate distances and degrees
     * 
     * @param encoderset    The robot's drivetrain encoders
     * @param gyroBase      The robot's primary gyro
     * @param driveBase     The robot's drivetrain
     * @param driveBaseType The type of the robot's drivetrain {@link DriveType}
     * @param topSpeed      Fastest speed the robot should go in autonomous
     */
    public static void initializeTRCDrivePID(TRCEncoderSet encoderset, TRCGyroBase gyroBase, Object driveBase, DriveType driveBaseType, double topSpeed)
    {
        encoders = encoderset;
        gyro = gyroBase;
        drive = driveBase;
        driveType = driveBaseType;
        maxSpeed = topSpeed;

        TRCNetworkData.logString(VerbosityType.Log_Info, "DrivePID is online.");

        TRCNetworkData.createDataPoint("PIDSetpoint");
        TRCNetworkData.createDataPoint("PIDOutput");
        TRCNetworkData.createDataPoint("PIDOutputSmoothed");
    }

    /**
     * Gives the auto driving functions permission to move the robot in a state
     * other than Autonomous
     */
    public static void grantSubautonomousAction()
    {
        autoAuthorized = true;
    }

    /**
     * Disables the auto driving functions from moving the robot in any other state
     * than Autonomous (Recomended to call this in every disabled())
     */
    public static void denySubautonomousAction()
    {
        autoAuthorized = false;
    }

    /**
     * Executes an action based on the specified vector
     * 
     * @param action The type of action (a {@link TRCVector}) the robot should take
     */
    public static void drive(TRCVector action)
    {
        driving = false;
        if (!action.getIsValid())
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: drive(): the specified action is not valid");
            return;
        }

        encoders.resetAllEncoders();
        gyro.reset();

        switch (action.getType())
        {
            case ForwardBack: driveForwardBack(action); break;
            case   LeftRight: driveLeftRight(action); break;
            case    Rotation: driveRotation(action); break;
            case    Diagonal: driveDiagonal(action); break;
            case    OnCorner: driveOnCorner(action); break;
            case      OnAxis: driveOnAxis(action); break;
        }
    }

    /**
     * Drive the robot forward [or backward] an amount of inches. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param action the action (a {@link TRCVector}) in which to actually perform
     */
    protected static void driveForwardBack(TRCVector action)
    {
        if (action.getUnitType() == UnitType.Degrees) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveForwardBack(): given value in Degrees");
            return;
        }

        if (!RobotState.isAutonomous() && !autoAuthorized) // check for clearance
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveForwardBack(): tried to drive subautonomously without permission");
            return;
        }

        MiniPID PID = new MiniPID(-1.0, 0.0, 0.0);
        PID.setOutputLimits(-maxSpeed, maxSpeed);
        PID.setSetpoint(action.translateValue(UnitType.Inches));
        TRCSpeed autoSpeed = new TRCSpeed();

        if (!driving) return;

        driving = true;
        int deadbandcounter = 0;

        while (deadbandcounter < verificationMin)
        {
            double newSpeed = PID.getOutput(encoders.getAverageDistanceTraveled(DirectionType.ForwardBackward));
            double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 0.85);

            String[] dpNames = {"PIDOutput", "PIDOutputSmoothed"};
            Object[] dpValues = {newSpeed, smoothedSpeed};
            updateDriveDataPoints(dpNames, dpValues);

            if (driveType == DriveType.Mecanum)
            {
                ((TRCMecanumDrive) drive).driveCartesian(-smoothedSpeed, 0.0, 0.0);
            }
            else if (driveType == DriveType.Differential)
            {
                ((TRCDifferentialDrive) drive).arcadeDrive(smoothedSpeed, 0.0, false);
            }
            
            double difference = Math.abs(encoders.getAverageDistanceTraveled(DirectionType.ForwardBackward) - action.translateValue(UnitType.Inches));
            if (difference < deadband) deadbandcounter++; // if in the deadband, increment the counter
        }
        driving = false;
    }

    /**
     * Drive the robot side to side in inches (exclusive to mecanum). See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param action the action (a {@link TRCVector}) in which to actually perform
     */
    protected static void driveLeftRight(TRCVector action)
    {
        if (action.getUnitType() == UnitType.Degrees) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveLeftRight(): given a value in Degrees");
            return;
        }

        if (driveType != DriveType.Mecanum)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveLeftRight(): tried to strafe without Mecanum drive");
            return;
        }

        if (!RobotState.isAutonomous() && !autoAuthorized) // check for clearance
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveLeftRight(): tried to drive subautonomously without permission");
            return;
        }

        MiniPID PID = new MiniPID(-1.0, 0.0, 0.0);
        PID.setOutputLimits(-maxSpeed, maxSpeed);
        PID.setSetpoint(action.translateValue(UnitType.Inches));
        TRCSpeed autoSpeed = new TRCSpeed();

        if (!driving) return;

        driving = true;
        int deadbandcounter = 0;

        while (deadbandcounter < verificationMin)
        {
            double newSpeed = PID.getOutput(encoders.getAverageDistanceTraveled(DirectionType.LeftRight));
            double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

            String[] dpNames = {"PIDOutput", "PIDOutputSmoothed"};
            Object[] dpValues = {newSpeed, smoothedSpeed};
            updateDriveDataPoints(dpNames, dpValues);

            ((TRCMecanumDrive) drive).driveCartesian(0.0, smoothedSpeed, 0.0);

            double difference = Math.abs(encoders.getAverageDistanceTraveled(DirectionType.LeftRight) - action.translateValue(UnitType.Inches));
            if (difference < deadband) deadbandcounter++; // if in the deadband, increment the counter
        }
        driving = false;
    }

    /**
     * Rotate the robot on the center a certain degrees. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param action the action (a {@link TRCVector}) in which to actually perform
     */
    protected static void driveRotation(TRCVector action)
    {
        if (action.getUnitType() == UnitType.Inches) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveRotation(): given a value in Inches");
            return;
        }

        if (!RobotState.isAutonomous() && !autoAuthorized) // check for clearance
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveRotation(): tried to drive subautonomously without permission");
            return;
        }

        MiniPID PID = new MiniPID(-1.0, 0.0, 0.0);
        PID.setOutputLimits(-maxSpeed, maxSpeed);
        PID.setSetpoint(action.translateValue(UnitType.Degrees));
        TRCSpeed autoSpeed = new TRCSpeed();

        if (!driving) return;

        driving = true;
        int deadbandcounter = 0;

        while (deadbandcounter < verificationMin)
        {
            double newSpeed = PID.getOutput(gyro.getAngle());
            double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

            String[] dpNames = {"PIDOutput", "PIDOutputSmoothed"};
            Object[] dpValues = {newSpeed, smoothedSpeed};
            updateDriveDataPoints(dpNames, dpValues);

            if (driveType == DriveType.Mecanum)
            {
                ((TRCMecanumDrive) drive).driveCartesian(0.0, 0.0, smoothedSpeed);
            }
            else if (driveType == DriveType.Differential)
            {
                ((TRCDifferentialDrive) drive).arcadeDrive(0.0, smoothedSpeed, false);
            }

            double difference = Math.abs(gyro.getAngle() - action.translateValue(UnitType.Degrees));
            if (difference < deadband) deadbandcounter++; // if in the deadband, increment the counter
        }
        driving = false;
    }

    /**
     * Moves the robot in a diagonal bound (Up-Left, Up-Right, Down-Left, or
     * Down-Right) direction (exclusive to mecanum). See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param action the action (a {@link TRCVector}) in which to actually perform
     */
    protected static void driveDiagonal(TRCVector action)
    {
        if (action.getUnitType() == UnitType.Degrees) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveDiagonal(): giving a value in Degrees");
            return;
        }

        if (driveType != DriveType.Mecanum)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveDiagonal(): tried to drive diagonally without Mecanum drive");
            return;
        }

        if (!RobotState.isAutonomous() && !autoAuthorized) // check for clearance
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveDiagonal(): tried to drive subautonomously without permission");
            return;
        }

        MiniPID PID = new MiniPID(-1.0, 0.0, 0.0);
        PID.setOutputLimits(-maxSpeed, maxSpeed);
        TRCSpeed autoSpeed = new TRCSpeed();

        // override setpoint setting looking for inversion
        double inversion = 1.0;
        if (action.getDirection() == Direction.BackwardLeft || action.getDirection() == Direction.BackwardRight || action.translateValue(UnitType.General) < 0.0) 
        {
            inversion = -1.0; // invert
        }
        PID.setSetpoint(action.translateValue(UnitType.Inches)*inversion);

        if (driving) return;

        driving = true;
        int deadbandcounter = 0;

        while (deadbandcounter < verificationMin)
        {
            double forwardBackDist = encoders.getAverageDistanceTraveled(DirectionType.ForwardBackward);
            double leftRightDist = encoders.getAverageDistanceTraveled(DirectionType.LeftRight);
            double averageDistanceTraveled = (forwardBackDist+leftRightDist)/2;

            double newSpeed = PID.getOutput(averageDistanceTraveled);
            double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

            String[] dpNames = {"PIDOutput", "PIDOutputSmoothed"};
            Object[] dpValues = {newSpeed, smoothedSpeed};
            updateDriveDataPoints(dpNames, dpValues);

            TRCMecanumDrive mDrive = (TRCMecanumDrive)drive;
            if (action.getDirection() == Direction.ForwardLeft || action.getDirection() == Direction.BackwardRight)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, 0.0);               // drive front left at none
                mDrive.driveWheel(MotorType.kFrontRight, smoothedSpeed);    // drive front right at speed
                mDrive.driveWheel(MotorType.kRearLeft, smoothedSpeed);      // drive rear left at speed
                mDrive.driveWheel(MotorType.kRearRight, 0.0);               // drive rear right at none
            }
            else if (action.getDirection() == Direction.ForwardRight || action.getDirection() == Direction.BackwardLeft)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, smoothedSpeed); // drive front left at speed
                mDrive.driveWheel(MotorType.kFrontRight, 0.0);          // drive front right at none
                mDrive.driveWheel(MotorType.kRearLeft, 0.0);            // drive rear left at none
                mDrive.driveWheel(MotorType.kRearRight, smoothedSpeed); // drive rear right at speed
            }

            double difference = Math.abs(averageDistanceTraveled - action.translateValue(UnitType.Inches));
            if (difference < deadband) deadbandcounter++; // if in the deadband, increment the counter
        }
        driving = false;
    }

    /**
     * Moves the robot on a corner of the robot (exclusive to mecanum). See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param action the action (a {@link TRCVector}) in which to actually perform
     */
    protected static void driveOnCorner(TRCVector action)
    {
        if (action.getUnitType() == UnitType.Inches) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveOnCorner(): given a value in Inches");
            return;
        }

        if (driveType != DriveType.Mecanum)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveOnCorner(): tried to drive on a corner without Mecanum drive");
            return;
        }

        if (!RobotState.isAutonomous() && !autoAuthorized) // check for clearance
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveOnCorner(): tried to drive subautonomously without permission");
            return;
        }

        MiniPID PID = new MiniPID(-1.0, 0.0, 0.0);
        PID.setOutputLimits(-maxSpeed, maxSpeed);
        PID.setSetpoint(action.translateValue(UnitType.Degrees));
        TRCSpeed autoSpeed = new TRCSpeed();

        if (driving) return;

        driving = true;
        int deadbandcounter = 0;

        while (deadbandcounter < verificationMin)
        {
            double newSpeed = PID.getOutput(gyro.getAngle());
            double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

            String[] dpNames = {"PIDOutput", "PIDOutputSmoothed"};
            Object[] dpValues = {newSpeed, smoothedSpeed};
            updateDriveDataPoints(dpNames, dpValues);

            TRCMecanumDrive mDrive = (TRCMecanumDrive)drive;
            if (action.getTurnWheel() == MotorType.kFrontLeft)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, 0.0);               // drive front left at none
                mDrive.driveWheel(MotorType.kFrontRight, -smoothedSpeed);   // drive front right at negative speed
                mDrive.driveWheel(MotorType.kRearLeft, -smoothedSpeed);      // drive rear left at negative speed
                mDrive.driveWheel(MotorType.kRearRight, -smoothedSpeed);    // drive rear right at negative speed
            }
            else if (action.getTurnWheel() == MotorType.kFrontRight)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, -smoothedSpeed);    // drive front left at negative speed
                mDrive.driveWheel(MotorType.kFrontRight, 0.0);              // drive front right at none
                mDrive.driveWheel(MotorType.kRearLeft, -smoothedSpeed);     // drive rear left at negative speed
                mDrive.driveWheel(MotorType.kRearRight, -smoothedSpeed);     // drive rear right at negative speed
            }
            else if (action.getTurnWheel() == MotorType.kRearLeft)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, smoothedSpeed);    // drive front left at speed
                mDrive.driveWheel(MotorType.kFrontRight, smoothedSpeed);    // drive front right at speed
                mDrive.driveWheel(MotorType.kRearLeft, 0.0);                // drive rear left at none
                mDrive.driveWheel(MotorType.kRearRight, smoothedSpeed);     // drive rear right at speed
            }
            else if (action.getTurnWheel() == MotorType.kRearRight)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, smoothedSpeed);     // drive front left at speed
                mDrive.driveWheel(MotorType.kFrontRight, smoothedSpeed);   // drive front right at speed
                mDrive.driveWheel(MotorType.kRearLeft, smoothedSpeed);      // drive rear left at speed
                mDrive.driveWheel(MotorType.kRearRight, 0.0);               // drive rear right at none
            }

            double difference = Math.abs(gyro.getAngle() - action.translateValue(UnitType.Degrees));
            if (difference < deadband) deadbandcounter++; // if in the deadband, increment the counter
        }
        driving = false;
    }

    /**
     * Turns the robot on an axis (side) of the robot, only works with front and
     * rear sides (exclusive to mecanum). See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param action the action (a {@link TRCVector}) in which to actually perform
     */
    protected static void driveOnAxis(TRCVector action)
    {
        if (action.getUnitType() == UnitType.Inches) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveOnAxis(): given a value in Inches");
            return;
        }

        if (driveType != DriveType.Mecanum)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveOnAxis(): tried to drive on an axis without Mecanum drive");
            return;
        }

        if (!RobotState.isAutonomous() && !autoAuthorized) // check for clearance
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveOnAxis(): tried to drive subautonomously without permission");
            return;
        }

        MiniPID PID = new MiniPID(-1.0, 0.0, 0.0);
        PID.setOutputLimits(-maxSpeed, maxSpeed);
        PID.setSetpoint(action.translateValue(UnitType.Degrees));
        TRCSpeed autoSpeed = new TRCSpeed();

        if (driving) return;

        driving = true;
        int deadbandcounter = 0;

        while (deadbandcounter < verificationMin)
        {
            double newSpeed = PID.getOutput(gyro.getAngle());
            double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

            String[] dpNames = {"PIDOutput", "PIDOutputSmoothed"};
            Object[] dpValues = {newSpeed, smoothedSpeed};
            updateDriveDataPoints(dpNames, dpValues);

            TRCMecanumDrive mDrive = (TRCMecanumDrive)drive;
            if (action.getTurnSide() == RobotSide.kFront)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, 0.0);              // drive front left at none
                mDrive.driveWheel(MotorType.kFrontRight, 0.0);             // drive front right at none
                mDrive.driveWheel(MotorType.kRearLeft, -smoothedSpeed);    // drive rear left at negative speed
                mDrive.driveWheel(MotorType.kRearRight, smoothedSpeed);    // drive rear right at speed
            }
            else if (action.getTurnSide() == RobotSide.kRear)
            {
                mDrive.driveWheel(MotorType.kFrontLeft, smoothedSpeed);    // drive front left at speed
                mDrive.driveWheel(MotorType.kFrontRight, -smoothedSpeed);  // drive front right at negative speed
                mDrive.driveWheel(MotorType.kRearLeft, 0.0);               // drive rear left at none
                mDrive.driveWheel(MotorType.kRearRight, 0.0);              // drive rear right at none
            }

            double difference = Math.abs(gyro.getAngle() - action.translateValue(UnitType.Degrees));
            if (difference < deadband) deadbandcounter++; // if in the deadband, increment the counter
        }
        driving = false;
    }

    /**
     * Update driving datapoints
     * 
     * @param names  An array of the names of the Data Points to update
     * @param values An array of the values of the Data Points to update
     */
    private static void updateDriveDataPoints(String[] names, Object[] values)
    {
        TRCNetworkData.updateDataPoint("PIDSetpoint", 0.0);
        TRCNetworkData.updateDataPoint("PIDOuput", 0.0);
        TRCNetworkData.updateDataPoint("PIDOutputSmoothed", 0.0);

        if (names.length != values.length) TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: updateDriveDataPoints(): names and values have different sizes, points not updated");

        for (int index = 0; index < names.length; index++)
        {
            String name = names[index];
            Object value = values[index];
            TRCNetworkData.updateDataPoint(name, value);
        }
    }
}
