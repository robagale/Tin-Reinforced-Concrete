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
import org.usfirst.frc.team6500.trc.util.TRCVector;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DriveActionType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DriveType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.VerbosityType;

import edu.wpi.first.wpilibj.RobotState;

public class TRCDrivePID
{
    private static final double deadband = 2.0;
    private static final int verificationMin = 25;

    private static TRCEncoderSet encoders;
    private static TRCGyroBase gyro;
    private static Object drive;

    private static boolean driving;
    private static double maxSpeed = 0.0;
    private static DriveType driveType;

    private static TRCSpeed autoSpeed;

    private static TRCVector mVector;

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
        TRCNetworkData.createDataPoint("PIDSetpoint_X");
        TRCNetworkData.createDataPoint("PIDSetpoint_Y");
        TRCNetworkData.createDataPoint("PIDSetpoint_Z");
        TRCNetworkData.createDataPoint("PIDSetpoint_Generic");

        TRCNetworkData.createDataPoint("PIDOutput_X");
        TRCNetworkData.createDataPoint("PIDOutput_Y");
        TRCNetworkData.createDataPoint("PIDOutput_Z");
        TRCNetworkData.createDataPoint("PIDOutput_Generic");
        TRCNetworkData.createDataPoint("PIDOutputSmoothed_X");
        TRCNetworkData.createDataPoint("PIDOutputSmoothed_Y");
        TRCNetworkData.createDataPoint("PIDOutputSmoothed_Z");
        TRCNetworkData.createDataPoint("PIDOutputSmoothed_Generic");
    }

    /**
     * Executes the specified action
     * 
     * @param driveAction The type of action the robot should take (one of
     *                    {@link DriveActionType})
     * @param unit        The inches/degrees of the action
     */
    public static void run(DriveActionType driveAction, double unit)
    {
        TRCVector vector = new TRCVector(unit);
        run(driveAction, vector);
    }

    /**
     * Executes the specified action
     * 
     * @param driveAction The type of action the robot should take (one of
     *                    {@link DriveActionType})
     * @param vector      The vector of the action
     */
    public static void run(DriveActionType driveAction, TRCVector vector)
    {
        driving = false;
        mVector = vector;

        autoSpeed = new TRCSpeed();

        switch(driveAction)
        {
            case Forward: driveForwardBack(); break;
            case Right: driveLeftRight(); break;
            case Rotate: driveRotation(); break;
            case Direct: driveDirect(); break;
            default: break;
        }
    }

    /**
     * Drive the robot directly with the specified vector, only works with the
     * Mecanum drive type
     */
    public static void driveDirect()
    {
        if (mVector.getType() != TRCVector.TRCVECTORTYPE_3D)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveDirect(): not given a 3D vector");
            return;
        }

        if (driveType != DriveType.Mecanum)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveDirect(): tried to drive without Mecanum drive");
            return;
        }
        
        if (!driving)
        {
            prepareToDrive();
            driving = true;
            
            MiniPID xPID = mVector.getLinearPID();
            MiniPID yPID = mVector.getStrafePID();
            MiniPID zPID = mVector.getRotationPID();
    
            int xdeadbandcounter = 0, ydeadbandcounter = 0, zdeadbandcounter = 0;
            boolean xdone = false, ydone = false, zdone = false;
            while (RobotState.isAutonomous()) // infinite loop if the robot is autonomous
            {
                // TODO: ALL ENCODER STUFFS NEED TO BE FIXED
                double x = mVector.getLinear();
                double y = mVector.getStrafe();
                double z = mVector.getRotation();

                double xNewSpeed = xPID.getOutput(encoders.getAverageDistanceTraveled()); // TODO:
                double yNewSpeed = yPID.getOutput(encoders.getAverageDistanceTraveled()); // TODO:
                double zNewSpeed = zPID.getOutput(gyro.getAngle());
 
                double xSmoothedSpeed = autoSpeed.calculateSpeed(xNewSpeed, 1.0);
                double ySmoothedSpeed = autoSpeed.calculateSpeed(yNewSpeed, 1.0);
                double zSmoothedSpeed = autoSpeed.calculateSpeed(zNewSpeed, 1.0);

                /* UPDATE DATA POINTS */
                String[] dpNames = {
                    "PIDSetpoint_X", "PIDSetpoint_Y", "PIDSetpoint_Z",
                    "PIDOutput_X", "PIDOutput_Y", "PIDOutput_Z",
                    "PIDOutputSmoothed_X", "PIDOutputSmoothed_Y", "PIDOutputSmoothed_Z"
                };
                Object[] dpValues = {
                    x, y, z,
                    xNewSpeed, yNewSpeed, zNewSpeed,
                    xSmoothedSpeed, ySmoothedSpeed, zSmoothedSpeed
                };
                updateDriveDataPoints(dpNames, dpValues);
                /* =================== */

                // vvv =============================================================================== vvv
                ((TRCMecanumDrive) drive).driveCartesian(-ySmoothedSpeed, xSmoothedSpeed, zSmoothedSpeed);
                // ^^^ ============================== ACTUALLY DRIVE ================================= ^^^

                if (Math.abs(encoders.getAverageDistanceTraveled() - mVector.getStrafe()) < deadband) xdeadbandcounter++; // TODO:
                if (xdeadbandcounter >= verificationMin) xdone = true;
                if (Math.abs(encoders.getAverageDistanceTraveled() - mVector.getLinear()) < deadband) ydeadbandcounter++; // TODO:
                if (ydeadbandcounter >= verificationMin) ydone = true;
                if (Math.abs(gyro.getAngle() - mVector.getRotation()) < deadband) zdeadbandcounter++;
                if (zdeadbandcounter >= verificationMin) zdone = true;

                if (xdone && ydone && zdone) break;
            }
            driving = false;
        }
    }

    /**
     * Drive the robot forward (measurement) inches
     */
    public static void driveForwardBack()
    {
        if (mVector.getType() != TRCVector.TRCVECTORTYPE_1D) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveForwardBack(): not given a 1D vector");
            return;
        }

        MiniPID PID = mVector.getUnitPID();
        if (!driving)
        {
            prepareToDrive();
    
            driving = true;
            int deadbandcounter = 0;

            while (deadbandcounter < verificationMin && RobotState.isAutonomous())
            {
                double newSpeed = PID.getOutput(encoders.getAverageDistanceTraveled());
                double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 0.85);

                String[] dpNames = {"PIDOutput_Generic", "PIDOutputSmoothed_Generic"};
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
                if (Math.abs(encoders.getAverageDistanceTraveled() - mVector.getUnit()) < deadband) 
                { // if in the deadband
                    deadbandcounter++;
                }
            }
            driving = false;
        }
    }

    /**
     * Drive the robot right (measurement) inches (exclusive to mecanum)
     */
    public static void driveLeftRight()
    {
        if (mVector.getType() != TRCVector.TRCVECTORTYPE_1D) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveLeftRight(): not given a 1D vector");
            return;
        }

        if (driveType != DriveType.Mecanum)
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveLeftRight(): tried to strafe without Mecanum drive");
            return;
        }

        MiniPID PID = mVector.getUnitPID();
        if (!driving)
        {
            prepareToDrive();

            driving = true;
            int deadbandcounter = 0;

            while (deadbandcounter < verificationMin && RobotState.isAutonomous())
            {
                double newSpeed = PID.getOutput(encoders.getAverageDistanceTraveled());
                double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

                String[] dpNames = {"PIDOutput_Generic", "PIDOutputSmoothed_Generic"};
                Object[] dpValues = {newSpeed, smoothedSpeed};
                updateDriveDataPoints(dpNames, dpValues);

                ((TRCMecanumDrive) drive).driveCartesian(0.0, smoothedSpeed, 0.0);

                if (Math.abs(encoders.getAverageDistanceTraveled() - mVector.getUnit()) < deadband) 
                { // if in the deadband
                    deadbandcounter++;
                }
            }
            driving = false;
        }
    }

    /**
     * Rotate the robot (measurement) degrees
     */
    public static void driveRotation()
    {
        if (mVector.getType() != TRCVector.TRCVECTORTYPE_1D) 
        {
            TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: driveRotation(): not given a 1D vector");
            return;
        }

        MiniPID PID = mVector.getUnitPID();
        if (!driving)
        {
            prepareToDrive();

            driving = true;
            int deadbandcounter = 0;

            while (deadbandcounter < verificationMin && RobotState.isAutonomous())
            {
                double newSpeed = PID.getOutput(gyro.getAngle());
                double smoothedSpeed = autoSpeed.calculateSpeed(newSpeed, 1.0);

                String[] dpNames = {"PIDOutput_Generic", "PIDOutputSmoothed_Generic"};
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

                if (Math.abs(gyro.getAngle() - mVector.getUnit()) < deadband) 
                { // if in the deadband
                    deadbandcounter++;
                }
            }
            driving = false;
        }
    }

    /**
     * Prepare PIDs for driving & and update data points
     */
    private static void prepareToDrive()
    {
        autoSpeed.reset();

        if (mVector.getType() == TRCVector.TRCVECTORTYPE_3D) 
        {
            MiniPID xPID = mVector.getLinearPID();
            MiniPID yPID = mVector.getStrafePID();
            MiniPID zPID = mVector.getRotationPID();

            // reset PIDS
            xPID.reset();
            yPID.reset();
            zPID.reset();

            // set setpoints
            xPID.setSetpoint(mVector.getLinear());
            yPID.setSetpoint(mVector.getStrafe());
            zPID.setSetpoint(mVector.getRotation());

            // update data points
            TRCNetworkData.updateDataPoint("PIDSetpoint_X", mVector.getLinear());
            TRCNetworkData.updateDataPoint("PIDSetpoint_Y", mVector.getStrafe());
            TRCNetworkData.updateDataPoint("PIDSetpoint_Z", mVector.getRotation());
            TRCNetworkData.updateDataPoint("PIDSetpoint_Generic", 0.0);

            // set output limits
            xPID.setOutputLimits(-maxSpeed, maxSpeed);
            yPID.setOutputLimits(-maxSpeed, maxSpeed);
            zPID.setOutputLimits(-maxSpeed, maxSpeed);
        }
        else if (mVector.getType() == TRCVector.TRCVECTORTYPE_1D)
        {
            MiniPID PID = mVector.getUnitPID();

            PID.reset();
            PID.setSetpoint(mVector.getUnit());

            TRCNetworkData.updateDataPoint("PIDSetpoint_X", 0.0);
            TRCNetworkData.updateDataPoint("PIDSetpoint_Y", 0.0);
            TRCNetworkData.updateDataPoint("PIDSetpoint_Z", 0.0);
            TRCNetworkData.updateDataPoint("PIDSetpoint_Generic", mVector.getUnit());

            PID.setOutputLimits(-maxSpeed, maxSpeed);
        }
    }

    /**
     * Update driving datapoints based on if the vector is 1D or 3D
     * 
     * @param names  An array of the names of the Data Points to update
     * @param values An array of the values of the Data Points to update
     */
    private static void updateDriveDataPoints(String[] names, Object[] values)
    {
        if (mVector.getType() == TRCVector.TRCVECTORTYPE_1D)
        {
            TRCNetworkData.updateDataPoint("PIDSetpoint_X", 0.0);
            TRCNetworkData.updateDataPoint("PIDSetpoint_Y", 0.0);
            TRCNetworkData.updateDataPoint("PIDSetpoint_Z", 0.0);
            TRCNetworkData.updateDataPoint("PIDOuput_X", 0.0);
            TRCNetworkData.updateDataPoint("PIDOuput_Y", 0.0);
            TRCNetworkData.updateDataPoint("PIDOuput_Z", 0.0);
            TRCNetworkData.updateDataPoint("PIDOutputSmoothed_X", 0.0);
            TRCNetworkData.updateDataPoint("PIDOutputSmoothed_Y", 0.0);
            TRCNetworkData.updateDataPoint("PIDOutputSmoothed_Z", 0.0);
        }
        else if (mVector.getType() == TRCVector.TRCVECTORTYPE_3D)
        {
            TRCNetworkData.updateDataPoint("PIDSetpoint_Generic", 0.0);
            TRCNetworkData.updateDataPoint("PIDOuput_Generic", 0.0);
            TRCNetworkData.updateDataPoint("PIDOutputSmoothed_Generic", 0.0);
        }

        if (names.length != values.length) TRCNetworkData.logString(VerbosityType.Log_Error, "TRCDrivePID: updateDriveDataPoints(): names and values have different sizes, points not updated.");

        for (int index = 0; index < names.length; index++)
        {
            String name = names[index];
            Object value = values[index];
            TRCNetworkData.updateDataPoint(name, value);
        }
    }
}