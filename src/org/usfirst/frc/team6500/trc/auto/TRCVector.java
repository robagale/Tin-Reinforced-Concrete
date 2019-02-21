package org.usfirst.frc.team6500.trc.auto;

import org.usfirst.frc.team6500.trc.util.TRCTypes.Direction;
import org.usfirst.frc.team6500.trc.util.TRCTypes.DriveAction;
import org.usfirst.frc.team6500.trc.util.TRCTypes.RobotSide;
import org.usfirst.frc.team6500.trc.util.TRCTypes.UnitType;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class TRCVector
{
    private DriveAction type;
    protected UnitType unitType;
    protected double value = 0.0;

    private boolean isValid = true;

    /* THESE VARS FOR SPECIFIC ACTIONS */
    private Direction direction; // used for Diagonal, only use ForwardLeft, ForwardRight, BackwardLeft, & BackwardRight
    private MotorType turnWheel; // used for OnCorner
    private RobotSide turnSide; // used for OnAxis

    /**
     * Create a generic TRCVector object to use with navigation, useful for
     * ForwardBack, LeftRight, & Rotation. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param type     the type of DriveAction to store
     * @param value    the measurement or value of the type
     * @param unitType the type of measurement
     */
    public TRCVector(DriveAction type, double value, UnitType unitType)
    {
        if (!this.validate(type, value, unitType))
        {
            isValid = false;
        } 
        else 
        {
            this.type = type;
            this.value = value;
            this.unitType = unitType;
        }
    }

    /**
     * Create an autorecognized TRCVector object to use with navigation. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param type     the type of DriveAction to store
     * @param value    the measurement or value of the type
     * @param unitType the type of measurement
     * @param object   the object to pass to the drive action
     */
    public TRCVector(DriveAction type, double value, UnitType unitType, Object object)
    {
        if (!this.validate(type, value, unitType))
        {
            isValid = false;
        } 
        else 
        {
            if (object.getClass() == Direction.class && type == DriveAction.Diagonal)
            {
                this.isValid = validateDiagonal((Direction)object);
                this.direction = (Direction)object;
            }
            else if (object.getClass() == MotorType.class && type == DriveAction.OnCorner)
            {
                this.turnWheel = (MotorType)object;
            }
            else if (object.getClass() == RobotSide.class && type == DriveAction.OnAxis)
            {
                this.isValid = validateOnAxis((RobotSide)object);
                this.turnSide = (RobotSide)object;
            }
            else if (!(object == null && (type == DriveAction.ForwardBack || type == DriveAction.LeftRight || type == DriveAction.Rotation))) // test to see if the object is supposed to be null
            {
                isValid = false;
            }
            this.type = type;
            this.value = value;
            this.unitType = unitType;
        }
    }

    /**
     * Create a Diagonal TRCVector object to use with navigation. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param value     the measurement or value of the type
     * @param unitType  the type of measurement
     * @param direction the direction in which to move the robot, only ForwardLeft,
     *                  ForwardRight, BackwardLeft, & BackwardRight are supported
     */
    public TRCVector(double value, UnitType unitType, Direction direction)
    {
        if (!this.validate(type, value, unitType))
        {
            isValid = false;
        } 
        else 
        {
            this.isValid = validateDiagonal(direction);
            this.type = DriveAction.Diagonal;
            this.value = value;
            this.unitType = unitType;
            this.direction = direction;
        }
    }

    /**
     * Create a cornering (OnCorner) TRCVector object to use with navigation. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param value     the measurement or value of the type
     * @param unitType  the type of measurement
     * @param turnWheel the wheel on which the robot will turn on
     */
    public TRCVector(double value, UnitType unitType, MotorType turnWheel)
    {
        if (!this.validate(type, value, unitType))
        {
            isValid = false;
        } 
        else 
        {
            this.type = DriveAction.OnCorner;
            this.value = value;
            this.unitType = unitType;
            this.turnWheel = turnWheel;
        }
    }

    /**
     * Create a side rotating (OnAxis) TRCVector object to use with navigation. See
     * https://www.roboteq.com/images/article-images/frontpage/wheel-rotations.jpg
     * for details.
     * 
     * @param value    the measurement or value of the type
     * @param unitType the type of measurement
     * @param turnSide the side on which to turn the robot, only kFront & kRear are
     *                 supported
     */
    public TRCVector(double value, UnitType unitType, RobotSide turnSide)
    {
        if (!this.validate(type, value, unitType))
        {
            isValid = false;
        } 
        else 
        {
            if (!(turnSide == RobotSide.kFront || turnSide == RobotSide.kRear))
            {
                isValid = false; // if an invalid direction, invalidate the vector
            }
            this.type = DriveAction.OnAxis;
            this.value = value;
            this.unitType = unitType;
            this.turnSide = turnSide;
        }
    }

    protected boolean validateDiagonal(Direction direction)
    {
        if (!(direction == Direction.ForwardLeft || 
              direction == Direction.ForwardRight ||
              direction == Direction.BackwardLeft ||
              direction == Direction.BackwardRight))
        {
            return false;
        }
        return true;
    }

    protected boolean validateOnAxis(RobotSide turnSide)
    {
        if (!(turnSide == RobotSide.kFront || turnSide == RobotSide.kRear))
        {
            return false;
        }
        return true;
    }

    protected boolean validate(DriveAction type, double value, UnitType unitType)
    {

        if ((type == DriveAction.ForwardBack || 
               type == DriveAction.LeftRight || 
               type == DriveAction.Diagonal) && 
               unitType == UnitType.Degrees) return false;
        else if ((type == DriveAction.Rotation || 
                  type == DriveAction.OnCorner || 
                  type == DriveAction.OnAxis) && 
                  unitType == UnitType.Inches) return false;

        switch (unitType)
        {
            case General:
                if (value >= -1.0 && value <= 1.0) return true; // needs to be in between -1.0 and 1.0
                return false;
            case Inches: return true; // can be anything
            case Degrees:
                if (value >= 0 && value <= 360) return true; // needs to be greater than 0 and less than 360
                return false;
            default: return false;
        }
    }

    /* =============================================================================================================== */

    public DriveAction getType() { return this.type; }
    public UnitType getUnitType() { return this.unitType; }
    public double getValue() { return this.value; }
    public boolean getIsValid() { return this.isValid; }

    public Direction getDirection() 
    {
        if (this.type != DriveAction.Diagonal) 
        {
            this.isValid = false;
            return null;
        }
        return this.direction;
    }
    public MotorType getTurnWheel() 
    { 
        if (this.type != DriveAction.OnCorner) 
        {
            this.isValid = false;
            return null;
        }
        return this.turnWheel; 
    }
    public RobotSide getTurnSide()
    { 
        if (this.type != DriveAction.OnAxis) 
        {
            this.isValid = false;
            return null;
        }
        return this.turnSide; 
    }

    /**
     * Translates from the current {@link UnitType} to the specified
     * {@link UnitType}
     * 
     * @param unitType the {@link UnitType} to translate into
     * @return the translated version of the value
     */
    public double translateValue(UnitType unitType)
    {        
        switch (unitType) 
        {
        case General:
            switch (this.unitType) 
            {
                case General: return value;
                case Inches:
                    if (value >= 12) return 1.0;
                    if (value <= -12) return -1.0;
                    return ((100 / 12) * value) * 0.01;
                case Degrees: return ((1000 / 360) * value) * 0.001;
            }
            break;
        case Inches:
            switch (this.unitType) 
            {
                case General: return 12*value;
                case Inches: return value;
                case Degrees: return 0;
            }
            break;
        case Degrees:
            switch (this.unitType) 
            {
                case General: return 360*value;
                case Inches: return 0;
                case Degrees: return value;
            }
            break;
        }
        return 0;
    }
}
