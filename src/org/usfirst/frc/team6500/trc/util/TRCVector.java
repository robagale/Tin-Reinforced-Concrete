package org.usfirst.frc.team6500.trc.util;

import org.usfirst.frc.team6500.trc.auto.MiniPID;

public class TRCVector
{
    private double x, y, z;
    private MiniPID xPID, yPID, zPID;
    private double unit;
    private MiniPID PID;

    private int type;

    public static final int TRCVECTORTYPE_3D = 0;
    public static final int TRCVECTORTYPE_1D = 1;

    /**
     * Create a new 3D TRCVector object.
     * 
     * @param linear The forward/back measurment of the vector
     * @param strafe The side to side measurment of the vector
     * @param rotation The rotational degrees of the vector
     */
    public TRCVector(double linear, double strafe, double rotation)
    {
        this.x = strafe;
        this.y = linear;
        this.z = rotation;
        this.type = TRCVECTORTYPE_3D;

        xPID = new MiniPID(-1.0, 0.0, 0.0);
        yPID = new MiniPID(-1.0, 0.0, 0.0);
        zPID = new MiniPID(-1.0, 0.0, 0.0);
    }

    /**
     * Create a new single dimensional TRCVector object.
     * 
     * @param unit The measurment of the vector
     */
    public TRCVector(double unit)
    {
        this.unit = unit;
        this.type = TRCVECTORTYPE_1D;

        PID = new MiniPID(-1.0, 0.0, 0.0);
    }

    public int getType() { return this.type; }


    public double getStrafe() 
    { 
        if (this.type != TRCVECTORTYPE_3D) return -1;
        return this.x; 
    }
    public double getLinear() 
    { 
        if (this.type != TRCVECTORTYPE_3D) return -1;
        return this.y; 
    }
    public double getRotation()
    {
        if (this.type != TRCVECTORTYPE_3D) return -1;
        return this.z;
    }
    public MiniPID getLinearPID()
    {
        if (this.type != TRCVECTORTYPE_3D) return null;
        return this.yPID;
    }
    public MiniPID getStrafePID()
    {
        if (this.type != TRCVECTORTYPE_3D) return null;
        return this.xPID;
    }
    public MiniPID getRotationPID()
    {
        if (this.type != TRCVECTORTYPE_3D) return null;
        return this.zPID;
    }


    public double getUnit()
    {
        if (this.type != TRCVECTORTYPE_1D) return -1;
        return this.unit;
    }
    public MiniPID getUnitPID()
    {
        if (this.type != TRCVECTORTYPE_1D) return null;
        return this.PID;
    }
}