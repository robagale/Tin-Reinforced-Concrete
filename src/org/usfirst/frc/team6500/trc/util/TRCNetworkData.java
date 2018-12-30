package org.usfirst.frc.team6500.trc.util;

import org.usfirst.frc.team6500.trc.util.TRCTypes.DataInterfaceType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.VerbosityType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.HashMap;

/**
 * Send data from the robot to the driver station by some means, either simply through NetworkTables or
 * (Shuffle/SmartDash)Board
 */
public class TRCNetworkData
{
    private static DataInterfaceType interfaceType;
    private static NetworkTableInstance tableServer;
    private static NetworkTable table;
    private static NetworkTableEntry logEntry;
    
    private static HashMap<Integer, SendableChooser<Integer>> options;
    private static HashMap<String, NetworkTableEntry> dataPoints;

    private static int vOptionsID;


    /**
     * Setup the sending of data to the driver station specifing with DIType how it should be done.
     * 
     * @param DIType Which interface should be used for sending data, see {@link DataInterfaceType}.  Board is preferred except in select scenarios.
     */
    public static void initializeNetworkData(DataInterfaceType DIType)
    {
        interfaceType = DIType;
        tableServer = NetworkTableInstance.getDefault();
        if (!tableServer.isConnected()) { tableServer.startServer(); }

        switch (interfaceType)
        {
            case NetworkTables:
                table = tableServer.getTable("/data");
                break;
            case Board:
                table = tableServer.getTable("/SmartDashboard");
                break;
            default:
                break;
        }

        logEntry = table.getEntry("log");
        logEntry.setString("Logging Initialized.");

        String verbosityOptions[] = new String[3];
        verbosityOptions[0] = "Debug (All Messages)";
        verbosityOptions[1] = "Info. (Limited Messages)";
        verbosityOptions[2] = "Error (Critical Messages)";
        vOptionsID = putOptions(verbosityOptions);
    }

    public static VerbosityType getVerbosity()
    {
        switch (getSelection(vOptionsID))
        {
            case 0:
                return VerbosityType.Log_Debug;
            case 1:
                return VerbosityType.Log_Info;
            case 2:
                return VerbosityType.Log_Error;
            default:
                return null;
        }
    }

    /**
     * Add another line to the log entry, deleting the oldest line if there are too many.
     * 
     * @param v At what level of informationality the line should be displayed
     * @param logData Line to add the log
     */
    public static void logString(VerbosityType v, String logData)
    {
        if (getVerbosity().ordinal() > v.ordinal())
        {
            return;
        }

        String oldLog = logEntry.getString("");

        if (oldLog == "")
        {
            System.out.println("ERROR: NetworkTable Server has broken.  Something has gone critically wrong with networking.");
            return;
        }

        int lineCount = 1;
        String cutLog = oldLog;
        while (cutLog.contains("\n"))
        {
            if (lineCount >= 10)
            {
                oldLog = oldLog.substring(oldLog.indexOf("\n") + 1, oldLog.length() + 1);
                break;
            }
            cutLog = cutLog.substring(cutLog.indexOf("\n"), cutLog.length() - 1);
            lineCount++;
        }
        
        String newLog = oldLog + "\n" + logData;
        logEntry.setString(newLog);
    }

    /**
     * Put some radio buttons containing choices on the SmartDashboard/ShuffleBoard the user can choose from to be read from later
     * 
     * @param choices The options the user should have to choose from
     * @return Identifier to be used with getSelection to find out what choice was selected.
     */
    public static int putOptions(String[] choices)
    {
        SendableChooser<Integer> chooser = new SendableChooser<Integer>();
        chooser.addDefault(choices[0], 0);
        for (int i = 0; i < choices.length; i++)
        {
            chooser.addObject(choices[i], i);
        }

        options.put(options.size(), chooser);
        return options.size() - 1;
    }

    /**
     * Use an identifier obtained from putOptions to find out the option that is currently selected
     * 
     * @param chooserNum The identifier for the previously created options
     * @return Number of the option the user has selected
     */
    public static int getSelection(int chooserNum)
    {
        return options.get(chooserNum).getSelected();
    }

    /**
     * Create a data point in the NetworkTable/Board to update in the future with the indentifier name
     * 
     * @param name The identifying name of the data point
     */
    public static void createDataPoint(String name)
    {
        NetworkTableEntry entry = table.getEntry(name);
        dataPoints.put(name, entry);
    }

    /**
     * Set the value of a previously created data point in the NetworkTable/Board
     * 
     * @param name Identifier of the data point
     * @param value What to set the data point to
     */
    public static void updateDataPoint(String name, Object value)
    {
        dataPoints.get(name).setValue(value);
    }
}