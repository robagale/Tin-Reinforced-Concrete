/*
 *	TRCDriveInput
 * 		totally static class to handle input from the controller
 */

package org.usfirst.frc.team6500.trc.systems;

import java.util.HashMap;

import org.usfirst.frc.team6500.trc.util.TRCController;
import org.usfirst.frc.team6500.trc.util.TRCDriveParams;
import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.util.TRCTypes.ControllerType;
import org.usfirst.frc.team6500.trc.util.TRCTypes.VerbosityType;



public class TRCDriveInput
{
	private static ArrayList<TRCController> inputSticks;
	// private static HashMap<Integer, Joystick> inputSticks; removed this because you can just <joystick>.getPort()

    private static HashMap<Integer, HashMap<Object, Runnable>> pressFuncs; 
	private static HashMap<Integer, HashMap<Object[], Runnable>> absenceFuncs; // oh god why does this work  // spelled "absence", not "absense" :P
	/* Array in a HashMap in a HashMap... */
	private static double baseSpeed = 0.0;
    private static double boostSpeed = 0.0;
    
	/**
	 * Setup the DriveInput class.  Do this before using any other methods in this class.
	 * 
	 * @param ports The ids of the USB ports that the controller is plugged in to
	 * @param speedBase The default base speed of the robot
	 */
	public static void initializeDriveInput(int[] ports, ControllerType types[], double speedBase, double speedBoost)
	{
		inputSticks = new ArrayList<TRCController>();
		pressFuncs = new HashMap<Integer, HashMap<Object, Runnable>>();
		absenceFuncs = new HashMap<Integer, HashMap<Object[], Runnable>>();

		for (int i = 0; i < ports.length; i++)
		{
			inputSticks.add(new TRCController(ports[i], types[i]));
			pressFuncs.put(ports[i], new HashMap<Object, Runnable>());
			absenceFuncs.put(ports[i], new HashMap<Object[], Runnable>());
		}
		
        baseSpeed = speedBase;
        boostSpeed = speedBoost;
        
		TRCNetworkData.logString(VerbosityType.Log_Info, "Driver Input is online.");
	}
	
	/**
	 * Assign a function to be run when a certain button on the controller is pressed
	 * 
	 * @param button Button to bind to
	 * @param func Function to be run
	 */
	public static void bindButton(int button, Runnable[] func)
	{
		bindings.put(button, func);
		TRCNetworkData.logString(VerbosityType.Log_Debug, "A binding has been created for the " + buttonToString(button) + " on the connected controller");
	}

	/**
	 * Get the string representation of a button index
	 * 
	 * @param button The button index to test
	 * @return The string representation of the passed button index
	 */
	public static void bindButtonAbsence(int joystickPort, Object[] buttons, Runnable func)
	{
		absenceFuncs.get(joystickPort).put(buttons, func);
		TRCNetworkData.logString(VerbosityType.Log_Debug, "An absence binding has been created for " + buttons.length + "buttons on Joystick " + joystickPort);
	}
    
	/**
	 * Checks every button on the controller, and if the button is pressed and has a
	 * function bound to it then the function will be run
	 */
	public static void checkButtonBindings()
	{
		for (int index = 0; index < inputSticks.size(); index++) // get all input sticks (2; gunner and driver)
		{
			int stickPort = inputSticks.get(index).getPort(); // get the port of that joystick

			if (pressFuncs.containsKey(stickPort)) // if the joystick is supported on pressFuncs
			{
				for (Object button : pressFuncs.get(stickPort).keySet()) // get all supported buttons in pressFuncs
				{
					if (inputSticks.get(stickPort).getButton(button)) // if the button is pressed, run it's runnable
					{
						//System.out.println(button);
						pressFuncs.get(stickPort).get(button).run(); // RUN!
					}
				}
			}
			
			if (absenceFuncs.containsKey(stickPort)) // if the joystick is supported on absenceFuncs
			{
            	for (Object[] buttonList : absenceFuncs.get(stickPort).keySet()) // get all supported buttons in absenceFuncs
				{
					for (int i = 0; i < buttonList.length; i++) // check all buttons in the button list
					{
						if (!inputSticks.get(stickPort).getButton(buttonList[i])) // if the button is not pressed...
						{
							if (i == buttonList.length - 1)
							{
								absenceFuncs.get(inputSticks.get(stickPort).getPort()).get(buttonList).run();
								break;
							}
							continue;
						}
						break;
					} // This part of the function runs through all of the absence bindings for all joysticks,
					  // and if after going through all of them not being pressed on the last one it activates the bound function
				}
			}
		}
	}

	/**
	 * Get whether a certain button on the controller is currently being pressed
	 * 
	 * @param button Number of button to check
	 * @return True if button on joystick is pressed, false otherwise
	 */
	public static boolean getButton(int button)
	{
		return inputSticks.get(joystickPort).getButton(button);
	}

	public static TRCController getController(int joystickPort)
	{
		return inputSticks.get(joystickPort);
	}
	
	/**
	 * Get the POV (D-Pad or thumbstick) position from a controller
	 * 
	 * @return The position, in degrees, of the POV
	 */
	public static int getPOV()
	{
		return controller.getPOV();
	}
	
	/**
	 * Calculates the value of the throttle in a manner which makes the number much
	 * more sensible
	 * 
	 * @return The simplified throttle value
	 */
	public static double getThrottle() 
	{
		double multiplier;
		
		multiplier = getRawThrottle(joystickPort) + 1;        // Range is -1 to 1, change to 0 to 2 cuz its easier to work with
		multiplier = multiplier / 2;                          // Reduce to a scale between 0 to 1
        multiplier = 1 - multiplier;                          // Throttle is backwards from expectation, flip it
        if (!inputSticks.get(joystickPort).getButton(1))
        {
            multiplier = multiplier * baseSpeed; // Mix in some of that default...
        }
        else
        {
            multiplier = multiplier * boostSpeed; // Unless the trigger is pressed, then mix in some of that sweet boost :)
        }
		
		return multiplier;
	}

	/**
	 * Get the raw value of a joystick's throttle. This value is complicated to work
	 * with, use getThrottle for a more workable version
	 * 
	 * @return The value from the throttle which is returned by default
	 */
	public static double getRawThrottle()
	{
		return controller.getTriggerAxis(Hand.kRight);
	}
	
	/**
	 * Get a TRCDriveParams which has all the values from the controller for use in
	 * driving the robot
	 * 
	 * @return TRCDriveParams which have been set from values from the controller
	 */
	public static TRCDriveParams getStickDriveParams()
	{
		TRCDriveParams params = new TRCDriveParams();
		
		double x = controller.getX(Hand.kRight);
		double y = controller.getY(Hand.kRight);
		double z = controller.getX(Hand.kLeft);

		if (x < 0.1 && x > -0.1) x = 0.0;
		if (y < 0.1 && y > -0.1) y = 0.0;
		if (z < 0.1 && z > -0.1) z = 0.0;

		params.setRawX(x);
		params.setRawY(y);
		params.setRawZ(z);

		params.setM(getThrottle());
		
		return params;
	}
}
