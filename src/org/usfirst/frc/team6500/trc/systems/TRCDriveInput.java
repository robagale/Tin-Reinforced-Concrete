/*
 *	TRCDriveInput
 * 		totally static class to handle input from the controller
 */

package org.usfirst.frc.team6500.trc.systems;

import java.util.HashMap;

import org.usfirst.frc.team6500.trc.util.TRCController;
import org.usfirst.frc.team6500.trc.util.TRCDriveParams;
import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.util.TRCTypes.VerbosityType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

public class TRCDriveInput
{
	private static XboxController controller;

    private static HashMap<Integer, Runnable[]> bindings;  // <button key, [pressed action, released action]>

	private static double baseSpeed = 0.0;
    private static double boostSpeed = 0.0;
    
	/**
	 * Setup the DriveInput class.  Do this before using any other methods in this class.
	 * 
	 * @param ports The ids of the USB ports that the controller is plugged in to
	 * @param speedBase The default base speed of the robot
	 */
	public static void initializeDriveInput(int port, double speedBase, double speedBoost)
	{
		bindings = new HashMap<Integer, Runnable[]>();

		controller = new XboxController(port);
		
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
	private static String buttonToString(int button)
	{
		switch (button)
		{
			case TRCController.BUTTON_A: return "A button";
			case TRCController.BUTTON_B: return "B button";
			case TRCController.BUTTON_X: return "X button";
			case TRCController.BUTTON_Y: return "Y button";
			case TRCController.BUTTON_START: return "Start button";
			case TRCController.BUTTON_BACK: return "Back button";
			case TRCController.BUMPER_LEFT: return "Left bumper";
			case TRCController.BUMPER_RIGHT: return "Right bumper";
		}
		return "<unrecognized button type>";
	}
    
	/**
	 * Checks every button on the controller, and if the button is pressed and has a
	 * function bound to it then the function will be run
	 */
	public static void updateBindings() {
		for (Integer button : bindings.keySet()) // get all supported buttons in pressFuncs
		{
			boolean isPressed = getButton(button);
			if (isPressed)
				bindings.get(button)[0].run();
			else
				bindings.get(button)[1].run();
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
		switch (button) 
		{
		case TRCController.BUTTON_A: return controller.getAButton();
		case TRCController.BUTTON_B: return controller.getBButton();
		case TRCController.BUTTON_X: return controller.getXButton();
		case TRCController.BUTTON_Y: return controller.getYButton();
		case TRCController.BUTTON_START: return controller.getStartButton();
		case TRCController.BUTTON_BACK: return controller.getBackButton();
		case TRCController.BUMPER_LEFT: return controller.getBumper(Hand.kLeft);
		case TRCController.BUMPER_RIGHT: return controller.getBumper(Hand.kRight);
		}
		return false;
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
		
		multiplier = getRawThrottle(); // Range is -1 to 1, change to 0 to 2 cuz its easier to work with
        multiplier = 1 - multiplier;   // Throttle is backwards from expectation, flip it
        if (controller.getPOV() != 1)
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
