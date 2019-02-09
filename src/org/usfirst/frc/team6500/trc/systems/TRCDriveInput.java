/*
 *	TRCDriveInput
 * 		totally static class to handle input from the joysticks
 */

package org.usfirst.frc.team6500.trc.systems;

import java.util.HashMap;

import org.usfirst.frc.team6500.trc.util.TRCDriveParams;
import org.usfirst.frc.team6500.trc.util.TRCNetworkData;
import org.usfirst.frc.team6500.trc.util.TRCTypes.VerbosityType;
import org.usfirst.frc.team6500.trc.util.TRCController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class TRCDriveInput
{
	private static XboxController controller;

    private static HashMap<Integer, Runnable> pressFuncs; 

	private static double baseSpeed = 0.0;
    private static double boostSpeed = 0.0;
    
	/**
	 * Setup the DriveInput class.  Do this before using any other methods in this class.
	 * 
	 * @param ports The ids of the USB ports the joysticks are plugged in to
	 * @param speedBase The default base speed of the robot
	 */
	public static void initializeDriveInput(int port, double speedBase, double speedBoost)
	{
		pressFuncs = new HashMap<Integer, Runnable>();

		controller = new XboxController(port);
		
        baseSpeed = speedBase;
        boostSpeed = speedBoost;
        
		TRCNetworkData.logString(VerbosityType.Log_Info, "Driver Input is online.");
	}
	
	/**
	 * Assign a function to be run when a certain button on a certain joystick is pressed
	 * 
	 * @param button Button to bind to
	 * @param func Function to be run
	 */
	public static void bindButtonPress(int button, Runnable func)
	{
		pressFuncs.put(button, func);
		TRCNetworkData.logString(VerbosityType.Log_Debug, "A binding has been created for the " + buttonToString(button) + " on the connected controller");
	}
	
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
		return "";
	}
    	
	/**
	 * Checks every button on every Joystick, and if the button is pressed and has a function bound to it then
	 * the function will be run
	 */
	public static void updateBindings() 
	{
		int controllerPort = controller.getPort(); // get the port of the controller

		if (pressFuncs.containsKey(controllerPort)) // if the joystick is supported on pressFuncs
		{
			for (Integer button : pressFuncs.keySet()) // get all supported buttons in pressFuncs
			{
				boolean isPressed = getButton(button);
				if (isPressed) pressFuncs.get(button).run();
			}
		}
	}
	
	/**
	 * Get whether a certain button on a certain joystick is currently being pressed
	 * 
	 * @param button Number of button to check
	 * @return True if button on joystick is pressed, false otherwise
	 */
	public static boolean getButton(int button)
	{
		switch (button) {
		case TRCController.BUTTON_A:
			if (controller.getAButton()) return true;
			return false;
		case TRCController.BUTTON_B:
			if (controller.getBButton()) return true;
			return false;
		case TRCController.BUTTON_X:
			if (controller.getXButton()) return true;
			return false;
		case TRCController.BUTTON_Y:
			if (controller.getYButton()) return true;
			return false;
		case TRCController.BUTTON_START:
			if (controller.getStartButton()) return true;
			return false;
		case TRCController.BUTTON_BACK:
			if (controller.getBackButton()) return true;
			return false;
		case TRCController.BUMPER_LEFT:
			if (controller.getBumper(Hand.kLeft)) return true;
			return false;
		case TRCController.BUMPER_RIGHT:
			if (controller.getBumper(Hand.kRight)) return true;
			return false;
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
	 * Calculates the value of the throttle in a manner which makes the number much more sensible
	 * 
	 * @return The simplified throttle value
	 */
	public static double getThrottle() 
	{
		double multiplier;
		
		multiplier = getRawThrottle();        // Range is -1 to 1, change to 0 to 2 cuz its easier to work with
        multiplier = 1 - multiplier;                          // Throttle is backwards from expectation, flip it
        if (controller.getPOV() != 1)
        {
            multiplier = multiplier * baseSpeed;              // Mix in some of that sweet default...
        }
        else
        {
            multiplier = multiplier * boostSpeed;             // Unless the trigger is pressed, then mix in some of that sweet boost :)
        }
		
		return multiplier;
	}
	
	/**
	 * Get the raw value of a joystick's throttle
	 * This value is kinda hard to work with, so use getThrottle for a better version
	 * 
	 * @return The value from the throttle which is returned by default
	 */
	public static double getRawThrottle()
	{
		return controller.getTriggerAxis(Hand.kRight);
	}
	
	/**
	 * Get a TRCDriveParams which has all the values from joystick controllerPort for use in driving the robot
	 * 
	 * @param controllerPort What joystick to get values from
	 * @return TRCDriveParams which have been set from values from the joystick controllerPort
	 */
	public static TRCDriveParams getStickDriveParams()
	{
		TRCDriveParams params = new TRCDriveParams();
		
		params.setRawX(controller.getX(Hand.kRight));
		params.setRawY(controller.getY(Hand.kRight));
		params.setRawZ(controller.getX(Hand.kLeft));
		params.setM(getThrottle());
		
		return params;
	}
}
