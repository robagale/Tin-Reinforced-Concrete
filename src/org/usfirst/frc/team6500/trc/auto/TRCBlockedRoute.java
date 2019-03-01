package org.usfirst.frc.team6500.trc.auto;

import java.util.HashMap;

/**
 * Exists to ensure compatibility with any auto route which may want to be run.
 * Beware that changing this will force a change in ALL other existing auto
 * routes.
 */
public interface TRCBlockedRoute
{
    public HashMap<String, TRCVector> actions = new HashMap<String, TRCVector>();
    
    /**
     * The main method to run in autonomous
     */
    public void run();
}