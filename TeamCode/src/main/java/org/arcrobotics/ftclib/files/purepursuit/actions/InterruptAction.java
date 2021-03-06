package org.arcrobotics.ftclib.files.purepursuit.actions;

import org.arcrobotics.ftclib.files.purepursuit.waypoints.InterruptWaypoint;

/**
 * This interface represents an action that InterruptWaypoint perform when
 * they reach their interrupt point.
 * 
 * @see InterruptWaypoint
 * @version 1.0
 * @author Michael Baljet, Team 14470
 *
 */
public interface InterruptAction {
	
	/**
	 * Performs the action.
	 */
	public void doAction();
	
}
