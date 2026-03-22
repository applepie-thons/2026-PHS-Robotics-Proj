// FRC Team 4034

package frc.robot;

abstract public class CommandBase {
	/**
	 * Override this function to prepare any state that needs be set just as the command
	 * is about to run. For example, with a command for driving to a particualar distance,
	 * this function should snapshot the current (starting) position of the robot so that
	 * we can evaluate how far we've moved from that start position in commandPeriodic().
	 */
	public void commandInit() {}

	/**
	 * Will be called by autonomousPeriod() to actually execute this command.
	 *
	 * Returns `true` when finished, `false` when it's not finished. A return value of
	 * `true` indicates that autonomousPeriod() can move onto the next command.
	 */
	abstract public boolean commandPeriodic();
}
