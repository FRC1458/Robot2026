package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.subsystem.LoggedSubsystem;

public class Indexer extends LoggedSubsystem {
	public LeftIndexer lIndex;
	public RightIndexer rIndex;
	public Roller roller;

	public Indexer(LeftIndexer lIndex, RightIndexer rIndex, Roller roller) {
		super();
		this.lIndex = lIndex;
		this.rIndex = rIndex;
		this.roller = roller;

		setDefaultCommand(stopAll());
	}

	public Command indexAll() {
		return Commands.parallel(lIndex.index(), rIndex.index(), roller.index()).andThen(idle());
	}

	public Command stopAll() {
		return Commands.parallel(lIndex.stop(), rIndex.stop(), roller.stop()).andThen(idle());
	}
}
