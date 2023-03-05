package frc.lib.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadAxisButton extends Trigger {
	
	public GamepadAxisButton(BooleanSupplier bs) {
		super(bs);
	}
}
