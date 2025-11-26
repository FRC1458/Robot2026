package frc.robot.subsystems.led;

import java.util.Arrays;
import java.util.function.BiFunction;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Led extends SubsystemBase {    
    private static Led ledInstance;
    public static Led getInstance() {
        if (ledInstance == null) {
            ledInstance = new Led();
        }
        return ledInstance;
    }

    /** A helper to optimize LEDs state */
    private static class Colorer {
        /** The state of the LEDs */
        private static enum State {
            SOLID,
            PATTERN,
            TIMED_PATTERN;
        }

        private final Object lock = new Object();
        private State state = State.SOLID;
        private Color solidColor = Color.kGreen;
        private Color[] pattern;
        private BiFunction<Integer, Double, Color> timedPatternSupplier;
        private Color[] colors = new Color[LedConstants.LED_LENGTH];
        private boolean hasUpdated = true;
        
        public void setSolidColor(Color color) {
            synchronized (lock) {
                hasUpdated = true;
                solidColor = color;
                state = State.SOLID;
            }
        }

        public void setPattern(Color[] colors) {
            synchronized (lock) {
                hasUpdated = true;
                pattern = Arrays.stream(colors)
                    .map((Color c) -> new Color(c.red, c.green, c.blue))
                    .toArray(Color[]::new);
                state = State.PATTERN;
            }
        }

        public void setTimedPattern(BiFunction<Integer, Double, Color> timedPatternSupplier) {
            synchronized (lock) {
                hasUpdated = true;
                this.timedPatternSupplier = timedPatternSupplier;
                state = State.TIMED_PATTERN;
            }
        }

        /** Gets the colors based on the state */
        public Color[] get(double time) {
            synchronized (lock) {
                switch (state) {
                    case SOLID: 
                        if (hasUpdated) {
                            Arrays.fill(colors, solidColor);
                        }
                        break;
                    case PATTERN:
                        if (hasUpdated) {
                            colors = pattern;
                        }
                        break;
                    case TIMED_PATTERN:
                        for (int i = 0; i < colors.length; i++) {
                            colors[i] = timedPatternSupplier.apply(i, time);
                        }
                        break;
                    default:
                        break;
                };
                hasUpdated = false;
                return colors;
            }
        }
    }

    private final Notifier ledNotifier;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer timer;
    private final Colorer colorer;

    public Led() {
        ledNotifier = new Notifier(this::update);
        timer = new Timer();
        led = new AddressableLED(LedConstants.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
        led.setLength(ledBuffer.getLength());
        led.start();
        timer.start();
        colorer = new Colorer();
        if (Robot.isSimulation()) {
            new AddressableLEDSim(led);
        }
        ledNotifier.startPeriodic(LedConstants.UPDATE_DT);
        SmartDashboard.putData(this);
    }

    /** Updates the LEDs */
    private void update() {
        if (colorer.state != Colorer.State.TIMED_PATTERN || colorer.hasUpdated) {
            var colors = colorer.get(timer.get());
            for(int i = LedConstants.LED_START; i < LedConstants.LED_LENGTH; i++) {
                ledBuffer.setLED(i, colors[i]);
            }
            led.setData(ledBuffer);
        }
    }    
    
    /** Sets the LEDs to a color indefinitely */
    public Command setSolidColorCommand(Color color) {
        return setSolidColorCommand(color, Double.POSITIVE_INFINITY);
    }

    /** Sets the LEDs to a color for a set time */
    public Command setSolidColorCommand(Color color, double holdTime) {
        return runOnce(() -> colorer.setSolidColor(color))
            .andThen(
                Commands.waitSeconds(holdTime)
            ).ignoringDisable(true)
            .withName(color.toString() + ": Solid Color");
    }

    /** Animates the LEDs with a rainbow animation */
    public Command setRainbowCommand() {
        return setRainbowCommand(Double.POSITIVE_INFINITY);
    }

    /** Sets the LEDs to an animated rainbow for a set time */
    public Command setRainbowCommand(double holdTime) {
        return runOnce(() -> colorer.setTimedPattern(
            (Integer index, Double time) -> 
                Color.fromHSV(index + (int) (time * 50), 255, 255)
        )).andThen(
            Commands.waitSeconds(holdTime)
        ).ignoringDisable(true).withName("Rainbow");
    }

    /** Holds the LEDs at a random state */
    public Command setRandomCommand() {
        return setRandomCommand(Double.POSITIVE_INFINITY);
    }

    /** Sets the LEDs to a random state for a set time */
    public Command setRandomCommand(double holdTime) {
        return defer(() -> {
            Color[] colors = new Color[LedConstants.LED_LENGTH];
            for (int i = 0; i < colors.length; i++) {
                colors[i] = new Color(Math.random(), Math.random(), Math.random());
            }
            return runOnce(
                () -> colorer.setPattern(
                    colors));
            }
        ).andThen(
            Commands.waitSeconds(holdTime)
        ).ignoringDisable(true).withName("Random");
    }

    /** Blinks the LEDs indefinitely */
    public Command blinkCommand(Color color1, Color color2, double delta) {
        return blinkCommand(color1, color2, delta, Double.POSITIVE_INFINITY);
    }

    /** Blinks the LEDs for a set time */
    public Command blinkCommand(Color color1, Color color2, double delta, double holdTime) {
        return Commands.repeatingSequence(
            setSolidColorCommand(color1, delta),
            setSolidColorCommand(color2, delta)
        ).raceWith(
            Commands.waitSeconds(holdTime)
        ).withName(color1.toString() + ":" + color2.toString() + ":" +  (int) (delta * 1000) + " ms Blink");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}