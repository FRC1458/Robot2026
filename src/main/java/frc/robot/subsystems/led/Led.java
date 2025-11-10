package frc.robot.subsystems.led;

import java.util.Arrays;
import java.util.function.BiFunction;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {    
    private static Led ledInstance;
    public static Led getInstance() {
        if (ledInstance == null) {
            ledInstance = new Led();
        }
        return ledInstance;
    }

    private static class Colorer {
        private static enum State {
            SOLID,
            PATTERN,
            TIMED_PATTERN;
        }

        private State state = State.SOLID;
        private Color solidColor = Color.kGreen;
        private Color[] pattern;
        private BiFunction<Integer, Double, Color> timedPatternSupplier;
        private Color[] colors = new Color[LedConstants.ledLength];
        private boolean hasUpdated = true;
        
        public void setSolidColor(Color color) {
            solidColor = color;
            state = State.SOLID;
            hasUpdated = true;
        }

        public void setPattern(Color[] colors) {
            pattern = colors;
            state = State.PATTERN;
            hasUpdated = true;
        }

        public void setTimedPattern(BiFunction<Integer, Double, Color> timedPatternSupplier) {
            this.timedPatternSupplier = timedPatternSupplier;
            state = State.TIMED_PATTERN;
            hasUpdated = true;
        }

        public Color[] get(double time) {
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

    private final Notifier ledNotifier;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer timer;
    private final Colorer colorer;

    public Led() {
        ledNotifier = new Notifier(this::update);
        timer = new Timer();
        led = new AddressableLED(3);
        ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
        led.setLength(ledBuffer.getLength());
        led.start();
        timer.start();
        colorer = new Colorer();
        ledNotifier.startPeriodic(0.125);
        setDefaultCommand(setRainbowCommand());
        SmartDashboard.putData(this);
    }

    private void update() {
        if (colorer.state != Colorer.State.TIMED_PATTERN || colorer.hasUpdated) {
            var colors = colorer.get(timer.get());
            for(int i = LedConstants.ledStart; i < LedConstants.ledLength; i++) {
                ledBuffer.setLED(i, colors[i]);
            }
            led.setData(ledBuffer);
        }
    }

    public Command setSolidColorCommand(Color color) {
        return runOnce(() -> colorer.setSolidColor(color)).withName(
            "Solid Color " + color.toString());
    }

    public Command setRainbowCommand() {
        return runOnce(() -> colorer.setTimedPattern(
            (Integer index, Double time) -> 
                Color.fromHSV(index + (int) (time * 50), 255, 255)
        )).withName("Rainbow");
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}