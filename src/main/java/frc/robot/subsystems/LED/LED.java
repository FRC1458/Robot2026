

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
import frc.robot.subsystems.TelemetryManager;

public class LED extends SubsystemBase {    
    
    private static LED ledInstance;

    public static LED getInstance() {
        if (ledInstance == null) {
            ledInstance = new LED();
        }
        return ledInstance;
    }

    private final AddressableLED led;
    private final AddressableLEDBuffer ledPattern;
    public static int numCycles = 0;

    private LED() {
        super();
        led = new AddressableLED(3);
        ledPattern = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
        TelemetryManager.getInstance().addSendable(this);
        led.setLength(LedConstants.LED_LENGTH);
        led.start();
    }

    @Override
    public void periodic() {
        numCycles++;
        led.setData(ledPattern);
    }

    public void setRGB(int r, int g, int b) {
        for (int i = 0; i < LedConstants.LED_LENGTH; i++) {
            ledPattern.setRGB(i,r,g,b);
        }
    }

    public void rainbowStream() {
        for (int i = 0; i < LedConstants.LED_LENGTH; i++) {
            ledPattern.setRGB(i,
                alternate(numCycles),
                alternate(numCycles+170),
                alternate(numCycles+340));
        }
    }

    //desmos graph:
    //f(x)= {mod(x,510)<=255: mod(x,510),  mod(x,510)>255: 510-mod(x,510)}
    public static int alternate(int num) {
        num %= 510;
        if (num <= 255) {
            return num;
        } else {
            return 510-num;
        }
    }

    public void flow(int[] colors) {
        if (colors.length > 3*LedConstants.LED_LENGTH || colors.length % 3 != 0 || LedConstants.LED_LENGTH % (colors.length/3) != 0) {
            return;
        }
        int nodesPerColor = LedConstants.LED_LENGTH / (colors.length/3);
        int node = numCycles % LedConstants.LED_LENGTH;
        for (int i = 0; i < colors.length; i += 3) {
            for (int j = 1; j <= nodesPerColor; j++) {
                ledPattern.setRGB(node++, colors[i],colors[i+1],colors[i+2]);
                if (node >= LedConstants.LED_LENGTH) {
                    node = 0;
                }
            }
        }
    }

    public void errorRed() {
        setRGB(255,0,0);
    }

}