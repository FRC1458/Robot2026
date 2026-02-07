package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum GameState {
    RED,
    BLUE,
    BOTH;
    
    public static volatile GameState wonAuto = BOTH;

    public static Thread startListenerThread() {
        Thread listenerThread = new Thread(
            () -> {
                while (wonAuto == BOTH) {
                    try {
                        update();
                        Thread.sleep(1000);
                    } catch (Exception e) {
                        DriverStation.reportWarning("Sleep failed, how", false);
                    }
                }
            }, "Game message listener");
        listenerThread.setPriority(2);
        listenerThread.start();
        return listenerThread;
    }

    public static void update() {
        var message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            switch (message.charAt(0)) {
                case 'B' :
                    wonAuto = BLUE;
                    break;
                case 'R' :
                    wonAuto = RED;
                    break;
                default :
                    wonAuto = BOTH;
                    break;
            }
        } else {
            wonAuto = BOTH;
        }
    }

    private GameState getOppositeState() {
        switch (this) {
            case RED: return BLUE;
            case BLUE: return RED;
            default: return BOTH;
        }
    }

    public static GameState getCurrentState() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            return BOTH;
        }
        double elapsed = 150 - matchTime;
        if (elapsed < 30) {
            return BOTH;
        }
        int cycle = (int) ((elapsed - 30) / 25.0);
        if (cycle % 2 == 0) {
            return wonAuto;
        } else {
            return wonAuto.getOppositeState();
        }
    }

    public static boolean isAllianceHubActive() {
        var state = getCurrentState();
        if (state == BOTH) {
            return true;
        } 
        
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent()) {
            if (state == RED) {
                return currentAlliance.get() == Alliance.Red;
            } else {
                return currentAlliance.get() == Alliance.Blue;
            }
        } else {
            return true;
        }
    }
}
