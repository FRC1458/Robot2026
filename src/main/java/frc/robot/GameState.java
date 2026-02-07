package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
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
        if (!DriverStation.isTeleop()) {
            return BOTH;
        }
        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            return BOTH;
        }
        if (matchTime > 135) {
            return BOTH;
        }
        if (matchTime <= 30) {
            return BOTH;
        }
        double teleopElapsed = 135 - matchTime;
        if (teleopElapsed < 10) {
            return BOTH;
        }
        int shift = (int)((teleopElapsed - 10) / 25.0);
        if (shift < 0 || shift > 3) {
            return BOTH;
        }
        if (shift % 2 == 0) {
            return wonAuto.getOppositeState();
        } else {
            return wonAuto;
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
