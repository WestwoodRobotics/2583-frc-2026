package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GetHubStatus {

    public static boolean didWin = true;
    public static boolean isPractice = false;
    public static double countdown;

    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // If we're in practice mode, always enable the hub.
        if (isPractice) {
            return true;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            countdown = matchTime - 130;
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            countdown = matchTime - 105;
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            countdown = matchTime - 80;
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            countdown = matchTime - 55;
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            countdown = matchTime - 30;
            return !shift1Active;
        } else {
            countdown = matchTime;
            // End game, hub always active.
            return true;
        }
    }

    public static double getHubCountdown() {
        return countdown;
    }
    public static double getEndgameCountdown() {
        return DriverStation.getMatchTime() - 30;
    }

    public static void changeWhoWon(boolean won) {
        didWin = won;
    }

    public static void togglePracticeMode() {
        isPractice = !isPractice;
    }

    public static boolean isPractice() {
        return isPractice;
    }
}
