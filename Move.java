package org.firstinspires.ftc.teamcode;

public class Move extends Command {

    private static double TOLERANCE = 0.5;
    private double distance;
    private double power;

    public Move(double distance, double power) {
        this.distance = distance;
        this.power = power;
    }

    @Override
    public boolean isComplete(double currentPoint) {
        return Math.abs(distance - currentPoint) < TOLERANCE;
    }
}
