package org.firstinspires.ftc.teamcode;

public class Move extends Command {

    private static double TOLERANCE = 0.5;
    public double distance;
    private double power;

    public Move(double distance, double power) {
        this.distance = distance;
        this.power = power;

        if (distance < 0) {
            this.power *= -1;
        }
    }

    @Override
    public boolean isComplete(double currentPoint) {
        if (distance < 0) {
            return currentPoint - startPoint < distance + TOLERANCE;
        }
        return currentPoint - startPoint > distance - TOLERANCE;
    }

    public double getPower() {
        return power;
    }
}
