package org.firstinspires.ftc.teamcode;

public class Turn extends Command {

    private final double TOLERANCE = 0.5;
    private double angle;
    private double power;

    public Turn(double angle, double power) {
        this.angle = angle;
        this.power = power;
    }

    @Override
    public boolean isComplete(double currentPoint) {
        double difference = Math.abs(angle - currentPoint);

        while (difference > 180) {
            difference -= 180;
        }

        return Math.abs(difference) < TOLERANCE;
    }
}
