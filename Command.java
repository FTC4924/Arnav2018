package org.firstinspires.ftc.teamcode;

public abstract class Command {

    public double startPoint;

    public void startCommand(double startPoint) {
        this.startPoint = startPoint;
    }

    public abstract boolean isComplete(double currentPoint);
}
