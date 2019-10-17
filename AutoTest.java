package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;

@Autonomous(name="AutoTest")
public class AutoTest extends AutonomousBase {
    public AutoTest() {
        commands = Arrays.asList(
                new Move(5000.0, 0.2),
                new Turn(90.0, 0.8),
                new Turn(-90.0, 0.8),
                new Move(-5000.0, 0.2)
        );
    }
}
