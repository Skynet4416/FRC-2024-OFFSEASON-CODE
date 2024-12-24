package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.DoubleSubscriber;

public class ShooterHandler {
    public static Pair<Double, Double> calcShooterAngleAndRPM(double distance) {
        return new Pair<Double, Double>(distance * 1, distance * 1); // rpm, angle;
    }
}
