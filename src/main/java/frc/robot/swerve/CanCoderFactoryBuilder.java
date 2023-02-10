package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import static frc.robot.Constants.*;

import java.io.Console;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    //TODO: are we polling too much?
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = direction == Direction.CLOCKWISE;
            //TODO: is the sensor oriented in a clockwise direction?

            CANCoder encoder = new CANCoder(configuration.getId(), CANIVORE_BUS_NAME);
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANCoder encoder;
        

        private EncoderImplementation(CANCoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            //TODO: log here!
            
            
            double angle = Math.toRadians(encoder.getAbsolutePosition());
            System.out.println("Absolute position of the cancoders as reported by the phoenix libraries" + Math.toDegrees(angle));
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }
            System.out.println("Absolute angle as calculated: " + Math.toDegrees(angle));
            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
