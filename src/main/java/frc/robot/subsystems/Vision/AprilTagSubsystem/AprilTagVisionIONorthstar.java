// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Vision.AprilTagSubsystem;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int cameraId = 0;
  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 5;
  private static final int cameraGain = 25;

  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  private static final double disconnectedTimeout = 1.5;
  private final Timer disconnectedTimer = new Timer();

  public AprilTagVisionIONorthstar(String identifier) {
    var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);
    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
      inputs.frames[i] = queue[i].value;
    }
    inputs.fps = fpsSubscriber.get();

  }
}
