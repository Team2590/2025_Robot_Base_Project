package frc.robot;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;

public class FieldConstants {
  public static enum ZONES {
    REEF,
    BARGE,
    PROCCESOR,
    NOTE
  }

  public static double[] Reef_x = {2.16, 6.61, 2.16, 6.61};
  public static double[] Reef_y = {6.08, 6.08, 1.57, 1.56};
  public static Path2D ReefBounds = create_zone(Reef_x, Reef_y);
  // public static FRCPolygon ReefPolygon= new FRCPolygon("REEF", ReefBounds,false);

  public static double[] Barge_x = {10.23, 10.23, 7.367, 7.367};

  public static Path2D BargeBoundsBLUE = create_zone(Reef_x, Reef_y); // bottom
  public static Path2D BargeBoundsRED =
      create_zone(Reef_x, new double[] {3.5, 4.16, 3.5, .416}); // top

  public static Path2D create_zone(double[] x, double[] y) {

    Path2D bounds = new Path2D.Double();
    bounds.moveTo(x[0], y[0]);

    for (int i = 1; i < x.length; ++i) {
      bounds.lineTo(x[i], y[i]);
    }

    return bounds;
  }

  public static Path2D flipZone(Path2D zone) {
    AffineTransform t = new AffineTransform();
    t = AffineTransform.getQuadrantRotateInstance(2, 8.785, 4.02);
    Shape flipped = zone.createTransformedShape(t);

    return new Path2D.Double(flipped);
  }
}

// porcesor barge feeding station
