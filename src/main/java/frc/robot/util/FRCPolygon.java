package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

public class FRCPolygon {

  private final Path2D.Double path;
  private final String name;

  public FRCPolygon(String polygonName, Translation2d... translations) {
    path = new Path2D.Double();
    this.name = polygonName;
    if (translations.length > 0) {
      path.moveTo(translations[0].getX(), translations[0].getY());
      for (int i = 1; i < translations.length; i++) {
        path.lineTo(translations[i].getX(), translations[i].getY());
      }
      path.closePath();
    }
  }

  public FRCPolygon(String polygonName, Path2D path) {
    this.path = new Path2D.Double(path);
    this.name = polygonName;
  }

  public FRCPolygon(String polygonName, FRCPolygon frcPolygon) {
    this.path = new Path2D.Double(frcPolygon.path);
    this.name = polygonName;
  }

  public void translate(double x, double y) {
    AffineTransform at = AffineTransform.getTranslateInstance(x, y);
    path.transform(at);
  }

  public void translate(Translation2d translation) {
    translate(translation.getX(), translation.getY());
  }

  public void rotate(double theta) {
    Rectangle2D bounds = path.getBounds2D();
    double centerX = bounds.getCenterX();
    double centerY = bounds.getCenterY();
    AffineTransform at = AffineTransform.getRotateInstance(theta, centerX, centerY);
    path.transform(at);
  }

  public void transform(AffineTransform at) {
    path.transform(at);
  }

  public Rectangle2D getBounds2D() {
    return path.getBounds2D();
  }

  public boolean contains(double x, double y) {
    return path.contains(x, y);
  }

  public boolean contains(Point2D p) {
    return path.contains(p);
  }

  public boolean contains(double x, double y, double w, double h) {
    return path.contains(x, y, w, h);
  }

  public boolean contains(Rectangle2D r) {
    return path.contains(r);
  }

  public boolean intersects(double x, double y, double w, double h) {
    return path.intersects(x, y, w, h);
  }

  public boolean intersects(Rectangle2D r) {
    return path.intersects(r);
  }

  public Path2D.Double getPath() {
    return path;
  }

  public Rectangle2D getBoundingRect() {
    return path.getBounds2D();
  }

  public boolean contains(Translation2d translation) {
    return contains(translation.getX(), translation.getY());
  }

  public String getName() {
    return name;
  }
}
