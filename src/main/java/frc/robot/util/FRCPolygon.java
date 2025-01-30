package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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

  public FRCPolygon(String polygonName, String pathPlannername){
    PathPlannerPath ppath=null;
    try{
       ppath=PathPlannerPath.fromPathFile(pathPlannername);
    }
    catch(IOException e){ System.out.println("Error reading File:"+ e);
  }
  catch (ParseException e){System.out.println("Error Parsing File" + e);}
  
    


    List <Translation2d> translations = new ArrayList<>();
    for( Waypoint w : ppath.getWaypoints()){
      translations.add(w.anchor());

    }
    this.path = new Path2D.Double();
    this.name = polygonName;
    if (translations.size() > 0) {
      this.path.moveTo(translations.get(0).getX(), translations.get(0).getY());
      for (int i = 1; i < translations.size(); i++) {
        this.path.lineTo(translations.get(i).getX(), translations.get(i).getY());
      }
      this.path.closePath();
    }
    



  }

  public FRCPolygon(String polygonName, FRCPolygon frcPolygon) {
    this.path = new Path2D.Double(frcPolygon.path);
    this.name = polygonName;
  }

  public void translate(double x, double y) {
    AffineTransform at = AffineTransform.getTranslateInstance(x, y);
    path.transform(at);
  }

  public void flipSide(boolean blueside){

    if (!blueside){
      AffineTransform at =AffineTransform.getQuadrantRotateInstance(2, 8.756,4);
      path.transform(at);
      

    }
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

  public void scale(double scaleFactor) {
    AffineTransform at = AffineTransform.getScaleInstance(scaleFactor, scaleFactor);
    path.transform(at);
  }

  public void scale(double xScaleFactor, double yScaleFactor) {
    AffineTransform at = AffineTransform.getScaleInstance(xScaleFactor, yScaleFactor);
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
