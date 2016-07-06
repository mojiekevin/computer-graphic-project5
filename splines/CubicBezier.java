package cs4620.splines;

import java.util.ArrayList;
import egl.math.Vector2;
/*
 * Cubic Bezier class for the splines assignment
 */

public class CubicBezier {
	
	//This Bezier's control points
	public Vector2 p0, p1, p2, p3;
	
	//Control parameter for curve smoothness
	float epsilon;
	
	//Parameter for computing points
	float u = 0.5f;
	
	//The points on the curve represented by this Bezier
	private ArrayList<Vector2> curvePoints;
	
	//The normals associated with curvePoints
	private ArrayList<Vector2> curveNormals;
	
	//The tangent vectors of this bezier
	private ArrayList<Vector2> curveTangents;
	
	
	/**
	 * 
	 * Cubic Bezier Constructor
	 * 
	 * Given 2-D BSpline Control Points correctly set self.{p0, p1, p2, p3},
	 * self.uVals, self.curvePoints, and self.curveNormals
	 * 
	 * @param bs0 First Bezier Spline Control Point
	 * @param bs1 Second Bezier Spline Control Point
	 * @param bs2 Third Bezier Spline Control Point
	 * @param bs3 Fourth Bezier Spline Control Point
	 * @param eps Maximum angle between line segments
	 */
	public CubicBezier(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float eps) {
		curvePoints = new ArrayList<Vector2>();
		curveTangents = new ArrayList<Vector2>();
		curveNormals = new ArrayList<Vector2>();
		epsilon = eps;
		int maxrecur = 10;
		int recursion = 0;
		
		this.p0 = new Vector2(p0);
		this.p1 = new Vector2(p1);
		this.p2 = new Vector2(p2);
		this.p3 = new Vector2(p3);
		
		tessellate(p0, p1, p2, p3, epsilon, maxrecur, recursion);
	}
	

    /**
     * Approximate a Bezier segment with a number of vertices, according to an appropriate
     * smoothness criterion for how many are needed.  The points on the curve are written into the
     * array self.curvePoints, the tangents into self.curveTangents, and the normals into self.curveNormals.
     * The final point, p3, is not included, because cubic Beziers will be "strung together".
     */
    private void tessellate(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float eps, int maxrecur, int recursion) {
    	 // TODO A5
    	//SOLUTION
    	recursion++;
    	Vector2 v1 = p1.clone().sub(p0);
    	Vector2 v2 = p2.clone().sub(p1);
    	Vector2 v3 = p3.clone().sub(p2);
    	Vector2 tan2normal1 = new Vector2(0, 1);
    	Vector2 tan2normal2 = new Vector2(-1, 0);
    	float theta1 = v1.angle(v2);
		float theta2 = v2.angle(v3);
    	if((theta1 < 0.5 * eps && theta2 < 0.5 * eps) || recursion >= maxrecur){
    		//use the coordinate of nearby point(p1) on the curve to minus the coordinate of p0 as the tangent vector 
    		Vector2 tangent = p1.clone().sub(p0).normalize();
    		curvePoints.add(p0);
    		curveTangents.add(tangent);
    		//rotate the tangent vector 90 degree(clockwise) as the normal vector
    		curveNormals.add(new Vector2(tan2normal1.dot(tangent), tan2normal2.dot(tangent)).normalize());
    	}
    	else{
    		Vector2 p10 = p0.clone().mul(1 - u).add(p1.clone().mul(u));
    		Vector2 p11 = p1.clone().mul(1 - u).add(p2.clone().mul(u));
    		Vector2 p12 = p2.clone().mul(1 - u).add(p3.clone().mul(u));
    		Vector2 p20 = p10.clone().mul(1 - u).add(p11.clone().mul(u));
    		Vector2 p21 = p11.clone().mul(1 - u).add(p12.clone().mul(u));
    		Vector2 p30 = p20.clone().mul(1 - u).add(p21.clone().mul(u));
    		tessellate(p0, p10, p20, p30, eps, maxrecur, recursion);
    		tessellate(p30, p21, p12, p3, eps, maxrecur, recursion);
    	}
    
    	//END SOLUTION
    }
	
    
    /**
     * @return The points on this cubic bezier
     */
    public ArrayList<Vector2> getPoints() {
    	ArrayList<Vector2> returnList = new ArrayList<Vector2>();
    	for(Vector2 p : curvePoints) returnList.add(p.clone());
    	return returnList;
    }
    
    /**
     * @return The tangents on this cubic bezier
     */
    public ArrayList<Vector2> getTangents() {
    	ArrayList<Vector2> returnList = new ArrayList<Vector2>();
    	for(Vector2 p : curveTangents) returnList.add(p.clone());
    	return returnList;
    }
    
    /**
     * @return The normals on this cubic bezier
     */
    public ArrayList<Vector2> getNormals() {
    	ArrayList<Vector2> returnList = new ArrayList<Vector2>();
    	for(Vector2 p : curveNormals) returnList.add(p.clone());
    	return returnList;
    }
    
    
    /**
     * @return The references to points on this cubic bezier
     */
    public ArrayList<Vector2> getPointReferences() {
    	ArrayList<Vector2> returnList = new ArrayList<Vector2>();
    	for(Vector2 p : curvePoints) returnList.add(p);
    	return returnList;
    }
    
    /**
     * @return The references to tangents on this cubic bezier
     */
    public ArrayList<Vector2> getTangentReferences() {
    	ArrayList<Vector2> returnList = new ArrayList<Vector2>();
    	for(Vector2 p : curveTangents) returnList.add(p);
    	return returnList;
    }
    
    /**
     * @return The references to normals on this cubic bezier
     */
    public ArrayList<Vector2> getNormalReferences() {
    	ArrayList<Vector2> returnList = new ArrayList<Vector2>();
    	for(Vector2 p : curveNormals) returnList.add(p);
    	return returnList;
    }
}
