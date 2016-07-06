package cs4620.splines;
import java.util.ArrayList;

import egl.math.Matrix4;
import egl.math.Vector2;
import egl.math.Vector4;

public class BSpline extends SplineCurve{

	public BSpline(ArrayList<Vector2> controlPoints, boolean isClosed,
			float epsilon) throws IllegalArgumentException {
		super(controlPoints, isClosed, epsilon);
	}

	@Override
	public CubicBezier toBezier(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3,
			float eps) {
		//TODO A5 (Extra Credit)
		//SOLUTION
		Matrix4 Mbez = new Matrix4(-1, 3, -3, 1,
									3, -6, 3, 0,
									-3, 3, 0, 0,
									1, 0, 0, 0);
		Matrix4 Mcr = new Matrix4(-1/6f, 0.5f, -0.5f, 1/6f,
								   0.5f, -1f, 0.5f, 0,
								   -0.5f, 0, 0.5f, 0,
								   1/6f, 2/3f, 1/6f, 0);
		Matrix4 M = Mbez.clone().invert().mulBefore(Mcr);
		Vector4 px = new Vector4(p0.x, p1.x, p2.x, p3.x);
		Vector4 py = new Vector4(p0.y, p1.y, p2.y, p3.y);
		M.mul(px);
		M.mul(py);

		Vector2 a = new Vector2(px.x, py.x);
		Vector2 b = new Vector2(px.y, py.y);
		Vector2 c = new Vector2(px.z, py.z);
		Vector2 d = new Vector2(px.w, py.w);
		
		return new CubicBezier(a, b, c, d, eps);
		//END SOLUTION
	}
	

		
	
}
