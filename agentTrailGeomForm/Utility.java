package agentTrailGeomForm;

import toxi.geom.Vec3D;

public class Utility {
	public static float paraboloid(float x, float y, float a, float b, float c) {
		float z = c * (y * y / (b * b) - x * x / (a * a));
		return z;
	}
	
	public static Vec3D getNormalPoint(Vec3D p, Vec3D a, Vec3D b) {
		Vec3D ap = p.sub(a);
		Vec3D ab = b.sub(a);
		ab.normalize();
		ab.scaleSelf(ap.dot(ab));
		return a.add(ab);
	}
	
	public static int[] coorToIndex(Vec3D pos, int DIMX, int DIMY, int DIMZ, int ratio) {
		int[] result = new int[3];
		Vec3D rel = pos.add(new Vec3D(DIMX / 2, DIMY / 2, DIMZ / 2));
		result[0] = (int) Math.floor(rel.x()) / ratio;
		result[1] = (int) Math.floor(rel.y()) / ratio;
		result[2] = (int) Math.floor(rel.z()) / ratio;
		return result;
	}
}
