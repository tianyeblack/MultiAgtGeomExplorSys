package agentTrailGeomForm;

import javafx.util.Pair;
import toxi.geom.Vec3D;

public class AgentLine {
	Agent a, b;
	Vec3D pta, ptb;
	int indexa, indexb;
	
	AgentLine(Agent _a, Agent _b, Vec3D _pta, Vec3D _ptb, int _indexa, int _indexb) {
		a = _a;
		b = _b;
		pta = new Vec3D(_pta.x(), _pta.y(), _pta.z());
		ptb = new Vec3D(_ptb.x(), _ptb.y(), _pta.z());
		indexa = _indexa;
		indexb = _indexb;
	}
	
	public Vec3D getPta() {
		return new Vec3D(pta.x(), pta.y(), pta.z());
	}
	
	public Vec3D getPtb() {
		return new Vec3D(ptb.x(), ptb.y(), ptb.z());
	}
	
	public Pair<Vec3D, Vec3D> getPts() {
		return new Pair<Vec3D, Vec3D>(pta.copy(), ptb.copy());
	}
}
