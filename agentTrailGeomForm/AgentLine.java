package agentTrailGeomForm;

import toxi.geom.Vec3D;

// a line connecting two agents' two trail points
public class AgentLine {
	// the two agents
	Agent a, b;
	// their two trail points
	Vec3D pta, ptb;
	// the indices of the two points in their trail
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
}
