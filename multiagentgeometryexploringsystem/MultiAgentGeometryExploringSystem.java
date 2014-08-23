package multiagentgeometryexploringsystem;

import java.util.ArrayList;

import agentTrailGeomForm.Agent;
import agentTrailGeomForm.AgentLine;
import agentTrailGeomForm.AgentsTrail;
import controlP5.ControlP5;
import peasy.PeasyCam;
import processing.core.PApplet;
import toxi.geom.AABB;
import toxi.geom.Vec3D;
import toxi.geom.mesh.STLReader;
import toxi.geom.mesh.TriangleMesh;
import toxi.processing.ToxiclibsSupport;
import toxi.volume.ArrayIsoSurface;
import toxi.volume.IsoSurface;
import toxi.volume.MeshVoxelizer;
import toxi.volume.RoundBrush;
import toxi.volume.VolumetricBrush;


@SuppressWarnings("serial")
public class MultiAgentGeometryExploringSystem extends PApplet {

	PeasyCam cam;
	VolumetricBrush brushA;
	VolumetricBrush brushB;
	VolumetricBrush brushC;
	IsoSurface surfaceA;
	IsoSurface surfaceB;
	IsoSurface surfaceC;
	IsoSurface surfaceS;
	TriangleMesh meshA = new TriangleMesh("meshA");
	TriangleMesh meshB = new TriangleMesh("meshB");
	TriangleMesh meshC = new TriangleMesh("meshC");
	TriangleMesh meshS;
	MeshVoxelizer voxelizer;
	ToxiclibsSupport gfx;
	ControlP5 ui;
	int movieCounter ;
	boolean videoRecord = false;
	int counter = 0;
	boolean runToggle = false;
	boolean capture = false;
	boolean record = false;
	boolean strok = false;
	boolean compute = false;
	boolean trail = false;
	boolean runAttraction = false;

	//Box size
	int bX = 0, bY = 0, bZ = 0;
	//Affects the mesh
	float ISO = 0.5f;

	int ratio = 1;
	float isoBrushSize = ratio;
	float isoBrushDensity = 1f;

	AgentsTrail agt;

	public void setup() {
		size(1200, 800, OPENGL);
		smooth();
		cam = new PeasyCam(this, 600);
		agt = AgentsTrail.getInstance();
		getStructure("src/data/TOPY_MESH_POINTS_01.stl");
		getStructurePt("src/data/TOPY_MESH_POINTS_01.txt");
		float[] fs = new float[agt.loads.size()];
		Vec3D[] vs = new Vec3D[agt.loads.size()];
		agt.addAgents(agt.loads.size(), fs, agt.loads.toArray(vs));
		AABB bBox = meshS.getBoundingBox();
		Vec3D center = bBox.getMax().add(bBox.getMin());
		center = new Vec3D().sub(new Vec3D(center.x / 2, center.y / 2, center.z / 2));
		meshS.translate(center);

//		getGeometry("src/data/catenary_mesh_relaxed_03.txt");
//		getBoundary("src/data/catenary_mesh_relaxed_04_outlines.txt");
//		get2DPlane("src/data/catenary_mesh_relaxed_04_flatsrf.txt");
		agt.setDIMAndGRID(bX, bY, bZ, ratio);
		voxelizer = new MeshVoxelizer(agt.GRIDX, agt.GRIDY, agt.GRIDZ);
		agt.volumeS = voxelizer.voxelizeMesh(meshS);
		
		agt.assignAgentsType(1);
		//		agt.assignAgentsType(2);
		surfaceA = new ArrayIsoSurface(agt.volumeA);
		surfaceB = new ArrayIsoSurface(agt.volumeB);
		surfaceC = new ArrayIsoSurface(agt.volumeC);
		brushA = new RoundBrush(agt.volumeA, isoBrushSize);
		brushB = new RoundBrush(agt.volumeB, isoBrushSize);
		brushC = new RoundBrush(agt.volumeC, isoBrushSize);

		gfx = new ToxiclibsSupport(this);
		ui = new ControlP5(this);
		ui.setAutoDraw(false);

		ui.addSlider("ISO",0,1,ISO,20,20,300,14);
	}

	@SuppressWarnings("deprecation")
	public void draw() {
		if (record) beginRaw(PDF, "output/msa_catenary_Srf_trails"+ frameCount+".pdf") ;
		background(255);
		lights();
		drawAxes();

		displayPoints(agt.loads, new Vec3D(0, 0, 0), 4);
		displayPoints(agt.supports, new Vec3D(0, 0, 0), 4);
		agt.runAgents(frameCount);
		if (trail == true) {
			displayTrails();
//			displayConnections();
		} else {
			displayLocs();
		}
		displayStarts();
//		displayOutline();

		// A bounding box for a better view
		displayBox();
		
		if (frameCount % 5 == 0 && compute) {
			drawTrails();
		}

		displayMeshes(strok);

		if (frameCount == 1 || (frameCount % 20 == 0 && frameCount < 1000)) {
			agt.exportText(frameCount);
		}
		if (capture) saveFrame("MSA_agent_cat_Trails" + frameCount + ".png");
		if (record) {
			endRaw();
			record = false;
		}

		recording();
		if (ui.window(this).isMouseOver()) cam.setActive(false);
		else cam.setActive(true);
		gui();
	}

	void drawAxes() {
		stroke(0);
		fill(0);
		strokeWeight(3);
		point(0, 0, 0);
		strokeWeight(1);
		stroke(255, 0, 0);
		line(0, 0, 0, 500, 0, 0);
		stroke(0, 255, 0);
		line(0, 0, 0, 0, 500, 0);
		stroke(0, 0, 255);
		line(0, 0, 0, 0, 0, 500);
	}
	
	void gui() {
		hint(DISABLE_DEPTH_TEST);
		cam.beginHUD();
		ui.draw();
		cam.endHUD();
		hint(ENABLE_DEPTH_TEST);
	}

	void getStructurePt(String filename) {
		float x_min, y_min, x_max, y_max, z_min, z_max;
		z_min = y_min = x_min = Float.MAX_VALUE;
		z_max = y_max = x_max = Float.MIN_VALUE;
		String[] lines = loadStrings(filename);
		println("There are " + lines.length + " lines in the mesh point cloud file...");
		Vec3D[] strc = new Vec3D[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] coordinates = split(lines[i].substring(1, lines[i].length() - 1), ", ");
			float x_cur = Float.parseFloat(coordinates[0]);
			float y_cur = Float.parseFloat(coordinates[1]);
			float z_cur = Float.parseFloat(coordinates[2]);
			if (x_cur < x_min) x_min = x_cur;
			else if (x_cur > x_max) x_max = x_cur;
			if (y_cur < y_min) y_min = y_cur;
			else if (y_cur > y_max) y_max = y_cur;
			if (z_cur < z_min) z_min = z_cur;
			else if (z_cur > z_max) z_max = z_cur;
			strc[i] = new Vec3D(x_cur, y_cur, z_cur);
		}
		Vec3D trans = new Vec3D((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);
		int bx = (int) (Math.ceil((x_max - x_min) / ratio) * ratio);
		int by = (int) (Math.ceil((y_max - y_min) / ratio) * ratio);
		int bz = (int) (Math.ceil((z_max - z_min) / ratio) * ratio);
		if (bx > bX) bX = bx;
		if (by > bY) bY = by;
		if (bz > bZ) bZ = bz;
		for (Vec3D v : strc) {
			if (v.z() >= Math.floor(z_max) - 1) agt.loads.add(v.copy());
			else if (v.z() <= Math.ceil(z_min) + 1) agt.supports.add(v.copy());
		}
		for (Vec3D v : agt.loads) v.subSelf(trans);
		for (Vec3D v : agt.supports) v.subSelf(trans);
		System.out.printf("%d, %d\n", agt.loads.size(), agt.supports.size());
		System.out.printf("%f, %f\n", z_max, z_min);
	}
	
	void getStructure(String filename) {
		STLReader reader = new STLReader();
		meshS = (TriangleMesh) reader.loadBinary(filename, STLReader.TRIANGLEMESH);
		System.out.println("1: " + meshS.toString());
	}

	void get2DPlane(String filename) {
		float x_min, y_min, x_max, y_max;
		y_min = x_min = Float.MAX_VALUE;
		y_max = x_max = Float.MIN_VALUE;
		String[] lines=loadStrings (filename);
		agt.twod_plane = new Vec3D[lines.length];
		agt.twod_scores = new float[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] coordinates = split(lines[i].substring(1, lines[i].length() - 1), ", ");
			float x_cur = Float.parseFloat(coordinates[0]);
			float y_cur = Float.parseFloat(coordinates[1]);
			if (x_cur < x_min) x_min = x_cur;
			else if (x_cur > x_max) x_max = x_cur;
			if (y_cur < y_min) y_min = y_cur;
			else if (y_cur > y_max) y_max = y_cur;
			agt.twod_plane[i]= new Vec3D (x_cur, y_cur, 0f);
			agt.twod_scores[i] = 0f;
		}
		Vec3D trans = new Vec3D((x_max + x_min) / 2, (y_max + y_min) / 2, 0);
		int bx = (int) (Math.ceil((x_max - x_min) / ratio) * ratio);
		int by = (int) (Math.ceil((y_max - y_min) / ratio) * ratio);
		int bz = (int) (Math.ceil((agt.DIMZ) / ratio) * ratio);
		if (bx > bX) bX = bx;
		if (by > bY) bY = by;
		if (bz > bY) bZ = bz;
		for (Vec3D v : agt.twod_plane) v.subSelf(trans);
		agt.addAgents(lines.length, agt.twod_scores, agt.twod_plane);
	}

	void getGeometry(String filename) {
		float x_min, y_min, z_min, x_max, y_max, z_max;
		z_min = y_min = x_min = Float.MAX_VALUE;
		z_max = y_max = x_max = Float.MIN_VALUE;
		String[] lines = loadStrings(filename);
		println("There are " + lines.length + " lines in the elevation point file..");
		agt.geo_scores = new float[lines.length];
		agt.geo_starts = new Vec3D[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] parts = split(lines[i], "}");
			String[] coordinates = split(parts[0].substring(1), ", ");
			float x_cur = Float.parseFloat(coordinates[0]);
			float y_cur = Float.parseFloat(coordinates[1]);
			float z_cur = Float.parseFloat(coordinates[2]);
			if (x_cur < x_min) x_min = x_cur;
			else if (x_cur > x_max) x_max = x_cur;
			if (y_cur < y_min) y_min = y_cur;
			else if (y_cur > y_max) y_max = y_cur;
			if (z_cur < z_min) z_min = z_cur;
			else if (z_cur > z_max) z_max = z_cur;
			agt.geo_starts[i] = new Vec3D(x_cur, y_cur, z_cur);
			agt.geo_scores[i] = Float.parseFloat(parts[1]);
		}
		Vec3D trans = new Vec3D((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);
		int bx = (int) (Math.ceil((x_max - x_min) / ratio) * ratio);
		int by = (int) (Math.ceil((y_max - y_min) / ratio) * ratio);
		int bz = (int) (Math.ceil((z_max - z_min) / ratio) * ratio);
		if (bx > bX) bX = bx;
		if (by > bY) bY = by;
		if (bz > bZ) bZ = bz;
		for (Vec3D v : agt.geo_starts) v.subSelf(trans);
		agt.addAgents(lines.length, agt.geo_scores, agt.geo_starts);
	}

	void getBoundary(String filename){
		float x_min, y_min, z_min, x_max, y_max, z_max;
		z_min = y_min = x_min = Float.MAX_VALUE;
		z_max = y_max = x_max = Float.MIN_VALUE;
		String[] lines=loadStrings (filename);
		agt.outlines = new Vec3D[lines.length];
		agt.out_scores = new float[lines.length];
		for (int i = 0; i < lines.length; i++) {
			String[] coordinates = split(lines[i].substring(1, lines[i].length() - 1), ", ");
			float x_cur = Float.parseFloat(coordinates[0]);
			float y_cur = Float.parseFloat(coordinates[1]);
			float z_cur = Float.parseFloat(coordinates[2]);
			if (x_cur < x_min) x_min = x_cur;
			else if (x_cur > x_max) x_max = x_cur;
			if (y_cur < y_min) y_min = y_cur;
			else if (y_cur > y_max) y_max = y_cur;
			if (z_cur < z_min) z_min = z_cur;
			else if (z_cur > z_max) z_max = z_cur;
			agt.outlines[i]= new Vec3D (x_cur, y_cur, z_cur);
			agt.out_scores[i] = 0f;
		}
		Vec3D trans = new Vec3D((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);
		int bx = (int) (Math.ceil((x_max - x_min) / ratio) * ratio);
		int by = (int) (Math.ceil((y_max - y_min) / ratio) * ratio);
		int bz = (int) (Math.ceil((z_max - z_min) / ratio) * ratio);
		if (bx > bX) bX = bx;
		if (by > bY) bY = by;
		if (bz > bZ) bZ = bz;
		for (Vec3D v : agt.outlines) v.subSelf(trans);
		agt.addAgents(lines.length, agt.out_scores, agt.outlines);
	}

	void displayBox() {
		stroke(0, 0, 128);
		strokeWeight(1f);
		noFill();
		box(bX, bY, bZ);
	}
	
	void displayPoints(ArrayList<Vec3D> points, Vec3D color, float strkWt) {
		stroke(color.x, color.y, color.z);
		strokeWeight(strkWt);
		noFill();
		for (Vec3D v : points) point(v.x, v.y, v.z);
	}
	
	void displayOutline() {
		stroke(250,0,250);
		strokeWeight(4);
		noFill();
		for (Vec3D v : agt.outlines) {
			point(v.x, v.y, v.z);
		}
	}

	void displayStarts() {
		stroke(150);
		strokeWeight(2f);
		noFill();
		for (Vec3D v : agt.getStarts()) {
			point(v.x, v.y, v.z);
		}
	}

	void displayLocs() {
		strokeWeight(4);// no stroke for the shape.
		for (Agent a : agt.getAgents()) {
			String agtType = a.getType();
			Vec3D loc = a.getLoc();
			if (agtType.equals("a")) {
				stroke(0, 0, 250);// fill color, a dark blue.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
			if (agtType.equals("b")) {
				stroke(0, 250, 0);// fill color, a dark green.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
			if (agtType.equals("c")) {
				stroke(250, 0, 0);// fill color, a dark red.
				point(loc.x, loc.y, loc.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
			}
		}
	}

	void displayTrails() {
		strokeWeight(2);// no stroke for the shape.
		for (Agent a : agt.getAgents()) {
			String agtType = a.getType();
			ArrayList<Vec3D> trail = a.getTrail();
			if (agtType.equals("a")) {
				stroke(0, 0, 200);// fill color, a dark blue.
				for (Vec3D v : trail) {
					point(v.x, v.y, v.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
				}
			}
			if (agtType.equals("b")) {
				stroke(0, 200, 0);// fill color, a dark green.
				for (Vec3D v : trail) {
					point(v.x, v.y, v.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
				}
			}
			if (agtType.equals("c")) {
				stroke(200, 0, 0);// fill color, a dark red.
				for (Vec3D v : trail) {
					point(v.x, v.y, v.z);// an ellipse with center in the loc vector's coordinates, and 4 units wide and 4 units tall.
				}
			}
		}
	}
	
	void displayMeshes(boolean strk) {
		if (strk) {
			stroke(0, 0, 255);			
			gfx.mesh(meshA, true);
			stroke(0, 255, 0);
			gfx.mesh(meshB, true);
			stroke(255, 0, 0);	
			gfx.mesh(meshC, true);
			stroke(128, 128, 128);
			gfx.mesh(meshS, true);
		} else {
			noStroke();
			fill(0, 0, 200);			
			gfx.mesh(meshA, true);
			fill(0, 200, 0);	
			gfx.mesh(meshB, true);
			fill(200, 0, 0);	
			gfx.mesh(meshC, true);
//			fill(200, 200, 200);
//			gfx.mesh(meshS, true);
		}
	}

	void displayConnections() {
		stroke(64);
		strokeWeight(1.0f);
		ArrayList<AgentLine> connections = agt.getConnections();
		for (AgentLine l : connections) {
			Vec3D pta = l.getPta();
			Vec3D ptb = l.getPtb();
			line(pta.x, pta.y, pta.z, ptb.x, ptb.y, ptb.z);
		}
	}

	void drawLines(){
		for (Agent a : agt.getAgents()) {
			a.getClass();
		}
	}


	void drawTrails() {
		for (Agent a : agt.getAgents()) {
			if (!a.getType().equals("")) {
				VolumetricBrush brush;
				if(a.getType().equals("a")){
					brush = brushA;
					stroke(0, 255, 0);
					strokeWeight(2.0f);
				} else if(a.getType().equals("b")){
					brush = brushB;
					stroke(255, 255, 0);
					strokeWeight(1.0f);
				} else if(a.getType().equals("c")) {
					brush = brushC;
					stroke(0, 0, 255);
					strokeWeight(1.0f);
				} else brush = brushA;

				for (Vec3D v : a.getTrail()) {
					point(v.x, v.y, v.z);
					brush.drawAtAbsolutePos(v, isoBrushDensity);
				}
			}
		}
		for (int i = 0; i < agt.GRIDX; i++) {
			for (int j = 0; j < agt.GRIDY; i++) {
				for (int k = 0; k < agt.GRIDZ; k++) {
					int indexA = agt.volumeA.getIndexFor(i, j, k);
					int indexB = agt.volumeB.getIndexFor(i, j, k);
					int indexC = agt.volumeC.getIndexFor(i, j, k);
					if (0 != agt.volumeA.getVoxelAt(indexA) && 0 != agt.volumeB.getVoxelAt(indexB)) agt.volumeB.setVoxelAt(indexB, 0f);
					if (0 != agt.volumeA.getVoxelAt(indexB) && 0 != agt.volumeB.getVoxelAt(indexC)) agt.volumeC.setVoxelAt(indexC, 0f);
				}
			}
		}
		surfaceA.reset();
		surfaceA.computeSurfaceMesh(meshA, ISO);
		surfaceB.reset();
		surfaceB.computeSurfaceMesh(meshB, ISO);
		surfaceC.reset();
		surfaceC.computeSurfaceMesh(meshC, ISO);
	}

	public void keyPressed() {

		// Video
		if (key == 'r') {
			videoRecord = ! videoRecord;  // start/stop the movie (one press start, second press stop)
			if (videoRecord) {
				println("MOVIE STARTED!");
				movieCounter = 0;
			}
			if (!videoRecord) { 
				println("MOVIE STOPPED!");
			}
		}
		if (key=='s'){
			meshA.saveAsSTL(sketchPath(meshA.name + frameCount+ counter + ".stl"));
			meshB.saveAsSTL(sketchPath(meshB.name + frameCount+ counter + ".stl"));
			meshC.saveAsSTL(sketchPath(meshC.name + frameCount+ counter + ".stl"));
			counter = counter + 1;
			println ("Saved Successfull");
		}
		if (key=='e' || key== 'E') {
			// saveFrame("/output/seq-####.jpg");
			saveFrame("MSA_agent_cat_Trails" + frameCount + ".png");
			println("saved a frame");
		}
		if (key=='r') capture= !capture;
		if (key == 'n') {
			runToggle = !runToggle;
			agt.switchAgents();
		}
		if (key == 'a' || key == 'A') runAttraction= !runAttraction;
		if (key == 'p') record= !record;
		if (key == 'k') strok = !strok;
		if (key == 'c' ||key == 'C') compute = !compute;
		if (key == 't') trail = !trail;
	}

	void recording(){
		if(videoRecord==true){
			String saveName = "Agents" + "__" + nf(movieCounter, 4) + ".png";
			saveFrame(saveName);
			println(saveName);
			movieCounter+=2;
		}
	}

	public static void main(String _args[]) {
		PApplet.main(new String[] { multiagentgeometryexploringsystem.MultiAgentGeometryExploringSystem.class.getName() });
	}
}
