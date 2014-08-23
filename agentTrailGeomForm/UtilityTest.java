package agentTrailGeomForm;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import toxi.geom.Vec3D;

public class UtilityTest {

	@BeforeClass
	public static void setUpBeforeClass() throws Exception {
	}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {
	}

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void testParaboloid() {
		
	}

	@Test
	public void testGetNormalPoint() {
		
	}

	@Test
	public void testCoorToIndex() {
		int[] result = Utility.coorToIndex(new Vec3D(121.5f, 37.5f, 89.5f), 1000, 1000, 1000, 4);
		System.out.printf("%d, %d, %d\n", result[0], result[1], result[2]);
		assertEquals(155, result[0]);
		assertEquals(134, result[1]);
		assertEquals(147, result[2]);
	}

}
