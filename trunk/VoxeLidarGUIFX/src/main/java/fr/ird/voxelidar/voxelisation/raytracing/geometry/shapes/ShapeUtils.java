package fr.ird.voxelidar.voxelisation.raytracing.geometry.shapes;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import fr.ird.voxelidar.voxelisation.raytracing.util.BoundingBox3d;

/**
 * Common operations on shapes
 * @author Cresson, Nov. 2012
 *
 */
public class ShapeUtils {

		
	/**
	 * Create a rectangle box, aligned in XYZ axis (like a boundingbox)
	 * @param boundingBox	bounding box
	 * @param margin		margin
	 * @param culling		culling
	 * @return rectangular volumic shape
	 */
	public static ConvexMesh createRectangleBox(BoundingBox3d boundingBox, double margin, boolean culling) {
		
		// Walls
		double[]x = {boundingBox.getMax().x-margin, boundingBox.getMin().x+margin};
		double[]y = {boundingBox.getMax().y-margin, boundingBox.getMin().y+margin};
		double[]z = {boundingBox.getMax().z-margin, boundingBox.getMin().z+margin};

		// Get 8 corners of the bbox
		Point3d[] bboxPoint = new Point3d[8];
		for (int i = 0; i < 8 ; i++)
			bboxPoint[i] = new Point3d(x[i%2],y[((i-i%2)/2)%2],z[((i-i%4)/4)%2]);

		// Create paths
		int[][] paths = new int[12][];
		paths[0] = new int[] {1, 7, 3};
		paths[1] = new int[] {1, 5, 7};
		paths[2] = new int[] {0, 5, 1};
		paths[3] = new int[] {0, 4, 5};
		paths[4] = new int[] {2, 4, 0};
		paths[5] = new int[] {2, 6, 4};
		paths[6] = new int[] {3, 6, 2};
		paths[7] = new int[] {3, 7, 6};
		paths[8] = new int[] {0, 3, 2};
		paths[9] = new int[] {0, 1, 3};
		paths[10]= new int[] {5, 6, 7};
		paths[11]= new int[] {5, 4, 6};

		// Compute Box
		return new ConvexMesh(bboxPoint, paths, culling);
	}
	
	
	/**
	 * Add margins to the given bounding box
	 * @param bbox		bounding box
	 * @param margin	margin
	 * @return the new bounding box, updated with the given margin
	 */
	public static BoundingBox3d getPaddedBoundingBox(BoundingBox3d bbox, double margin) {
		
		Point3d ptSup 		= bbox.getMax();
		Point3d ptInf 		= bbox.getMin();
		Point3d epsVector 	= new Point3d(	margin,
											margin,
											margin);
		ptSup.add (epsVector);
		ptInf.sub (epsVector);
		BoundingBox3d newBbox = new BoundingBox3d ();
		newBbox.setMin (ptInf);
		newBbox.setMax (ptSup);
		return newBbox;
	}
	
}
