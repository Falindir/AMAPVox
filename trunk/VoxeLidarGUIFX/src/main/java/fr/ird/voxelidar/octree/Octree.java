/*
This software is distributed WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

This program is open-source LGPL 3 (see copying.txt).
Authors:
    Gregoire Vincent    gregoire.vincent@ird.fr
    Julien Heurtebize   julienhtbe@gmail.com
    Jean Dauzat         jean.dauzat@cirad.fr
    Rémi Cresson        cresson.r@gmail.com

For further information, please contact Gregoire Vincent.
 */

package fr.ird.voxelidar.octree;

import fr.ird.voxelidar.engine3d.math.point.Point3F;
import java.util.ArrayList;
import java.util.List;
import org.apache.log4j.Logger;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */


public class Octree {
    
    private final static Logger logger = Logger.getLogger(Octree.class);
    
    private Point3F[] points;
    private Point3F minPoint;
    private Point3F maxPoint;
    private int depth;
    private Node root;
    private List<Node> leafs;
    
    private final int maximumPoints;
    
    public final static short BINARY_SEARCH = 0;
    public final static short INCREMENTAL_SEARCH = 1;
    
    public Octree(int maximumPoints){
        this.maximumPoints = maximumPoints;
        depth = 0;
    }
    
    public void build(){
        
        if(points != null){
            
            root = new Node(minPoint, maxPoint);
            
            for(int i=0;i<points.length;i++){
                
                root.insertPoint(this, i);
            }
            
        }else{
            logger.warn("Attempt to build octree but points array is null");
        }
    }
    
    public List<Node> getLeafs(){
        
        leafs = new ArrayList<>();
        
        if(root != null){
            getChilds(root);
        }
        
        return leafs;
    }
    
    private void getChilds(Node node){
        
        if(node.hasChilds()){
            
            for(short i=0;i<8;i++){
                Node child = node.getChild(i);
                if(child.isLeaf()){
                    leafs.add(child);
                }else{
                    getChilds(child);
                }
            }

        }
    }
    
    public Node traverse(Point3F point){
        
        Node node = null;
        
        if(root != null){
            
            node = root;
            
            while(node.hasChilds()){
                short indice = node.get1DIndiceFromPoint(point);
                
                //le point est à l'extérieur de la bounding-box
                if(indice == -1){
                    return null;
                }
                
                Node child = node.getChild(indice);
                node = child;
                
            }
        }
        
        return node;
    }
    
    public Point3F searchNearestPoint(Point3F point, short type, float errorMargin){
        
        switch(type){
            case BINARY_SEARCH:
                break;
            case INCREMENTAL_SEARCH:
                return incrementalSearchNearestPoint(point, errorMargin);
        }
        
        return null;
    }
    
    public boolean isPointBelongsToPointcloud(Point3F point, float errorMargin){
        
        Point3F incrementalSearchNearestPoint = searchNearestPoint(point, Octree.INCREMENTAL_SEARCH, errorMargin);
                            
                            
        boolean test = false;
        if(incrementalSearchNearestPoint != null){
            float distance = point.distanceTo(incrementalSearchNearestPoint);
            

            if(distance < errorMargin){
                test = true;
            }
        }
        
        return test;
    }
    
    private Point3F incrementalSearchNearestPoint(Point3F point, float errorMargin){
        
        Point3F nearestPoint = null;
        
        if(root != null){
            
            Node leaf = traverse(point);
            
            if(leaf == null){
                return null;
            }
            
            float distance = 99999999;
            
            int[] nearestPoints = leaf.getPoints();
            
            if(nearestPoints != null){
                
                for (int pointToTest : nearestPoints) {
                    
                    float dist = point.distanceTo(points[pointToTest]);
                    
                    if(dist < distance){
                        distance = dist;
                        nearestPoint = points[pointToTest];
                    }
                }
            }
            
            if(distance > errorMargin){
                
                List<Node> nodesIntersectingSphere = new ArrayList<>();
                
                Sphere searchArea = new Sphere(point, errorMargin);
                
                incrementalSphereIntersectionSearch(nodesIntersectingSphere, root, searchArea);
                
                for(Node node : nodesIntersectingSphere){
                    
                    nearestPoints = node.getPoints();

                    if(nearestPoints != null){

                        for (int pointToTest : nearestPoints) {

                            float dist = point.distanceTo(points[pointToTest]);

                            if(dist < distance){
                                distance = dist;
                                nearestPoint = points[pointToTest];
                            }
                        }
                    }
                }
            }
            
        }
        
        return nearestPoint;
    }
    
    private void incrementalSphereIntersectionSearch(List<Node> nodesIntersectingSphere, Node node, Sphere sphere){
        
        if(sphereIntersection(node, sphere)){
                    
            if(node.hasChilds()){

                for(short i=0;i<8;i++){
                    Node child = node.getChild(i);
                    boolean intersect = sphereIntersection(child, sphere);

                    if(child.isLeaf() && intersect){
                        nodesIntersectingSphere.add(child);
                    }else{
                        incrementalSphereIntersectionSearch(nodesIntersectingSphere, child, sphere);
                    }
                }

            }else{
                nodesIntersectingSphere.add(node);
            }
        }
    }
    
    private boolean sphereIntersection(Node node, Sphere sphere){
        
        float dist_squared = sphere.getRadius()*sphere.getRadius();
        
        Point3F sphereCenter = sphere.getCenter();
        
        Point3F nodeBottomCorner = node.getMinPoint();
        Point3F nodeTopCorner = node.getMaxPoint();
        
        if (sphereCenter.x < nodeBottomCorner.x){
            dist_squared -= Math.pow(sphereCenter.x - nodeBottomCorner.x, 2);
        }else if (sphereCenter.x > nodeTopCorner.x) {
            dist_squared -= Math.pow(sphereCenter.x - nodeTopCorner.x, 2);
        }
        
        if (sphereCenter.y < nodeBottomCorner.y){
            dist_squared -= Math.pow(sphereCenter.y - nodeBottomCorner.y, 2);
        }else if (sphereCenter.y > nodeTopCorner.y) {
            dist_squared -= Math.pow(sphereCenter.y - nodeTopCorner.y, 2);
        }
        
        if (sphereCenter.z < nodeBottomCorner.z){
            dist_squared -= Math.pow(sphereCenter.z - nodeBottomCorner.z, 2);
        }else if (sphereCenter.z > nodeTopCorner.z) {
            dist_squared -= Math.pow(sphereCenter.z - nodeTopCorner.z, 2);
        }
        
        return dist_squared > 0;
    }

    public int getMaximumPoints() {
        return maximumPoints;
    }

    public Point3F[] getPoints() {
        return points;
    }

    public int getDepth() {
        return depth;
    }

    public void setDepth(int depth) {
        this.depth = depth;
    }

    public void setPoints(Point3F[] points) {
        this.points = points;
    }

    public void setMinPoint(Point3F minPoint) {
        this.minPoint = minPoint;
    }

    public void setMaxPoint(Point3F maxPoint) {
        this.maxPoint = maxPoint;
    }
    
}
