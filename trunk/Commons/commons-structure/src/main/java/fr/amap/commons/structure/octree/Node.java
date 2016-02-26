
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

package fr.amap.commons.structure.octree;

import fr.amap.commons.math.point.Point3D;
import fr.amap.commons.math.point.Point3F;
import fr.amap.commons.math.point.Point3I;
import java.util.Arrays;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */


public class Node {
        
    private boolean leaf;
    private Node[] childs;
    private int[] elements;
    private int elementNumber;
    private Point3D minPoint;
    private Point3D maxPoint;
    
    public Node(Point3D minPoint, Point3D maxPoint){
        
        this.minPoint = minPoint;
        this.maxPoint = maxPoint;
        
        init();
    }
    
    public Node(Node parent, short indice) throws Exception{
        
        if(parent != null && indice >= 0 && indice <= 7){
            
            init();
            
            short[] decToBin = decToBin(indice);
            
            short indiceX = decToBin[0];
            short indiceY = decToBin[1];
            short indiceZ = decToBin[2];
            
            double minPointX, minPointY, minPointZ;
            double maxPointX, maxPointY, maxPointZ;
            
            if(indiceX == 0){
                minPointX = parent.minPoint.x;
                maxPointX = (parent.minPoint.x+parent.maxPoint.x)/2.0f;
            }else{
                minPointX = (parent.minPoint.x+parent.maxPoint.x)/2.0f;
                maxPointX = parent.maxPoint.x;
            }
            
            if(indiceY == 0){
                minPointY = parent.minPoint.y;
                maxPointY = (parent.minPoint.y+parent.maxPoint.y)/2.0f;
            }else{
                minPointY = (parent.minPoint.y+parent.maxPoint.y)/2.0f;
                maxPointY = parent.maxPoint.y;
            }
            
            if(indiceZ == 0){
                minPointZ = parent.minPoint.z;
                maxPointZ = (parent.minPoint.z+parent.maxPoint.z)/2.0f;
            }else{
                minPointZ = (parent.minPoint.z+parent.maxPoint.z)/2.0f;
                maxPointZ = parent.maxPoint.z;
            }
            
            minPoint = new Point3D(minPointX, minPointY, minPointZ);
            maxPoint = new Point3D(maxPointX, maxPointY, maxPointZ);
            
        }else{
            throw new Exception("Cannot instantiate node cause parent is null or indice is not range between 0 to 7");
        }
    }
    
    private void init(){
        leaf = true;
        childs = null;
    }

    public boolean isLeaf() {
        return leaf;
    }
    
    public void insertElement(Octree octree, int indice) throws Exception{
        
        //la condition est vraie quand aucun point n'a déjà été ajouté au noeud
        if(childs == null && elements == null){
            
            elements = new int[octree.getMaximumPoints()];
            elementNumber = 0;
        }
        
        
        if(childs == null && elements !=null){
            
            if(elementNumber < elements.length){ 
            
                elements[elementNumber] = indice;
                elementNumber++;
            }else{ //la condition est vraie quand le nombre maximum de points dans le noeud a été atteint
                
                // on crée les enfants
                subdivide();
                
                //on replace tous les points du noeud dans ses enfants
                for(int i = 0 ; i< elementNumber;i++){
                    insertElement(octree, elements[i]);
                }
                
                elements = null;
            }
            
            
        }
        
        if(childs != null){
            
            //on détermine dans quel enfant se trouve le point
            int childContainingPointID;
            Point3D point = octree.getPoints()[indice];

            Point3I indices = get3DIndicesFromPoint(point);

            childContainingPointID = get1DIndiceFrom3DIndices(indices.x, indices.y, indices.z);

            if(isInsideRange(childContainingPointID, 0, 7)){
                //on ajoute le point à l'enfant
                childs[childContainingPointID].insertElement(octree, indice);
            }else{
                throw new Exception("Cannot insert point!");
                
            }
            
        }
    }
    
    public short get1DIndiceFromPoint(Point3D point){
        
        Point3I indices = get3DIndicesFromPoint(point);
        
        if(indices == null){
            return -1;
        }
        
        return get1DIndiceFrom3DIndices(indices.x, indices.y, indices.z);
    }
    
    public Point3I get3DIndicesFromPoint(Point3D point){
        
        int indiceX = (int)((point.x-minPoint.x)/((maxPoint.x-minPoint.x)/2.0f));
        int indiceY = (int)((point.y-minPoint.y)/((maxPoint.y-minPoint.y)/2.0f));
        int indiceZ = (int)((point.z-minPoint.z)/((maxPoint.z-minPoint.z)/2.0f));
        
        //cas où la coordonnée du point est sur la limite maximum de la bounding-box
        if(indiceX == 2){indiceX = 1;}
        if(indiceY == 2){indiceY = 1;}
        if(indiceZ == 2){indiceZ = 1;}
        
        //cas où la coordonnée est à l'extérieur de la bounding-box
        if(!isInsideRange(indiceX, 0, 1) || !isInsideRange(indiceY, 0, 1) || !isInsideRange(indiceZ, 0, 1)){
            return null;
        }
        
        return new Point3I(indiceX, indiceY, indiceZ);
    }
    
    public boolean hasChilds(){
        
        return childs != null;
    }

    public int[] getElements() {
        
        if(elements != null){
            return Arrays.copyOf(elements, elementNumber);
        }
        return null;
    }
    
    private void subdivide() throws Exception{
        
        childs = new Node[8];
        
        for(short i=0;i<8;i++){
            childs[i] = new Node(this, i);
        }
        
        leaf = false;
    }
    
    private short get1DIndiceFrom3DIndices(int bit1, int bit2, int bit3){
        
        short dec = 0;
        
        dec += bit1 * Math.pow(2, 2);
        dec += bit2 * Math.pow(2, 1);
        dec += bit3 * Math.pow(2, 0);
        
        return dec;
    }
    
    private short[] decToBin(short decimal){
        
        short[] result = new short[3];
        short tmp = decimal;
        
        for(short i=2 ; i >= 0 ; i--){
            result[i] = (short) (tmp%2);
            tmp = (short) (tmp/2);
        }
        
        return result;
    }
    
    public Node getChild(short indice){
        
        if(childs != null && indice <= 7 && indice >= 0){
            return childs[indice];
        }
        
        return null;
    }
    
    private boolean isInsideRange(int value, int min, int max){
        return (value >= min && value <= max);
    }

    public Point3D getMinPoint() {
        return minPoint;
    }

    public Point3D getMaxPoint() {
        return maxPoint;
    }
    
}
