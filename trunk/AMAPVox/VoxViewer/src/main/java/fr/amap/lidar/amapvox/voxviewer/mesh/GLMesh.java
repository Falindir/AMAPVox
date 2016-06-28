/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.amap.lidar.amapvox.voxviewer.mesh;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.GL3;
import fr.amap.commons.math.matrix.Mat4F;
import fr.amap.commons.math.point.Point3F;
import fr.amap.commons.math.vector.Vec3F;
import fr.amap.commons.math.vector.Vec4F;
import fr.amap.commons.util.Statistic;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */
public abstract class GLMesh {
    
    public enum DrawType{
        
        TRIANGLES(GL3.GL_TRIANGLES),
        POINTS(GL3.GL_POINTS),
        LINES(GL3.GL_LINES);
        
        private final int type;

        private DrawType(int type) {
            this.type = type;
        }

        public int get() {
            return type;
        }
    }
    
    public static final int FLOAT_SIZE = Buffers.SIZEOF_FLOAT;
    public static final int INTEGER_SIZE = Buffers.SIZEOF_INT;
    //public static final int SHORT_SIZE = Buffers.SIZEOF_SHORT;
    public static final int DEFAULT_SIZE = -1;
    
    long offset = 0;
    long totalBuffersSize;
    
    protected FloatBuffer vertexBuffer;
    public FloatBuffer normalBuffer;
    public IntBuffer indexBuffer;
    public FloatBuffer colorBuffer;
    public int vertexCount;
    public int dimensions = 3;
    
    private Statistic xValues;
    private Statistic yValues;
    private Statistic zValues;
    
    
    /**
     * Vertex Buffer Object identifier
     * @link <a href="https://www.opengl.org/wiki/Vertex_Specification#Vertex_Buffer_Object">https://www.opengl.org/wiki/Vertex_Specification#Vertex_Buffer_Object</a>
     */
    private int vboId;

    /**
     * Index Buffer Object Identifier
     * @link <a href="https://www.opengl.org/wiki/Vertex_Specification#Index_buffers">https://www.opengl.org/wiki/Vertex_Specification#Index_buffers</a>
     */
    private int iboId;
    
    protected List<Long> offsets;
    protected List<Long> buffersSizes;
    
    public GLMesh(){
        
        totalBuffersSize = 0;
        offset = 0;
        vboId = -1;
        offsets = new ArrayList<>();
        buffersSizes = new ArrayList<>();
        
        xValues = new Statistic();
        yValues = new Statistic();
        zValues = new Statistic();
    }
    
    public void resetIds(){
        vboId = -1;
        iboId = -1;
    }
    
    
    /**
     *
     * @param gl opengl context
     * @param drawType
     */
    public abstract void draw(GL3 gl, DrawType drawType);
    public abstract void initBuffers(GL3 gl, long maximumTotalBufferSize);
    /*
    public void initBuffers(GL3 gl, int maxSize, ShortBuffer indexBuffer, FloatBuffer... floatBuffers){
        
        bindBuffer(gl);
        
        if(maxSize == DEFAULT_SIZE){
            for (FloatBuffer buffer : floatBuffers) {
                totalBuffersSize += buffer.capacity()*FLOAT_SIZE;
            }
        }else{
            totalBuffersSize = maxSize;
        }
        
        gl.glBufferData(GL3.GL_ARRAY_BUFFER, totalBuffersSize, null, GL3.GL_STATIC_DRAW);
        
        for (FloatBuffer buffer : floatBuffers) {
            addSubBuffer(gl, buffer);
        }
        
        gl.glBindBuffer(GL3.GL_ELEMENT_ARRAY_BUFFER, iboId);
            gl.glBufferData(GL3.GL_ELEMENT_ARRAY_BUFFER, indexBuffer.capacity()*SHORT_SIZE, indexBuffer, GL3.GL_STATIC_DRAW);
        gl.glBindBuffer(GL3.GL_ELEMENT_ARRAY_BUFFER, 0);
        
        unbindBuffer(gl);
    }*/
    /*
    public void initBuffersV2(GL3 gl, int maxSize, ShortBuffer indexBuffer, FloatBuffer... floatBuffers){
        
        bindBuffer(gl);
        
        totalBuffersSize = maxSize;
        gl.glBufferData(GL3.GL_ARRAY_BUFFER, totalBuffersSize, null, GL3.GL_STATIC_DRAW);
        
        for (FloatBuffer buffer : floatBuffers) {
            addSubBuffer(gl, buffer);
        }
        
        gl.glBindBuffer(GL3.GL_ELEMENT_ARRAY_BUFFER, iboId);
            gl.glBufferData(GL3.GL_ELEMENT_ARRAY_BUFFER, indexBuffer.capacity()*SHORT_SIZE, indexBuffer, GL3.GL_STATIC_DRAW);
        gl.glBindBuffer(GL3.GL_ELEMENT_ARRAY_BUFFER, 0);
        
        unbindBuffer(gl);
    }*/
    
    /**
     *
     * @param gl opengl context
     */
    public void bindBuffer(GL3 gl){
        
        gl.glBindBuffer(GL3.GL_ARRAY_BUFFER, vboId);
    }
    
    /**
     * Add buffer into current global buffer
     * @param gl opengl context
     * @param buffer buffer to set
     */
    protected void addSubBuffer(GL3 gl, FloatBuffer buffer){
        long bufferSize = buffer.capacity()*FLOAT_SIZE;
        gl.glBufferSubData(GL3.GL_ARRAY_BUFFER, offset, bufferSize, buffer);
        offsets.add(offset);
        offset += bufferSize;
        buffersSizes.add(bufferSize);
    }
    
    public void updateColorBuffer(GL3 gl, int index){
        
        bindBuffer(gl);
        
            gl.glBufferSubData(GL3.GL_ARRAY_BUFFER, offsets.get(index), buffersSizes.get(index), colorBuffer);
        
        unbindBuffer(gl);
    }
    
    /**
     * Update buffer linked to the specified index
     * @param gl opengl context
     * @param index subbuffer index
     * @param buffer buffer to update
     */
    public void updateBuffer(GL3 gl, int index, FloatBuffer buffer){
        
        bindBuffer(gl);
            
            long bufferSize = buffer.capacity()*FLOAT_SIZE;
            
            if(index >= buffersSizes.size()){
                buffersSizes.add(bufferSize);
            }
            
            if(index >= offsets.size()){
                offsets.add(computeOffset(index));
            }
            
            long difference = bufferSize-buffersSizes.get(index);
            
            totalBuffersSize += difference;
            //gl.glBufferData(GL3.GL_ARRAY_BUFFER, totalBuffersSize, null, GL3.GL_STATIC_DRAW);
        
            gl.glBufferSubData(GL3.GL_ARRAY_BUFFER, offsets.get(index), bufferSize, buffer);
            
            long oldBufferSize = buffersSizes.get(index);
            
            if(bufferSize != oldBufferSize){
                
                buffersSizes.set(index, bufferSize);
                
                if((index+1) < offsets.size()){
                    
                    offsets.set(index+1, computeOffset(index+1));
                }
            }
        
        unbindBuffer(gl);
    }
    
    private long computeOffset(int index){
        
        long offsetTot = 0;
        
        if(index <= buffersSizes.size()){
            
            for(int i=0;i<index;i++) {
                offsetTot+=buffersSizes.get(i);
            }
        }
        
        return offsetTot;
    }

    /**
     *
     * @param gl opengl context
     */
    
    public void unbindBuffer(GL3 gl){
        gl.glBindBuffer(GL3.GL_ARRAY_BUFFER, 0);
    }

    /**
     *
     * @return Vertex Buffer Object identifier
     * @see <a href="https://www.opengl.org/wiki/Vertex_Specification#Vertex_Buffer_Object">https://www.opengl.org/wiki/Vertex_Specification#Vertex_Buffer_Object</a>
     */
    public int getVboId() {
        return vboId;
    }

    /**
     *
     * @return Index Buffer Object Identifier
     * @see <a href="https://www.opengl.org/wiki/Vertex_Specification#Index_buffers">https://www.opengl.org/wiki/Vertex_Specification#Index_buffers</a>
     */
    public int getIboId() {
        return iboId;
    }
    
    
    protected void initVBOAndIBO(GL3 gl){
        
        IntBuffer tmp = IntBuffer.allocate(2);
        gl.glGenBuffers(2, tmp);
        vboId=tmp.get(0);
        iboId=tmp.get(1);
        
        totalBuffersSize = 0;
        offset = 0;
        offsets = new ArrayList<>();
        buffersSizes = new ArrayList<>();
    }
    
    protected void sendIBOData(GL3 gl){
        
        gl.glBindBuffer(GL3.GL_ELEMENT_ARRAY_BUFFER, iboId);
            gl.glBufferData(GL3.GL_ELEMENT_ARRAY_BUFFER, indexBuffer.capacity()*INTEGER_SIZE, indexBuffer, GL3.GL_STATIC_DRAW);
        gl.glBindBuffer(GL3.GL_ELEMENT_ARRAY_BUFFER, 0);
    }
    
    public void setGlobalScale(float scale){
        
        if(vertexBuffer != null){
            
            float[] tab = new float[vertexBuffer.capacity()];
            vertexBuffer.get(tab);

            for(int i=0;i<tab.length;i++){
                tab[i] *= scale;
            }

            vertexBuffer = Buffers.newDirectFloatBuffer(tab);
        }
    }   
    
    public void setVertexData(float[] vertices){
        vertexBuffer = Buffers.newDirectFloatBuffer(vertices);
    }
    
    public void setColorData(float[] colors){
        colorBuffer = Buffers.newDirectFloatBuffer(colors);
    }

    public FloatBuffer getVertexBuffer() {
        return vertexBuffer;
    }

    public void setVertexBuffer(FloatBuffer vertexBuffer) {
        
        this.vertexBuffer = vertexBuffer;
        
        computeGravityCenter();
    }
    
    public void computeGravityCenter(){
        
        if(vertexBuffer != null){
            
            xValues = new Statistic();
            yValues = new Statistic();
            zValues = new Statistic();
            
            for(int j = 0 ; j<this.vertexBuffer.capacity(); j+=3){
            
                xValues.addValue(this.vertexBuffer.get(j));
                yValues.addValue(this.vertexBuffer.get(j+1));
                zValues.addValue(this.vertexBuffer.get(j+2));
            }
        }
    }
    
    public Point3F getGravityCenter(){
        
        return new Point3F((float)xValues.getMean(), (float)yValues.getMean(), (float)zValues.getMean());
    }
    
    /**
     * Translate the vertices contained inside the vertex buffer
     * @param translation The translation vector
     */
    public void translate(Vec3F translation){
                
        for(int j = 0 ; j<vertexBuffer.capacity(); j+=3){
            
            
            float x = vertexBuffer.get(j);
            float y = vertexBuffer.get(j+1);
            float z = vertexBuffer.get(j+2);
            
            vertexBuffer.put(j, x+translation.x);
            vertexBuffer.put(j+1, y+translation.y);
            vertexBuffer.put(j+2, z+translation.z);
            
        }
        
        computeGravityCenter();
    }
    
    public void scale(Vec3F scale){
                
        for(int j = 0 ; j<vertexBuffer.capacity(); j+=3){
            
            
            float x = vertexBuffer.get(j);
            float y = vertexBuffer.get(j+1);
            float z = vertexBuffer.get(j+2);
            
            vertexBuffer.put(j, x*scale.x);
            vertexBuffer.put(j+1, y*scale.y);
            vertexBuffer.put(j+2, z*scale.z);
        }
        
        computeGravityCenter();
        
    }
    
    public void rotate(Mat4F rotation){
                
        for(int j = 0 ; j<vertexBuffer.capacity(); j+=3){
            
            float x = vertexBuffer.get(j);
            float y = vertexBuffer.get(j+1);
            float z = vertexBuffer.get(j+2);
            
                
            Vec4F result = Mat4F.multiply(rotation, new Vec4F(x, y, z, 1));
            vertexBuffer.put(j, result.x);
            vertexBuffer.put(j+1, result.y);
            vertexBuffer.put(j+2, result.z);
        }
        
        computeGravityCenter();
    }

    public Statistic getxValues() {
        return xValues;
    }

    public Statistic getyValues() {
        return yValues;
    }

    public Statistic getzValues() {
        return zValues;
    }
}
