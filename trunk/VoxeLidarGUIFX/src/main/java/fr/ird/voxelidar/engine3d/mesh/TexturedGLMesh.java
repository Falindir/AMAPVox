/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidar.engine3d.mesh;

import com.jogamp.opengl.GL3;
import static fr.ird.voxelidar.engine3d.mesh.GLMesh.DEFAULT_SIZE;
import static fr.ird.voxelidar.engine3d.mesh.GLMesh.FLOAT_SIZE;
import static fr.ird.voxelidar.engine3d.mesh.GLMesh.SHORT_SIZE;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

/**
 *
 * @author Julien Heurtebize (julienhtbe@gmail.com)
 */
public class TexturedGLMesh extends GLMesh{
    
    public FloatBuffer textureCoordinatesBuffer;

    public TexturedGLMesh() {
    }
    
    public TexturedGLMesh(GL3 gl) {
        super(gl);
    }

    @Override
    public void initBuffers(GL3 gl, long maximumTotalBufferSize) {
        
        if(textureCoordinatesBuffer != null){
            
            initVBOAndIBO(gl);

            bindBuffer(gl);

            FloatBuffer[] floatBuffers = new FloatBuffer[]{vertexBuffer, textureCoordinatesBuffer};

            if(maximumTotalBufferSize == DEFAULT_SIZE){
                for (FloatBuffer buffer : floatBuffers) {
                    totalBuffersSize += buffer.capacity()*FLOAT_SIZE;
                }
            }else{
                totalBuffersSize = maximumTotalBufferSize;
            }

            gl.glBufferData(GL3.GL_ARRAY_BUFFER, totalBuffersSize, null, GL3.GL_STATIC_DRAW);

            for (FloatBuffer buffer : floatBuffers) {
                addSubBuffer(gl, buffer);
            }

            sendIBOData(gl);

            unbindBuffer(gl);
        }
    }

    @Override
    public void draw(GL3 gl) {
                
        gl.glDrawElements(GL3.GL_TRIANGLES, vertexCount, GL3.GL_UNSIGNED_SHORT, 0);
    }
}