/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.amap.lidar.amapvox.gui.viewer3d;

import fr.amap.commons.math.point.Point3F;
import fr.amap.lidar.amapvox.voxviewer.mesh.GLMesh;
import fr.amap.lidar.amapvox.voxviewer.object.scene.SceneObject;
import fr.amap.lidar.amapvox.voxviewer.object.scene.VoxelSpaceAdapter;
import fr.amap.lidar.amapvox.voxviewer.object.scene.VoxelSpaceSceneObject;
import java.io.File;
import javafx.stage.Stage;

/**
 *
 * @author calcul
 */
public class VoxImportTask extends SceneObjectImportTask {

    public VoxImportTask(File file) {
        super(file);
    }

    
    @Override
    public void showImportFrame(Stage stage) throws Exception {
        
    }

    @Override
    protected SceneObject call() throws Exception {
                
        VoxelSpaceSceneObject voxelSpace = new VoxelSpaceSceneObject(file);

        voxelSpace.addVoxelSpaceListener(new VoxelSpaceAdapter() {

            @Override
            public void voxelSpaceCreationProgress(int progress) {
                updateProgress(progress, 100);
            }
        });

        voxelSpace.loadVoxels();

        voxelSpace.changeCurrentAttribut("transmittance");
        voxelSpace.setShader(fr.amap.lidar.amapvox.voxviewer.object.scene.Scene.instanceLightedShader);
        voxelSpace.setDrawType(GLMesh.DrawType.TRIANGLES);
        voxelSpace.setGravityCenter(new Point3F(0, 0, 0));

        return voxelSpace;
    }
    
}
