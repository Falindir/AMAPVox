package fr.amap.lidar.amapvox.voxelisation.mixmodtrans;

/**
 * @author Jimmy Lopez
 */
public class MiniVoxel {

    public int ID;

    public int I;

    public int J;

    public int K;

    public float trials;

    public float prop;

    public MiniVoxel(int ID, int i, int j, int k, float trials, float prop) {
        this.ID = ID;
        I = i;
        J = j;
        K = k;
        this.trials = trials;
        this.prop = prop;
    }
}
