package fr.amap.lidar.amapvox.voxelisation.mixmodtrans;

import fr.amap.lidar.amapvox.commons.Voxel;
import javax.vecmath.Point3i;

/**
 * @author Jimmy Lopez
 */
public class MixModTrans {

    private Voxel[][][] voxels;

    private Point3i split;

    private MiniVoxel[] filtered_data;

    private float[][] tempVoxel;

    private int ground_distance = 0;

    private float max_ground_distance = 0;

    private RCommunication R;

    private String[] ijkInit;

    private double[][][][] resultat;

    public MixModTrans(Voxel[][][] voxels, Point3i split) {
        this.voxels = voxels;
        this.split = split;
        ijkInit = new String[split.getX()*split.getY()*split.getZ()];
        R = new RCommunication();
    }

    /**
     * Prétraitement pour l'utilisation du code R
     * @return
     */
    public MixModTrans preprocess() {

        int nline = 0;

        for (int i = 0; i < split.x; i++) {
            for (int j = 0; j < split.y; j++) {
                for (int k = 0; k < split.z; k++) {
                    Voxel v = voxels[i][j][k];
                    float ground = Tools.round(v.ground_distance);
                    if(ground > 0) {
                        ground_distance ++;
                        max_ground_distance = Math.max(max_ground_distance, ground);
                    }

                    ijkInit[nline] = Tools.createIJKVoxel(v);

                    nline++;
                }
            }
        }

        filtered_data = new MiniVoxel[this.ground_distance];
        int filter = 0;
        int index = 0;
        for (int i = 0; i < split.x; i++) {
            for (int j = 0; j < split.y; j++) {
                for (int k = 0; k < split.z; k++) {
                    Voxel v = voxels[i][j][k];
                    float ground = Tools.round(v.ground_distance);
                    if (ground > 0) {

                        float trials = Tools.round((float)(v.bvEntering / TConstant.EP));
                        float success = Tools.round((float)(v.bvEntering * v.transmittance / TConstant.EP));

                        filtered_data[filter] = new MiniVoxel(
                                index,
                                (int)(Math.floor((double)v.$i / TConstant.minivox)),
                                (int)(Math.floor((double)v.$j / TConstant.minivox)),
                                (int)(Math.floor((double)v.$k / TConstant.minivox)),
                                trials,
                                success / trials);

                        filter++;
                    }

                    index++;
                }
            }
        }

        int VI = (int)Math.floor(this.split.getX()) + 1;
        int VJ = (int)Math.floor(this.split.getY()) + 1;
        int VK = (int)Math.floor(this.split.getZ()) + 1;

        int[][][] nbIntanceIJK = new int[VI][VJ][VK];

        for (int i = 0; i < ground_distance; i++) {
            int vvi = filtered_data[i].I;
            int vvj = filtered_data[i].J;
            int vvk = filtered_data[i].K;
            nbIntanceIJK[vvi][vvj][vvk]++;
        }

        int size = 0;
        for (int i = 0; i < VI; i++) {
            for (int j = 0; j < VJ; j++) {
                for (int k = 0; k < VK; k++) {
                    if(nbIntanceIJK[i][j][k] > 0) {
                        size++;
                    }
                }
            }
        }

        tempVoxel = new float[size][3];
        size = 0;
        for (int i = 0; i < VI; i++) {
            for (int j = 0; j < VJ; j++) {
                for (int k = 0; k < VK; k++) {
                    if(nbIntanceIJK[i][j][k] > 0) {
                        tempVoxel[size][0] = i;
                        tempVoxel[size][1] = j;
                        tempVoxel[size][2] = k;
                        size++;
                    }
                }
            }
        }

        return this;
    }

    /**
     * Boucle d'appel au code R
     * @return
     */
    public MixModTrans loopR() {

        int[] gd = new int[(int)this.max_ground_distance];

        resultat = new double[this.split.getX()][this.split.getY()][this.split.getZ()][1];

        for (int i = 0; i < split.x; i++) {
            for (int j = 0; j < split.y; j++) {
                for (int k = 0; k < split.z; k++) {

                    int ground = (int) Tools.round(voxels[i][j][k].ground_distance);

                    if (ground > 0) {
                        gd[ground - 1]++;
                    }

                    resultat[i][j][k][0] = Double.NaN;
                }
            }
        }

        for (int i = 0; i < tempVoxel.length; i++) {

            if (i % 100==0) {
                //System.out.println(i);
            }

            int TI = (int)tempVoxel[i][0];
            int TJ = (int)tempVoxel[i][1];
            int TK = (int)tempVoxel[i][2];

            int sizeP = 0;
            int sizeZ = 0;
            int size = 0;

            for(MiniVoxel v : filtered_data) {
                if((TI == v.I) && (TJ == v.J) && (TK == v.K)) {
                    size++;
                    if(v.trials > 0) {
                        sizeP++;
                    } else if(v.trials == 0) {
                        sizeZ++;
                    }
                }
            }

            int sP = 0;
            int sZ = 0;
            int sT = 0;

            String[] ijkT = new String[size];

            String[] ijkP = new String[sizeP];
            double[] propP = new double[sizeP];
            double[] trialsP = new double[sizeP];

            String[] ijkZ = new String[sizeZ];

            for(MiniVoxel v : filtered_data) {
                String ijk = ijkInit[v.ID];
                float VT = v.trials;
                float VP = v.prop;

                if((TI == v.I) && (TJ == v.J) && (TK == v.K)) {
                    ijkT[sT] = ijk;
                    sT++;
                    if(VT > 0) {
                        ijkP[sP] = ijk;
                        propP[sP] = VP;
                        trialsP[sP] = VT;
                        sP++;
                    } else if(VT == 0) {
                        ijkZ[sZ] = ijk;
                        sZ++;
                    }
                }
            }

            if(sizeP > TConstant.seuil_echantillonnage) {

                boolean allOne = true;
                for(double p : propP) {
                    if(p != 1.0) {
                        allOne = false;
                        break;
                    }
                }

                if(allOne) {

                    for(String s : ijkT) {
                        String[] values = s.split("_");
                        int wi = Integer.parseInt(values[0]);
                        int wj = Integer.parseInt(values[1]);
                        int wk = Integer.parseInt(values[2]);
                        resultat[wi][wj][wk][0] = 1.0;
                    }
                }
                else {
                    boolean allZero = true;
                    for(double p : propP) {
                        if(p != 0) {
                            allZero = false;
                            break;
                        }
                    }

                    if(allZero) {
                        //System.out.print("ALLZERO" + tempVoxel.toString());
                    }
                    else {

                        double[] res = R.glme(propP, trialsP, ijkP);

                        int index = 0;

                        for(String s : ijkP) {
                            String[] values = s.split("_");
                            int wi = Integer.parseInt(values[0]);
                            int wj = Integer.parseInt(values[1]);
                            int wk = Integer.parseInt(values[2]);

                            resultat[wi][wj][wk][0] = res[index];
                            index++;
                        }

                        if(sizeZ > 0) {
                            double mean = 0.0;

                            for (double r : res) {
                                mean += r;
                            }
                            mean = (mean / (double)res.length);


                            for(String s : ijkZ) {

                                String[] values = s.split("_");
                                int wi = Integer.parseInt(values[0]);
                                int wj = Integer.parseInt(values[1]);
                                int wk = Integer.parseInt(values[2]);

                                resultat[wi][wj][wk][0] = mean;
                                index++;
                            }
                        }
                    }
                }

            }
        }

        R.endConnection();

        return this;
    }

    /**
     * Post traitement pour avoir la bonne valeur de transmittance et de PadBVTotal
     * @return
     */
    public MixModTrans postprocess() {

        double[][] meadByGD = new double[(int)this.max_ground_distance+1][3];

        for (int i = 0; i < split.x; i++) {
            for (int j = 0; j < split.y; j++) {
                for (int k = 0; k < split.z; k++) {
                    Voxel v = voxels[i][j][k];
                    int ground = (int)Tools.round(v.ground_distance);
                    if (ground > 0) {
                        double pred = resultat[i][j][k][0];

                        if(!Double.isNaN(pred)) {
                            meadByGD[ground-1][0] += pred;
                            meadByGD[ground-1][1]++;
                        }
                    }
                }
            }
         }


        for (int id = 0; id < this.max_ground_distance; id++) {
            meadByGD[id][2] = meadByGD[id][0] / meadByGD[id][1];
        }

        for (int i = 0; i < split.x; i++) {
            for (int j = 0; j < split.y; j++) {
                for (int k = 0; k < split.z; k++) {
                    Voxel v = voxels[i][j][k];
                    int ground = (int) Tools.round(v.ground_distance);
                    if (ground > 0) {
                        double pred = resultat[i][j][k][0];

                        if(Double.isNaN(pred)) {
                            pred = meadByGD[ground-1][2];
                        }

                        voxels[i][j][k].transmittance = (float)pred;

                        if(!Double.isNaN(pred)) {
                            voxels[i][j][k].PadBVTotal = (float) (-2 * Math.log(pred));
                        }

                        if(voxels[i][j][k].transmittance > 0.99) {
                            voxels[i][j][k].transmittance = 1;
                        }
                    }
                    else {
                        voxels[i][j][k].PadBVTotal = 0;
                    }
                }
            }
        }

        return this;
    }
}
