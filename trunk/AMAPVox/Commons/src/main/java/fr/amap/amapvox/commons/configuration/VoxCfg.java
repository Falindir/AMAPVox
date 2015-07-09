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

package fr.amap.amapvox.commons.configuration;

import fr.amap.amapvox.commons.util.Filter;
import fr.amap.amapvox.commons.util.PointcloudFilter;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3i;
import org.apache.log4j.Logger;
import org.jdom2.Attribute;
import org.jdom2.Element;

/**
 *
 * @author calcul
 */


public class VoxCfg extends Configuration{

    private final static Logger logger = Logger.getLogger(VoxCfg.class);
    
    protected File inputFile;
    protected File outputFile;
    protected boolean usePopMatrix;
    protected boolean useSopMatrix;
    protected boolean useVopMatrix;
    protected Matrix4d popMatrix;
    protected Matrix4d sopMatrix;
    protected Matrix4d vopMatrix;
    protected VoxelParameters voxelParameters;
    protected float[] multiResPadMax;
    protected List<Filter> filters;
    protected boolean correctNaNs;
    protected boolean multiResUseDefaultMaxPad;
    
    protected Element limitsElement;
    protected Element filtersElement;
    
    @Override
    public void readConfiguration(File inputParametersFile) throws Exception {
        
        initDocument(inputParametersFile);
        
        voxelParameters = new VoxelParameters();
        
        Element inputFileElement = processElement.getChild("input_file");

        if(inputFileElement != null){
            int inputTypeInteger = Integer.valueOf(inputFileElement.getAttributeValue("type"));
            inputType = getInputFileType(inputTypeInteger);
            inputFile = new File(inputFileElement.getAttributeValue("src"));
        }else{
            logger.warn("Cannot find input_file element");
        }
        
        Element outputFileElement = processElement.getChild("output_file");
        if(outputFileElement != null){
            outputFile = new File(outputFileElement.getAttributeValue("src"));
        }else{
            logger.warn("Cannot find output_file element");
        }

        Element voxelSpaceElement = processElement.getChild("voxelspace");

        if(voxelSpaceElement != null){
            voxelParameters.setBottomCorner(new Point3d(
                        Double.valueOf(voxelSpaceElement.getAttributeValue("xmin")), 
                        Double.valueOf(voxelSpaceElement.getAttributeValue("ymin")), 
                        Double.valueOf(voxelSpaceElement.getAttributeValue("zmin"))));

            voxelParameters.setTopCorner(new Point3d(
                                Double.valueOf(voxelSpaceElement.getAttributeValue("xmax")), 
                                Double.valueOf(voxelSpaceElement.getAttributeValue("ymax")), 
                                Double.valueOf(voxelSpaceElement.getAttributeValue("zmax"))));

            voxelParameters.setSplit(new Point3i(
                                Integer.valueOf(voxelSpaceElement.getAttributeValue("splitX")), 
                                Integer.valueOf(voxelSpaceElement.getAttributeValue("splitY")), 
                                Integer.valueOf(voxelSpaceElement.getAttributeValue("splitZ"))));

            try{
                voxelParameters.setResolution(Double.valueOf(voxelSpaceElement.getAttributeValue("resolution")));
            }catch(Exception e){}
            
        }else{
            //logger.info("Cannot find bounding-box element");
        }
        
        
        Element ponderationElement = processElement.getChild("ponderation");
                    
        if(ponderationElement != null){

            voxelParameters.setWeighting(Integer.valueOf(ponderationElement.getAttributeValue("mode")));

            if(voxelParameters.getWeighting() > 0){
                Element matrixElement = ponderationElement.getChild("matrix");
                int rowNumber = 7;
                int colNumber = 7;
                String data = matrixElement.getText();
                String[] datas = data.split(" ");
                float[][] weightingData = new float[rowNumber][colNumber];

                int count = 0;
                for(int i=0;i<weightingData.length;i++){
                    for(int j=0;j<weightingData[0].length;j++){
                        weightingData[i][j] = Float.valueOf(datas[count]);
                        count++;
                    }
                }

                voxelParameters.setWeightingData(weightingData);
            }


        }

        Element dtmFilterElement = processElement.getChild("dtm-filter");

        if(dtmFilterElement != null){
            boolean useDTM = Boolean.valueOf(dtmFilterElement.getAttributeValue("enabled"));
            voxelParameters.setUseDTMCorrection(useDTM);
            if(useDTM){
                voxelParameters.setDtmFile(new File(dtmFilterElement.getAttributeValue("src")));
                voxelParameters.minDTMDistance = Float.valueOf(dtmFilterElement.getAttributeValue("height-min"));
            }                        
        }

        Element pointcloudFiltersElement = processElement.getChild("pointcloud-filters");

        if(pointcloudFiltersElement != null){
            boolean usePointCloudFilter = Boolean.valueOf(pointcloudFiltersElement.getAttributeValue("enabled"));
            voxelParameters.setUsePointCloudFilter(usePointCloudFilter);
            if(usePointCloudFilter){

                List<Element> childrens = pointcloudFiltersElement.getChildren("pointcloud-filter");

                if(childrens != null){
                    List<PointcloudFilter> pointcloudFilters = new ArrayList<>();
                    for(Element e : childrens){

                        boolean keep;
                        String operationType = e.getAttributeValue("operation-type");
                        if(operationType.equals("Keep")){
                            keep = true;
                        }else{
                            keep = false;
                        }
                        pointcloudFilters.add(new PointcloudFilter(new File(e.getAttributeValue("src")), 
                                Float.valueOf(e.getAttributeValue("error-margin")), keep));
                    }

                    voxelParameters.setPointcloudFilters(pointcloudFilters);

                }
            }                        
        }

        Element transformationElement = processElement.getChild("transformation");

        if(transformationElement != null){
            usePopMatrix = Boolean.valueOf(transformationElement.getAttributeValue("use-pop"));
            useSopMatrix = Boolean.valueOf(transformationElement.getAttributeValue("use-sop"));
            useVopMatrix = Boolean.valueOf(transformationElement.getAttributeValue("use-vop"));
            List<Element> matrixList = transformationElement.getChildren("matrix");
            for(Element e : matrixList){

                String matrixType = e.getAttributeValue("type_id");
                Matrix4d mat = getMatrixFromData(e.getText());



                switch(matrixType){
                    case "pop":
                        popMatrix = mat;
                        break;
                    case "sop":
                        sopMatrix = mat;
                        break;
                    case "vop":
                        vopMatrix = mat;
                        break;
                }
            }
        }

        limitsElement = processElement.getChild("limits");

        if(limitsElement != null){
            List<Element> limitChildrensElement = limitsElement.getChildren("limit");

            if(limitChildrensElement != null){

                if(limitChildrensElement.size() > 0){
                    voxelParameters.setMaxPAD(Float.valueOf(limitChildrensElement.get(0).getAttributeValue("max")));
                }

                if(limitChildrensElement.size() >= 6){
                    multiResPadMax = new float[5];
                    multiResPadMax[0] = Float.valueOf(limitChildrensElement.get(1).getAttributeValue("max"));
                    multiResPadMax[1] = Float.valueOf(limitChildrensElement.get(2).getAttributeValue("max"));
                    multiResPadMax[2] = Float.valueOf(limitChildrensElement.get(3).getAttributeValue("max"));
                    multiResPadMax[3] = Float.valueOf(limitChildrensElement.get(4).getAttributeValue("max"));
                    multiResPadMax[4] = Float.valueOf(limitChildrensElement.get(5).getAttributeValue("max"));

                }
            }

        }


        filtersElement = processElement.getChild("filters");
        filters = new ArrayList<>();

        if(filtersElement != null){

            Element shotFiltersElement = filtersElement.getChild("shot-filters");

            if(shotFiltersElement != null){                            

                List<Element> childrensFilter = shotFiltersElement.getChildren("filter");

                if(childrensFilter != null){



                    for(Element e : childrensFilter){

                        String variable = e.getAttributeValue("variable");
                        String inequality = e.getAttributeValue("inequality");
                        String value = e.getAttributeValue("value");

                        filters.add(new Filter(variable, Float.valueOf(value), Filter.getConditionFromString(inequality)));
                    }
                }
            }

            
            
            
            Element generateMultiBandRasterElement = processElement.getChild("multi-band-raster");
                
            if(generateMultiBandRasterElement != null){

                boolean generateMultiBandRaster = Boolean.valueOf(generateMultiBandRasterElement.getAttributeValue("generate"));
                voxelParameters.setGenerateMultiBandRaster(generateMultiBandRaster);

                if(generateMultiBandRaster){
                    voxelParameters.setShortcutVoxelFileWriting(Boolean.valueOf(generateMultiBandRasterElement.getAttributeValue("discard_voxel_file_writing")));
                    voxelParameters.setRasterStartingHeight(Float.valueOf(generateMultiBandRasterElement.getAttributeValue("starting-height")));
                    voxelParameters.setRasterHeightStep(Float.valueOf(generateMultiBandRasterElement.getAttributeValue("step")));
                    voxelParameters.setRasterBandNumber(Integer.valueOf(generateMultiBandRasterElement.getAttributeValue("band-number")));
                    voxelParameters.setRasterResolution(Integer.valueOf(generateMultiBandRasterElement.getAttributeValue("resolution")));
                }
            }
            
            Element formulaElement = processElement.getChild("formula");
                    
            if(formulaElement != null){
                Element transmittanceElement = formulaElement.getChild("transmittance");
                if(transmittanceElement != null){
                    voxelParameters.setTransmittanceMode(Integer.valueOf(transmittanceElement.getAttributeValue("mode")));
                }
            }
        }  
    }

    @Override
    public void writeConfiguration(File outputParametersFile) throws Exception{
        
        if(inputFile != null){
            Element inputFileElement = new Element("input_file");
            inputFileElement.setAttribute(new Attribute("type", String.valueOf(inputType.type)));
            inputFileElement.setAttribute(new Attribute("src",inputFile.getAbsolutePath()));
            processElement.addContent(inputFileElement);
        }else{
            //logger.info("Global input file ignored.");
        }
        

        Element outputFileElement = new Element("output_file");
        outputFileElement.setAttribute(new Attribute("src",outputFile.getAbsolutePath()));
        processElement.addContent(outputFileElement);
        
        if(voxelParameters != null){
            Element voxelSpaceElement = new Element("voxelspace");
            voxelSpaceElement.setAttribute("xmin", String.valueOf(voxelParameters.bottomCorner.x));
            voxelSpaceElement.setAttribute("ymin", String.valueOf(voxelParameters.bottomCorner.y));
            voxelSpaceElement.setAttribute("zmin", String.valueOf(voxelParameters.bottomCorner.z));
            voxelSpaceElement.setAttribute("xmax", String.valueOf(voxelParameters.topCorner.x));
            voxelSpaceElement.setAttribute("ymax", String.valueOf(voxelParameters.topCorner.y));
            voxelSpaceElement.setAttribute("zmax", String.valueOf(voxelParameters.topCorner.z));
            voxelSpaceElement.setAttribute("splitX", String.valueOf(voxelParameters.split.x));
            voxelSpaceElement.setAttribute("splitY", String.valueOf(voxelParameters.split.y));
            voxelSpaceElement.setAttribute("splitZ", String.valueOf(voxelParameters.split.z));
            voxelSpaceElement.setAttribute("resolution", String.valueOf(voxelParameters.resolution));
            processElement.addContent(voxelSpaceElement);
            
        }else{
            //logger.info("Global bounding-box ignored.");
        }
        
        
        /***PONDERATION***/
        Element ponderationElement = new Element("ponderation");
        ponderationElement.setAttribute(new Attribute("mode",String.valueOf(voxelParameters.getWeighting())));

        if(voxelParameters.getWeighting() > 0){
            StringBuilder weightingDataString = new StringBuilder();
            float[][] weightingData = voxelParameters.getWeightingData();
            for(int i=0;i<weightingData.length;i++){
                for(int j=0;j<weightingData[0].length;j++){
                    weightingDataString.append(weightingData[i][j]).append(" ");
                }
            }
            Element matrixElement = createMatrixElement("ponderation", weightingDataString.toString().trim());
            ponderationElement.addContent(matrixElement);
        }

        processElement.addContent(ponderationElement);

        /***DTM FILTER***/

        Element dtmFilterElement = new Element("dtm-filter");
        dtmFilterElement.setAttribute(new Attribute("enabled",String.valueOf(voxelParameters.useDTMCorrection())));
        if(voxelParameters.useDTMCorrection()){
            if(voxelParameters.getDtmFile() != null){
                dtmFilterElement.setAttribute(new Attribute("src", voxelParameters.getDtmFile().getAbsolutePath()));
            }

            dtmFilterElement.setAttribute(new Attribute("height-min",String.valueOf(voxelParameters.minDTMDistance)));
        }

        processElement.addContent(dtmFilterElement);

        Element pointcloudFiltersElement = new Element("pointcloud-filters");
        pointcloudFiltersElement.setAttribute(new Attribute("enabled",String.valueOf(voxelParameters.isUsePointCloudFilter())));

        if(voxelParameters.isUsePointCloudFilter()){

            List<PointcloudFilter> pointcloudFilters = voxelParameters.getPointcloudFilters();

            if(pointcloudFilters != null){

                for(PointcloudFilter filter: pointcloudFilters){
                    Element pointcloudFilterElement = new Element("pointcloud-filter");
                    pointcloudFilterElement.setAttribute(new Attribute("src", filter.getPointcloudFile().getAbsolutePath()));
                    pointcloudFilterElement.setAttribute(new Attribute("error-margin",String.valueOf(filter.getPointcloudErrorMargin())));

                    String operationType;
                    if(filter.isKeep()){
                        operationType = "Keep";
                    }else{
                        operationType = "Discard";
                    }

                    pointcloudFilterElement.setAttribute(new Attribute("operation-type",operationType));
                    pointcloudFiltersElement.addContent(pointcloudFilterElement);
                }
            }

        }

        processElement.addContent(pointcloudFiltersElement);

        /***TRANSFORMATION***/

        Element transformationElement = new Element("transformation");


        transformationElement.setAttribute(new Attribute("use-pop",String.valueOf(usePopMatrix)));
        transformationElement.setAttribute(new Attribute("use-sop",String.valueOf(useSopMatrix)));
        transformationElement.setAttribute(new Attribute("use-vop",String.valueOf(useVopMatrix)));

        if(usePopMatrix && popMatrix != null){
            Element matrixPopElement = createMatrixElement("pop", popMatrix.toString());
            transformationElement.addContent(matrixPopElement);
        }

        if(useSopMatrix && sopMatrix != null){
            Element matrixSopElement = createMatrixElement("sop", sopMatrix.toString());
            transformationElement.addContent(matrixSopElement);
        }

        if(useVopMatrix && vopMatrix != null){
            Element matrixVopElement = createMatrixElement("vop", vopMatrix.toString());
            transformationElement.addContent(matrixVopElement);
        }

        processElement.addContent(transformationElement);

        /***LIMITS***/

        limitsElement = new Element("limits");
        Element limitElement = new Element("limit");
        limitElement.setAttribute("name", "PAD");
        limitElement.setAttribute("min", "");
        limitElement.setAttribute("max", String.valueOf(voxelParameters.getMaxPAD()));
        limitsElement.addContent(limitElement);

        processElement.addContent(limitsElement);

        filtersElement = new Element("filters");

        if(filters != null && !filters.isEmpty()){

            Element shotFilterElement = new Element("shot-filters");

            for(Filter f : filters){
                Element filterElement = new Element("filter");
                filterElement.setAttribute("variable", f.getVariable());
                filterElement.setAttribute("inequality", f.getConditionString());
                filterElement.setAttribute("value", String.valueOf(f.getValue()));
                shotFilterElement.addContent(filterElement);
            }

            filtersElement.addContent(shotFilterElement);
        }

        


        processElement.addContent(filtersElement);
        
        if(voxelParameters.isGenerateMultiBandRaster()){
                
            Element generateMultiBandRasterElement = new Element("multi-band-raster");
            generateMultiBandRasterElement.setAttribute("generate", String.valueOf(voxelParameters.isGenerateMultiBandRaster()));
            generateMultiBandRasterElement.setAttribute("discard_voxel_file_writing", String.valueOf(voxelParameters.isShortcutVoxelFileWriting()));

            generateMultiBandRasterElement.setAttribute("starting-height", String.valueOf(voxelParameters.getRasterStartingHeight()));
            generateMultiBandRasterElement.setAttribute("step", String.valueOf(voxelParameters.getRasterHeightStep()));
            generateMultiBandRasterElement.setAttribute("band-number", String.valueOf(voxelParameters.getRasterBandNumber()));
            generateMultiBandRasterElement.setAttribute("resolution", String.valueOf(voxelParameters.getRasterResolution()));

            processElement.addContent(generateMultiBandRasterElement);
        }
    }
    
    public InputType getInputType() {
        return inputType;
    }

    public void setInputType(InputType inputType) {
        this.inputType = inputType;
    }

    public File getInputFile() {
        return inputFile;
    }

    public void setInputFile(File inputFile) {
        this.inputFile = inputFile;
    }

    public File getOutputFile() {
        return outputFile;
    }

    public void setOutputFile(File outputFile) {
        this.outputFile = outputFile;
    }

    public VoxelParameters getVoxelParameters() {
        return voxelParameters;
    }

    public void setVoxelParameters(VoxelParameters voxelParameters) {
        this.voxelParameters = voxelParameters;
    }

    public boolean isUsePopMatrix() {
        return usePopMatrix;
    }

    public void setUsePopMatrix(boolean usePopMatrix) {
        this.usePopMatrix = usePopMatrix;
    }

    public boolean isUseSopMatrix() {
        return useSopMatrix;
    }

    public void setUseSopMatrix(boolean useSopMatrix) {
        this.useSopMatrix = useSopMatrix;
    }

    public boolean isUseVopMatrix() {
        return useVopMatrix;
    }

    public void setUseVopMatrix(boolean useVopMatrix) {
        this.useVopMatrix = useVopMatrix;
    }

    public Matrix4d getPopMatrix() {
        return popMatrix;
    }

    public void setPopMatrix(Matrix4d popMatrix) {
        this.popMatrix = popMatrix;
    }

    public Matrix4d getSopMatrix() {
        return sopMatrix;
    }

    public void setSopMatrix(Matrix4d sopMatrix) {
        this.sopMatrix = sopMatrix;
    }

    public Matrix4d getVopMatrix() {
        return vopMatrix;
    }

    public void setVopMatrix(Matrix4d vopMatrix) {
        this.vopMatrix = vopMatrix;
    }

    public List<Filter> getFilters() {
        return filters;
    }

    public void setFilters(List<Filter> filters) {
        this.filters = filters;
    }

    public float[] getMultiResPadMax() {
        return multiResPadMax;
    }

    public void setMultiResPadMax(float[] multiResPadMax) {
        this.multiResPadMax = multiResPadMax;
    }    
    
}