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

package fr.amap.amapvox.voxelisation.configuration;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import javax.vecmath.Point3d;
import javax.vecmath.Point3i;
import org.jdom2.Attribute;
import org.jdom2.Element;

/**
 *
 * @author calcul
 */


public class MultiVoxCfg extends ALSVoxCfg{
    
    private List<Input> multiProcessInputs;
    
    @Override
    public void readConfiguration(File inputParametersFile) throws Exception {
        
        super.readConfiguration(inputParametersFile);
        
        processMode = ProcessMode.MULTI_VOXELISATION_ALS_AND_MULTI_RES;
        
        Element inputsElement = processElement.getChild("inputs");
                        
        if(inputsElement != null){
            List<Element> childrens = inputsElement.getChildren("input");

            if(childrens != null){

                multiProcessInputs = new ArrayList<>();

                int count = 0;

                for(Element child : childrens){

                    Element subVoxelSpaceElement = child.getChild("voxelspace");

                    VoxelParameters multiProcessVoxelParameters = null;

                    if(subVoxelSpaceElement != null){

                        multiProcessVoxelParameters = new VoxelParameters();
                        double resolution = Double.valueOf(subVoxelSpaceElement.getAttributeValue("resolution"));

                        multiProcessVoxelParameters.setBottomCorner(new Point3d(
                                        Double.valueOf(subVoxelSpaceElement.getAttributeValue("xmin")), 
                                        Double.valueOf(subVoxelSpaceElement.getAttributeValue("ymin")), 
                                        Double.valueOf(subVoxelSpaceElement.getAttributeValue("zmin"))));

                        multiProcessVoxelParameters.setTopCorner(new Point3d(
                                            Double.valueOf(subVoxelSpaceElement.getAttributeValue("xmax")), 
                                            Double.valueOf(subVoxelSpaceElement.getAttributeValue("ymax")), 
                                            Double.valueOf(subVoxelSpaceElement.getAttributeValue("zmax"))));

                        multiProcessVoxelParameters.setSplit(new Point3i(
                                            Integer.valueOf(subVoxelSpaceElement.getAttributeValue("splitX")), 
                                            Integer.valueOf(subVoxelSpaceElement.getAttributeValue("splitY")), 
                                            Integer.valueOf(subVoxelSpaceElement.getAttributeValue("splitZ"))));


                        multiProcessVoxelParameters.setResolution(resolution);
                    }

                    File inputFileChild = null;
                    Element inputFileChildElement = child.getChild("input_file");

                    if(inputFileChildElement != null){

                        if(count == 0){
                            inputType = getInputFileType(Integer.valueOf(inputFileChildElement.getAttributeValue("type")));
                        }
                        inputFileChild = new File(inputFileChildElement.getAttributeValue("src"));
                    }

                    File dtmFileChild = null;
                    Element dtmFileElement = child.getChild("dtm-filter");

                    if(dtmFileElement != null){

                        dtmFileChild = new File(dtmFileElement.getAttributeValue("src"));
                    }


                    File outputFileChild = null;
                    Element ouputFileChildElement = child.getChild("output_file");

                    if(ouputFileChildElement != null){

                        outputFileChild = new File(ouputFileChildElement.getAttributeValue("src"));
                    }

                    List<Input> multiResInputs = null;
                    Element multiResElement = child.getChild("multi-res");
                    File multiResOutputFile = null;

                    if(multiResElement != null){

                        boolean multiResEnabled = Boolean.valueOf(multiResElement.getAttributeValue("enabled"));

                        if(multiResEnabled){
                            multiResInputs = new ArrayList<>();
                            List<Element> multiResInputsElement = multiResElement.getChild("inputs").getChildren("input");

                            for(Element element : multiResInputsElement){

                                Element subSubVoxelSpaceElement = element.getChild("voxelspace");

                                VoxelParameters subVoxelParameters = null;

                                if(subVoxelSpaceElement != null){
                                    int splitX = Integer.valueOf(subSubVoxelSpaceElement.getAttributeValue("splitX"));
                                    int splitY = Integer.valueOf(subSubVoxelSpaceElement.getAttributeValue("splitY"));
                                    int splitZ = Integer.valueOf(subSubVoxelSpaceElement.getAttributeValue("splitZ"));
                                    double resolution = Double.valueOf(subSubVoxelSpaceElement.getAttributeValue("resolution"));

                                    subVoxelParameters = new VoxelParameters(null, null, new Point3i(splitX, splitY, splitZ));
                                    subVoxelParameters.setResolution(resolution);
                                }

                                Element subSubOutputFileElement = element.getChild("output-file");

                                File subOutputFile = null;
                                if(subSubOutputFileElement != null){
                                    subOutputFile = new File(subSubOutputFileElement.getAttributeValue("src"));
                                }

                                multiResInputs.add(new Input(subVoxelParameters, null, null, subOutputFile, null, null));
                            }

                            Element multiResOutputFileElement = multiResElement.getChild("output-file");

                            if(multiResOutputFileElement != null){
                                multiResOutputFile = new File(multiResOutputFileElement.getAttributeValue("src"));
                            }
                        }
                    }

                    multiProcessInputs.add(new Input(multiProcessVoxelParameters, inputFileChild, dtmFileChild, outputFileChild, multiResInputs, multiResOutputFile));

                }

                count++;
            }

        }
    }
    
    @Override
    public void writeConfiguration(File outputParametersFile) throws Exception {
        
        createCommonData();
        
        super.writeConfiguration(outputParametersFile);
        
        processElement.setAttribute(new Attribute("mode","multi-voxelisation"));
        processElement.setAttribute(new Attribute("type","ALS"));

        if(multiResPadMax != null){
            limitsElement.addContent(createLimitElement("PAD_1m", "", String.valueOf(multiResPadMax[0])));
            limitsElement.addContent(createLimitElement("PAD_2m", "", String.valueOf(multiResPadMax[1])));
            limitsElement.addContent(createLimitElement("PAD_3m", "", String.valueOf(multiResPadMax[2])));
            limitsElement.addContent(createLimitElement("PAD_4m", "", String.valueOf(multiResPadMax[3])));
            limitsElement.addContent(createLimitElement("PAD_5m", "", String.valueOf(multiResPadMax[4])));
        }

        Element inputsElement = new Element("inputs");

        if(multiProcessInputs != null){

            for(Input input : multiProcessInputs){

                Element inputElement = new Element("input");

                if(input.voxelParameters != null){
                    Element voxelSpaceElement = new Element("voxelspace");
                    voxelSpaceElement.setAttribute("xmin", String.valueOf(input.voxelParameters.bottomCorner.x));
                    voxelSpaceElement.setAttribute("ymin", String.valueOf(input.voxelParameters.bottomCorner.y));
                    voxelSpaceElement.setAttribute("zmin", String.valueOf(input.voxelParameters.bottomCorner.z));
                    voxelSpaceElement.setAttribute("xmax", String.valueOf(input.voxelParameters.topCorner.x));
                    voxelSpaceElement.setAttribute("ymax", String.valueOf(input.voxelParameters.topCorner.y));
                    voxelSpaceElement.setAttribute("zmax", String.valueOf(input.voxelParameters.topCorner.z));
                    voxelSpaceElement.setAttribute("splitX", String.valueOf(input.voxelParameters.split.x));
                    voxelSpaceElement.setAttribute("splitY", String.valueOf(input.voxelParameters.split.y));
                    voxelSpaceElement.setAttribute("splitZ", String.valueOf(input.voxelParameters.split.z));
                    voxelSpaceElement.setAttribute("resolution", String.valueOf(input.voxelParameters.resolution));
                    inputElement.addContent(voxelSpaceElement);
                }
                if(input.inputFile != null){
                    
                    Element inputFileElement = new Element("input_file");
                    inputFileElement.setAttribute(new Attribute("type", String.valueOf(inputType.type)));
                    inputFileElement.setAttribute(new Attribute("src",input.inputFile.getAbsolutePath()));
                    inputElement.addContent(inputFileElement);

                }

                if(input.dtmFile != null){
                    Element dtmFileElement = new Element("dtm-filter");
                    dtmFileElement.setAttribute(new Attribute("src",input.dtmFile.getAbsolutePath()));
                    inputElement.addContent(dtmFileElement);
                }

                if(input.outputFile != null){
                    Element outputFileElement = new Element("output_file");
                    outputFileElement.setAttribute(new Attribute("src",input.outputFile.getAbsolutePath()));
                    inputElement.addContent(outputFileElement);
                }

                if(input.voxelParameters.getGroundEnergyFile() != null){
                    Element groundEnergyElement = new Element("ground-energy");
                    groundEnergyElement.setAttribute("generate", String.valueOf(input.voxelParameters.isCalculateGroundEnergy()));

                    if(input.voxelParameters.getGroundEnergyFile() != null){
                        groundEnergyElement.setAttribute("src", input.voxelParameters.getGroundEnergyFile().getAbsolutePath());
                        groundEnergyElement.setAttribute("type", String.valueOf(input.voxelParameters.getGroundEnergyFileFormat()));
                    }
                    inputElement.addContent(groundEnergyElement);
                }

                if(input.multiResList != null){

                    Element multiResElement = new Element("multi-res");
                    multiResElement.setAttribute("enabled", String.valueOf(correctNaNs));

                    if(correctNaNs){

                        Element multiResInputsElement = new Element("inputs");

                        for(Input subInput : input.multiResList){

                            Element multiResInputElement = new Element("input");
                            Element subVoxelSpaceElement = new Element("voxelspace");
                            subVoxelSpaceElement.setAttribute("splitX", String.valueOf(subInput.voxelParameters.split.x));
                            subVoxelSpaceElement.setAttribute("splitY", String.valueOf(subInput.voxelParameters.split.y));
                            subVoxelSpaceElement.setAttribute("splitZ", String.valueOf(subInput.voxelParameters.split.z));
                            subVoxelSpaceElement.setAttribute("resolution", String.valueOf(subInput.voxelParameters.resolution));
                            multiResInputElement.addContent(subVoxelSpaceElement);

                            Element subOutputFileElement = new Element("output-file");
                            subOutputFileElement.setAttribute(new Attribute("src",subInput.outputFile.getAbsolutePath()));
                            multiResInputElement.addContent(subOutputFileElement);

                            multiResInputsElement.addContent(multiResInputElement);

                        }

                        multiResElement.addContent(multiResInputsElement);

                        if(input.outputFileMultiRes != null){
                            multiResElement.addContent(new Element("output-file").setAttribute("src", input.outputFileMultiRes.getAbsolutePath()));
                        }
                    }


                    inputElement.addContent(multiResElement);
                }

                inputsElement.addContent(inputElement);
            }
        }

        processElement.addContent(inputsElement);
        
        writeDocument(outputParametersFile);
    }

    public List<Input> getMultiProcessInputs() {
        return multiProcessInputs;
    }

    public void setMultiProcessInputs(List<Input> multiProcessInputs) {
        this.multiProcessInputs = multiProcessInputs;
    }
    
}
