#include "scatteringNodeCmd.h"

// Includes needed for implementation (not in header)
#include <maya/MArgDatabase.h>    // Used in doIt()
#include <maya/MArgList.h>         // Used in doIt() signature
#include <maya/MDagModifier.h>     // Used in implementation
#include <maya/MDagPath.h>         // Used in getShapeNode()
#include <maya/MFnDagNode.h>       // Used throughout implementation
#include <maya/MFnDependencyNode.h> // Used throughout implementation
#include <maya/MGlobal.h>          // Used for executeCommand, displayInfo
#include <maya/MObject.h>          // Used in function signatures
#include <maya/MSelectionList.h>   // Used in implementation

#include <boost/algorithm/string.hpp>

// Command name constant
const char* ScatteringNodeCmd::kName = "scatteringNodeCmd";

/**
 * Constructor: Initializes the command with default values.
 */
ScatteringNodeCmd::ScatteringNodeCmd()
    : m_randomPos(0.0)
{
}

/**
 * Creator function: Called by Maya to create a new instance of this command.
 * @return Pointer to a new ScatteringNodeCmd instance.
 */
void* ScatteringNodeCmd::creator()
{
    return new ScatteringNodeCmd;
}

/**
 * Defines the command syntax (flags and arguments).
 * Sets up the command flags: -n (name), -fp (filePath), -rd (randomPos),
 * and allows selection list input for meshes.
 * @return MSyntax object defining the command's argument structure.
 */
MSyntax ScatteringNodeCmd::newSyntax()
{
    MSyntax syntax;
    
    // Command flags
    syntax.addFlag("-n", "-name", MSyntax::kString);
    syntax.addFlag("-fp", "-filePath", MSyntax::kString);
    syntax.addFlag("-rd", "-randomPos", MSyntax::kDouble);
    
    // Allow selection list as input (meshes to scatter on)
    syntax.setObjectType(MSyntax::kSelectionList);
    syntax.useSelectionAsDefault(true);
    
    return syntax;
}

/**
 * Main command execution: Creates the scattering setup.
 * Parses arguments, creates emitter/particle/mesh nodes, connects them,
 * and configures all settings. This is the primary entry point when the
 * command is executed.
 * @param argList Command arguments from Maya.
 * @return MStatus indicating success or failure.
 */
MStatus ScatteringNodeCmd::doIt(const MArgList& argList)
{
    MStatus status;

    // Parse command arguments
    MArgDatabase argData(syntax(), argList, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Get selected meshes
    MSelectionList objects;
    status = argData.getObjects(objects);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    m_pathGeoList.clear();
    m_meshTransformNameList.clear();
    
    for (unsigned int i = 0; i < objects.length(); ++i)
    {
        MDagPath pathGeo;
        status = objects.getDagPath(i, pathGeo);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        
        status = getShapeNode(pathGeo);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        
        m_pathGeoList.append(pathGeo);

        // Extract transform name for matrix connections
        MString meshFullPathName = pathGeo.fullPathName();
        MStringArray pathComponents;
        meshFullPathName.split('|', pathComponents);
        
        if (pathComponents.length() >= 2)
        {
            MString meshTransformName = pathComponents[pathComponents.length() - 2];
            m_meshTransformNameList.append(meshTransformName);
        }
    }

    // Get node name prefix (if specified)
    if (argData.isFlagSet("-n"))
    {
        m_name = argData.flagArgumentString("-n", 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        m_nameEmitter = m_name + "Emitter";
        m_nameParticle = m_name + "Particle";
        m_nameMesh = m_name + "Mesh";
    }
    else
    {
        m_nameEmitter = "scatteringEmitter#";
        m_nameParticle = "scatteringParticle#";
        m_nameMesh = "scatteringMesh#";
    }

    // Get file path (if specified)
    if (argData.isFlagSet("-fp"))
    {
        m_filePath = argData.flagArgumentString("-fp", 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    // Get random position value (if specified)
    if (argData.isFlagSet("-rd"))
    {
        m_randomPos = argData.flagArgumentDouble("-rd", 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    else
    {
        m_randomPos = 0.0;
    }

    // Create the scattering emitter node
    MString command = "createNode(\"scatteringNode\");";
    MString emitterResult;
    status = MGlobal::executeCommand(command, emitterResult, false, true);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Create the particle system
    command = "createNode(\"particle\");";
    MString particleResult;
    status = MGlobal::executeCommand(command, particleResult, false, true);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Create the display mesh
    command = "createNode(\"mesh\");";
    MString meshResult;
    status = MGlobal::executeCommand(command, meshResult, false, true);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Get MObjects for the created nodes
    MSelectionList nodeList;
    nodeList.add(emitterResult);
    nodeList.add(particleResult);
    nodeList.add(meshResult);
    
    MObject emitterObj;
    MObject particleObj;
    MObject meshObj;
    
    status = nodeList.getDependNode(0, emitterObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = nodeList.getDependNode(1, particleObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = nodeList.getDependNode(2, meshObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Store created nodes for undo support
    m_createdNodes.append(emitterObj);
    m_createdNodes.append(particleObj);
    m_createdNodes.append(meshObj);

    // Connect all nodes together
    status = connectAll(emitterObj, particleObj, meshObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Hide unused attributes from channel box
    status = hideUnusedAttributes(emitterObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Add custom attributes to particle system
    status = addAttributesToParticleSys(particleObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Set random position values on emitter (if specified)
    MFnDependencyNode fnEmitterNode(emitterObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    MPlug randomPosXPlug = fnEmitterNode.findPlug("randomPosX", false, &status);
    if (status == MS::kSuccess)
    {
        randomPosXPlug.setDouble(m_randomPos);
    }
    
    MPlug randomPosYPlug = fnEmitterNode.findPlug("randomPosY", false, &status);
    if (status == MS::kSuccess)
    {
        randomPosYPlug.setDouble(m_randomPos);
    }
    
    MPlug randomPosZPlug = fnEmitterNode.findPlug("randomPosZ", false, &status);
    if (status == MS::kSuccess)
    {
        randomPosZPlug.setDouble(m_randomPos);
    }

    // Set default particle point size
    MFnDependencyNode fnParticleNode(particleObj, &status);
    if (status == MS::kSuccess)
    {
        MPlug pointSizePlug = fnParticleNode.findPlug("pointSize", false, &status);
        if (status == MS::kSuccess)
        {
            pointSizePlug.setInt(3);
        }
    }

    // Rename nodes
    MDagModifier dagMod;
    status = dagMod.renameNode(emitterObj, m_nameEmitter);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    MFnDagNode dagNodeParticle(particleObj);
    status = dagMod.renameNode(dagNodeParticle.parent(0), m_nameParticle);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    MFnDagNode dagNodeMesh(meshObj);
    status = dagMod.renameNode(dagNodeMesh.parent(0), m_nameMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    status = dagMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Select the emitter node
    MGlobal::select(emitterObj, MGlobal::kReplaceList);

    // Return names of created objects
    MStringArray nodeNameList;
    MFnDagNode dagNode(emitterObj);
    nodeNameList.append(dagNode.partialPathName());
    
    dagNode.setObject(particleObj);
    nodeNameList.append(dagNode.partialPathName());
    
    dagNode.setObject(meshObj);
    nodeNameList.append(dagNode.partialPathName());

    setResult(nodeNameList);

    // Reset timeline to frame 0
    command = "currentTime 0";
    MGlobal::executeCommand(command, false, true);

    return MS::kSuccess;
}

/**
 * Connects all nodes together and sets up the dependency graph.
 * Connects emitter to particle system, disconnects default time connection,
 * connects emitter's ParticleDriver to particle, connects input meshes to emitter,
 * connects output mesh, and sets file path if specified.
 * @param emitterObj The scattering emitter node.
 * @param particleObj The particle system node.
 * @param meshObj The display mesh node.
 * @return MStatus indicating success or failure.
 */
MStatus ScatteringNodeCmd::connectAll(MObject emitterObj, MObject particleObj, MObject meshObj)
{
    MStatus status;
    
    MFnDagNode particleDag(particleObj);
    MFnDagNode emitterDag(emitterObj);
    MFnDagNode meshDag(meshObj);

    // Connect emitter to particle system using connectDynamic
    MString command = "connectDynamic -em " + emitterDag.fullPathName() + " " + particleDag.fullPathName();
    status = MGlobal::executeCommand(command, false, true);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Get particle's currentTime plug
    MDagModifier dagMod;
    MFnDependencyNode fnParticleNode(particleObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    MPlug particlePlugInTime = fnParticleNode.findPlug("currentTime", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Disconnect default time connection from particle
    MSelectionList timeList;
    status = MGlobal::getSelectionListByName("time1", timeList);
    if (status == MS::kSuccess)
    {
        MObject timeObj;
        timeList.getDependNode(0, timeObj);
        MFnDependencyNode fnTime(timeObj, &status);
        if (status == MS::kSuccess)
        {
            MPlug plugOutTime = fnTime.findPlug("outTime", false, &status);
            if (status == MS::kSuccess)
            {
                // Disconnect the default time connection
                status = dagMod.disconnect(plugOutTime, particlePlugInTime);
                // Note: disconnect may fail if not connected, which is fine
            }
        }
    }

    // Connect emitter's ParticleDriver to particle's currentTime
    MFnDependencyNode fnEmitterNode(emitterObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    MPlug emitterPlugParticleDriver = fnEmitterNode.findPlug("ParticleDriver", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    status = dagMod.connect(emitterPlugParticleDriver, particlePlugInTime);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    status = dagMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Get emitter input attributes
    MObject inGeoAttr = fnEmitterNode.attribute("InputGeo");
    MObject inWorldMatrixAttr = fnEmitterNode.attribute("InputGeoMatrix");

    // Connect selected meshes to emitter
    MDagModifier dagModConnections;
    if (m_pathGeoList.length() > 0)
    {
        for (unsigned int i = 0; i < m_pathGeoList.length(); ++i)
        {
            // Connect mesh outMesh to emitter InputGeo
            MString sourcePlug = m_pathGeoList[i].fullPathName() + ".outMesh";
            MString destPlug = emitterDag.fullPathName() + ".InputGeo[" + MString() + (int)i + "]";
            command = "connectAttr -f " + sourcePlug + " " + destPlug;
            status = MGlobal::executeCommand(command, false, true);
            if (status != MS::kSuccess)
            {
                MGlobal::displayWarning("Failed to connect mesh to emitter InputGeo");
            }

            // Connect transform worldMatrix to emitter InputGeoMatrix
            if (i < m_meshTransformNameList.length())
            {
                MSelectionList transformList;
                status = transformList.add(m_meshTransformNameList[i]);
                if (status == MS::kSuccess)
                {
                    MObject transformObj;
                    status = transformList.getDependNode(0, transformObj);
                    if (status == MS::kSuccess)
                    {
                        sourcePlug = m_pathGeoList[i].fullPathName() + ".worldMatrix[0]";
                        destPlug = emitterDag.fullPathName() + ".InputGeoMatrix[" + MString() + (int)i + "]";
                        command = "connectAttr -f " + sourcePlug + " " + destPlug;
                        status = MGlobal::executeCommand(command, false, true);
                        if (status != MS::kSuccess)
                        {
                            MGlobal::displayWarning("Failed to connect transform matrix to emitter");
                        }
                    }
                }
            }
        }
    }

    // Add mesh to default shading group
    command = "sets -addElement initialShadingGroup " + meshDag.fullPathName();
    MString setResult;
    status = MGlobal::executeCommand(command, setResult, false, true);
    if (status != MS::kSuccess)
    {
        MGlobal::displayWarning("Failed to add mesh to shading group");
    }

    // Connect emitter output mesh to display mesh
    MFnDependencyNode fnMeshNode(meshObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    MObject inMeshAttr = fnMeshNode.attribute("inMesh");
    MObject outputMeshAttr = fnEmitterNode.attribute("OutputMesh");
    
    status = dagModConnections.connect(emitterObj, outputMeshAttr, meshObj, inMeshAttr);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Set file path on emitter (if specified)
    if (m_filePath.length() > 0)
    {
        // Convert Windows backslashes to forward slashes
        std::string pathString = m_filePath.asChar();
        boost::replace_all(pathString, "\\", "/");
        MString filePath(pathString.c_str());
        
        // Fix: Correct string concatenation (was: +"\"")
        command = "setAttr " + emitterDag.fullPathName() + ".pointCloudFile -type \"string\" \"" + filePath + "\"";
        status = MGlobal::executeCommand(command, false, true);
        if (status != MS::kSuccess)
        {
            MGlobal::displayWarning("Failed to set file path on emitter");
        }
    }

    status = dagModConnections.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}

/**
 * Adds custom attributes to the particle system.
 * Creates rgbPP (per-particle color), rgbPP0 (initial color), and pointSize
 * attributes on the particle system for custom rendering and display.
 * @param particleObj The particle system node to add attributes to.
 * @return MStatus indicating success or failure.
 */
MStatus ScatteringNodeCmd::addAttributesToParticleSys(MObject particleObj)
{
    MStatus status;
    
    MFnDagNode dagNode(particleObj);
    
    // Add rgbPP attribute for per-particle color
    MString command = "addAttr -ln \"rgbPP\" -dt vectorArray " + dagNode.fullPathName();
    status = MGlobal::executeCommand(command, false, true);
    if (status != MS::kSuccess)
    {
        MGlobal::displayWarning("Failed to add rgbPP attribute");
    }
    
    // Add rgbPP0 attribute (per-particle color initial value)
    command = "addAttr -ln \"rgbPP0\" -dt vectorArray " + dagNode.fullPathName();
    status = MGlobal::executeCommand(command, false, true);
    if (status != MS::kSuccess)
    {
        MGlobal::displayWarning("Failed to add rgbPP0 attribute");
    }

    // Add pointSize attribute (if it doesn't already exist)
    command = "addAttr -is true -ln \"pointSize\" -at long -min 1 -max 60 -dv 6 " + dagNode.fullPathName();
    status = MGlobal::executeCommand(command, false, true);
    // Note: This may fail if attribute already exists, which is fine

    return MS::kSuccess;
}

/**
 * Hides unused emitter attributes from the channel box.
 * The emitter inherits many attributes from MPxEmitterNode that we don't use.
 * This function hides them from the UI to keep the interface clean.
 * @param emitterObj The emitter node to hide attributes on.
 * @return MStatus indicating success or failure.
 */
MStatus ScatteringNodeCmd::hideUnusedAttributes(MObject emitterObj)
{
    MStatus status;
    
    MFnDagNode dagNode(emitterObj);
    MString nodePath = dagNode.fullPathName();
    
    // List of unused emitter attributes to hide from channel box
    // These are inherited from MPxEmitterNode but not used by our custom node
    const char* attributesToHide[] = {
        "emt", "rate", "scaleRateBySpeed", "useRatePP", "needParentUV",
        "cycleEmission", "cycleInterval", "maxDistance", "minDistance",
        "directionX", "directionY", "directionZ", "volumeShape",
        "scaleRateByObjectSize", "spread", "speed", "speedRandom",
        "tangentSpeed", "normalSpeed", "particleColorR", "particleColorG",
        "particleColorB", "volumeOffsetX", "volumeOffsetY", "volumeOffsetZ",
        "volumeSweep", "sectionRadius", "awayFromCenter", "awayFromAxis",
        "alongAxis", "aroundAxis", "randomDirection", "directionalSpeed",
        "scaleSpeedBySize", "displaySpeed"
    };
    
    const int numAttributes = sizeof(attributesToHide) / sizeof(attributesToHide[0]);
    
    for (int i = 0; i < numAttributes; ++i)
    {
        MString command = "setAttr -keyable false -channelBox false " + nodePath + "." + attributesToHide[i];
        status = MGlobal::executeCommand(command, false, true);
        // Note: Some attributes may not exist, which is fine
    }

    return MS::kSuccess;
}

/**
 * Extends a DAG path to point to a mesh shape node.
 * If the path already points to a mesh, returns immediately. Otherwise,
 * searches for a mesh shape node below the transform and extends the path to it.
 * Skips intermediate objects (construction history).
 * @param path DAG path to extend (modified in place).
 * @return MStatus::kSuccess if mesh found, MStatus::kFailure otherwise.
 */
MStatus ScatteringNodeCmd::getShapeNode(MDagPath& path)
{
    MStatus status;

    // If path already points to a mesh, we're done
    if (path.apiType() == MFn::kMesh)
    {
        return MS::kSuccess;
    }

    // Find mesh shape node below this transform
    unsigned int numShapes;
    status = path.numberOfShapesDirectlyBelow(numShapes);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    for (unsigned int i = 0; i < numShapes; ++i)
    {
        status = path.extendToShapeDirectlyBelow(i);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Skip if not a mesh
        if (!path.hasFn(MFn::kMesh))
        {
            path.pop();
            continue;
        }

        // Skip intermediate objects (construction history)
        MFnDagNode fnNode(path, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        
        if (!fnNode.isIntermediateObject())
        {
            return MS::kSuccess;
        }
        
        path.pop();
    }

    return MS::kFailure;
}

/**
 * Undo operation: Deletes all nodes created by this command.
 * Removes the emitter, particle system, and mesh nodes that were created
 * during doIt() execution.
 * @return MStatus indicating success or failure.
 */
MStatus ScatteringNodeCmd::undoIt()
{
    MStatus status;
    
    // Delete all created nodes
    for (unsigned int i = 0; i < m_createdNodes.length(); ++i)
    {
        MFnDependencyNode fnNode(m_createdNodes[i], &status);
        if (status == MS::kSuccess)
        {
            MString command = "delete " + fnNode.name();
            MGlobal::executeCommand(command, false, true);
        }
    }
    
    m_createdNodes.clear();
    return MS::kSuccess;
}

/**
 * Redo operation: Re-executes the command.
 * Currently a placeholder - a full implementation would store command
 * arguments and re-run doIt() with the same parameters.
 * @return MStatus indicating success or failure.
 */
MStatus ScatteringNodeCmd::redoIt()
{
    // For now, redo just re-executes the command
    // A proper implementation would store the command arguments and re-run doIt()
    return MS::kSuccess;
}

/**
 * Indicates whether this command supports undo/redo.
 * @return Always returns true (command is undoable).
 */
bool ScatteringNodeCmd::isUndoable() const
{
    return true;
}
