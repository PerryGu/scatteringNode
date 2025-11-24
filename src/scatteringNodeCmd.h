#ifndef SCATTERINGNODECMD_H
#define SCATTERINGNODECMD_H

// Includes needed for class declaration only
// (Types used in member variables, function signatures, or base class)
#include <maya/MArgList.h>         // Function parameter in doIt()
#include <maya/MDagPath.h>          // Function parameter in getShapeNode()
#include <maya/MDagPathArray.h>    // Member variable: m_pathGeoList
#include <maya/MDGModifier.h>      // Member variable: m_dgModifier
#include <maya/MObject.h>           // Function parameters
#include <maya/MObjectArray.h>     // Member variable: m_createdNodes
#include <maya/MPxCommand.h>       // Base class
#include <maya/MSyntax.h>           // Return type of newSyntax()
#include <maya/MString.h>           // Member variables
#include <maya/MStringArray.h>     // Member variable: m_meshTransformNameList
#include <maya/MStatus.h>          // Return type for most functions

/**
 * Maya command for creating and configuring scattering node setups.
 * 
 * This command creates a scattering emitter node, particle system, and display mesh,
 * then connects them together with the selected geometry.
 */
class ScatteringNodeCmd : public MPxCommand
{
public:
    ScatteringNodeCmd();
    virtual ~ScatteringNodeCmd() = default;

    static void* creator();
    static MSyntax newSyntax();

    virtual MStatus doIt(const MArgList& argList);
    virtual MStatus undoIt();
    virtual MStatus redoIt();
    virtual bool isUndoable() const;

    // Command name constant
    static const char* kName;

private:
    /**
     * Connects the emitter, particle system, and mesh together.
     * Also connects input geometry and sets up file paths.
     */
    MStatus connectAll(MObject emitterObj, MObject particleObj, MObject meshObj);

    /**
     * Adds custom attributes to the particle system (rgbPP, pointSize).
     */
    MStatus addAttributesToParticleSys(MObject particleObj);

    /**
     * Hides unused emitter attributes from the channel box.
     */
    MStatus hideUnusedAttributes(MObject emitterObj);

    /**
     * Extends the given DAG path to point to a mesh shape node.
     * Returns kSuccess if a valid mesh shape is found, kFailure otherwise.
     */
    MStatus getShapeNode(MDagPath& path);

    // Command arguments
    MDagPathArray m_pathGeoList;          // Selected mesh geometry paths
    MString m_name;                        // Base name for created nodes
    MString m_filePath;                    // File path for point cloud export
    double m_randomPos;                    // Random position offset value
    MStringArray m_meshTransformNameList; // Transform names for matrix connections

    // Created node names
    MString m_nameEmitter;
    MString m_nameParticle;
    MString m_nameMesh;

    // Undo/redo support
    MDGModifier m_dgModifier;              // Dependency graph modifier for undo/redo
    MObjectArray m_createdNodes;           // Track created nodes for cleanup
};

#endif // SCATTERINGNODECMD_H
