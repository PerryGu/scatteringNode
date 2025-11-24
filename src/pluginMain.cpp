
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MThreadPool.h>

#include "scatteringEmitter.h"
#include "scatteringNodeCmd.h"

namespace
{
// Metadata describing this plugin. Keeping it here prevents typo-prone string
// duplication later in the file.
constexpr char kPluginVendor[] = "Guy Perry";
constexpr char kPluginVersion[] = "maya2020_v1.1.1_open3D";
constexpr char kPluginRequiredApi[] = "vs_19";

// Public names for the custom node and command.
constexpr char kNodeName[] = "scatteringNode";
constexpr char kCommandName[] = "scatteringNodeCmd";
} // namespace

MStatus initializePlugin(MObject obj)
{
	MStatus status;
	
	// Initialize Maya's thread pool once for the entire plugin lifetime.
	// This is shared by all scatteringNode instances.
	MThreadPool::init();
	
	// The Maya plugin object encapsulates registration and deregistration.
	MFnPlugin plugin(obj, kPluginVendor, kPluginVersion, kPluginRequiredApi);

	// Register the MPxEmitterNode that performs the scattering work.
	status = plugin.registerNode(kNodeName,
	                            scatteringNode::id,
	                            scatteringNode::creator,
	                            scatteringNode::initialize,
	                            MPxNode::kEmitterNode);
	if (status != MS::kSuccess)
	{
		return status;
	}

	// Register the helper MPxCommand that builds the node setup.
	status = plugin.registerCommand(kCommandName,
	                                ScatteringNodeCmd::creator,
	                                ScatteringNodeCmd::newSyntax);
	if (status != MS::kSuccess)
	{
		// Command registration failed—undo the node registration so Maya
		// doesn't keep a half-installed plugin around.
		plugin.deregisterNode(scatteringNode::id);
		return status;
	}

	MGlobal::displayInfo("scatteringNode plugin loaded.");
	return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj);

	// Always deregister the command before removing the node; it's the reverse
	// of the order used in initializePlugin.
	status = plugin.deregisterCommand(kCommandName);
	if (status != MS::kSuccess)
	{
		return status;
	}

	status = plugin.deregisterNode(scatteringNode::id);
	if (status == MS::kSuccess)
	{
		MGlobal::displayInfo("scatteringNode plugin unloaded.");
	}
	
	// Release Maya's thread pool resources now that the plugin is unloading.
	MThreadPool::release();

	return status;
}
