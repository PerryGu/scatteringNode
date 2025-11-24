
#ifndef SCATTERING_EMITTER_H
#define SCATTERING_EMITTER_H

#include <maya/MArrayDataHandle.h>
#include <maya/MBoundingBox.h>
#include <maya/MDataBlock.h>
#include <maya/MDagPath.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MIntArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnParticleSystem.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPxEmitterNode.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MThreadPool.h>
#include <maya/MTime.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MTransformationMatrix.h>

#include <iostream>
#include <map>
#include <vector>


/**
 * Holds basic information about a mesh's axis-aligned bounding box and
 * derived spacing parameters used by the projection-based sampler.
 */
struct BoundingBoxData
{
	double xlen;
	double ylen;
	double zlen;
	float min_x;
	float min_y;
	float min_z;
	float max_x;
	float max_y;
	float max_z;
	float borderOffset;
	float step;
};

/**
 * Stores the rays emitted from a bounding-box face (origins + directions).
 * Each entry corresponds to a line we cast into the mesh.
 */
struct ProjectionPointsData
{
	MFloatPointArray raySourcePointsArray;
	MFloatVectorArray rayDirectionPointsArray;
};

//-- Thread Data ----------------------
/**
 * Set of inputs shared by all worker threads during scattering evaluation.
 * Each thread receives a subset of points/rays to process.
 */
struct TaskData
{
	MMatrix taskData_matrixHandle;
	MTransformationMatrix taskData_meshMatrix;
	MFloatPointArray points;
	MBoundingBox taskData_meshBbox;
	MDagPath taskData_dagPath;
	bool taskData_normalDirection;
	int taskData_pointsDensity;
	short taskData_eval_state;
	double  taskData_randomPosX;
	double  taskData_randomPosY;
	double  taskData_randomPosZ;
	double  taskData_preRandomPos;
	short taskData_surfaceVolume;
	short taskData_projectionDirection;
	std::vector<MFloatPointArray> taskData_pointsPosArray;
	std::vector<MVectorArray> taskData_pointsVecArray;
	std::vector<MFloatPointArray> taskData_pointsArrayToDraw;
	BoundingBoxData taskData_boundingBoxData;
};

//-- Thread Stuff ----------------------
/**
 * Metadata for each worker thread (range of work items + pointer to shared task data).
 */
struct ThreadData
{
	unsigned int start;
	unsigned int end;
	unsigned int numTasks;
	unsigned int currentTask;

	TaskData* pData;
};


/**
 * Custom Maya emitter node that scatters points over/inside input meshes, supports
 * shader sampling, multithreaded generation, and point-cloud export.
 */
class scatteringNode : public MPxEmitterNode// ,  public  MPxParticleAttributeMapperNode
{
public:
	scatteringNode();
	virtual ~scatteringNode();

	static void		*creator();
	static MStatus	initialize();
	virtual MStatus	compute(const MPlug& plug, MDataBlock& dataBlock);
	MStatus		    jumpToElement(MArrayDataHandle& hArray, unsigned int index);
	static MTypeId  id;

	/**
	 * Builds the projection rays used to sample a mesh's bounding box.
	 */
	ProjectionPointsData generateProjectionPoints(MDagPath dagPath, short points_density, double preRandomPos, bool surfaceVolume, BoundingBoxData boundingBoxData, short projectionDirection);
	/**
	 * Splits points/rays evenly among worker threads.
	 */
	TaskData pointsDivideToThreads(TaskData taskData, int numTasks, MFloatPointArray pointsPosArray, MFloatVectorArray pointsVecArray, short eval_state);
	/**
	 * Lazy-fetches the particle system connected to this emitter (if any).
	 */
	virtual MStatus getTheParticleSys();
	/**
	 * Tracks dirty plugs to minimize recomputation.
	 */
	virtual MStatus setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs);
	/**
	 * Validates a file/directory path to determine how point clouds should be loaded/saved.
	 */
	virtual MStatus	path_check(MString pathFile, short readWritRGB);
	/**
	 * Loads point-cloud data from a PCD/PLY file via Open3D.
	 */
	virtual MStatus	load_PCD_PLY_File(MString pathFile, short readWritRGB, MVectorArray& pointArrayPos_global);
	/**
	 * Saves generated point data to disk (PCD/PLY, ascii/binary).
	 */
	virtual MStatus	saveToFile(MString pathFile, short readWritRGB, short savePCDPLY, short saveAsciiBinary);
	/**
	 * Writes sampled colors back to the particle system.
	 */
	virtual MStatus setParticleColor(float colorR, float colorG, float colorB, short colorControl);
	/**
	 * Finds the shading networks connected to a mesh.
	 */
	MStringArray getConnectedShade(MDagPath dagPath);
	/**
	 * Samples colors from a mesh's shader network at the provided points.
	 */
	MStatus sampleColorFromMesh(MDagPath dagPath, MFloatPointArray pointsPos, MString shaderName);


	static MObject pointsCreate_Attr;
	static MObject density_Attr;
	static MObject projectionDirection_Attr;
	static MObject preRandomPos_Attr;
	static MObject randomPos_Attr;
	static MObject randomPosX_Attr;
	static MObject randomPosY_Attr;
	static MObject randomPosZ_Attr;
	static MObject colorControl_Attr;
	static MObject colorR_Attr;
	static MObject colorG_Attr;
	static MObject colorB_Attr;
	static MObject getNormalsFromMesh_Attr;
	static MObject voxelWidth_Attr;
	static MObject outputTypeDisplay_Attr;
	static MObject targetGeo_Attr;
	static MObject targetGeoMatrix_Attr;
	static MObject outputPos_Attr;
	static MObject pointNumber_Attr;
	static MObject particleDriver_Attr;
	static MObject saveToFile_Attr;
	static MObject savePCDPLY_attr;
	static MObject saveAsciiBinary_attr;
	static MObject pclFilePath_Attr;
	static MObject outputMesh_Attr;
	static MObject inColor_Attr;
	static MObject surfaceVolume_Attr;
	static MObject optimize_Attr;
	static MObject normalDirection_Attr;
	static MObject numTasks_Attr;


	//-- Threading functions ----------------------------------------------
	/**
	 * Allocates metadata describing how the work should be split across threads.
	 */
	ThreadData*         createThreadData(int numTasks, TaskData* pTaskData);
	/**
	 * Entry point passed to Maya's thread pool, responsible for queuing per-thread jobs.
	 */
	static void         createTasks(void* data, MThreadRootTask *pRoot);
	/**
	 * Actual threaded evaluation function (raycasts/intersections + point placement).
	 */
	static MThreadRetVal threadEvaluate(void* pParam);

private:
	std::map<int, MFloatPointArray> pointArrayPos_dyn;
	std::map<int, MFloatPointArray> pointArrayPos_stat;
	MVectorArray				pointArrayPos_global;
	MVectorArray				pointArrayPos_mesh;
	MVectorArray				colorFromFile_list;
	MVectorArray				sampleColorsMesh_list;
	MVectorArray				colorsOnParticles_list;
	MFloatVectorArray			vertexNormals_list;
	MObject						newOutputData;
	MFnParticleSystem			particle_sys;
	MObject						thisNode;
	MString						pathFile;
	MString						pathFile_check;
	MFnMesh						meshFn_obj;
	short						eval_state;
	MTime   currentTimeValue(MDataBlock& block);
	

	/**
	 * Emits particles into the connected particle system.
	 */
	void	emit(MVectorArray &outPosAry);
	/**
	 * Builds or updates the voxel mesh representation of the scattered points.
	 */
	void	createScatteringMesh(MVectorArray& pointArrayPos_mesh, MObject& newOutputMeshData, float voxelWidth);
	/**
	 * Helper used by createScatteringMesh to append a single voxel cube to the mesh arrays.
	 */
	void	createCube(MVector pVoxelPosition, float pWidth,
		MFloatPointArray& pVertexArray, int pVertexIndexOffset, int pNumVerticesPerVoxel,
		MIntArray& pPolygonCountArray, int pPolygonCountIndexOffset, int pNumPolygonsPerVoxel, int pNumVerticesPerPolygon,
		MIntArray& pPolygonConnectsArray, int pPolygonConnectsIndexOffset);


	/**
	 * Utility to query Maya's "particle system full" flag.
	 */
	bool	isFullValue(int plugIndex, MDataBlock& dataBlock);

};

/**
 * Retrieves the current time value from the Maya data block.
 * 
 * @param block The data block containing the current time attribute.
 * @return The current time value, or 0.0 if the attribute cannot be read.
 */
inline MTime scatteringNode::currentTimeValue(MDataBlock& block)
{
	MStatus status;

	MDataHandle hValue = block.inputValue(mCurrentTime, &status);

	MTime value(0.0);
	if (status == MS::kSuccess)
		value = hValue.asTime();

	return(value);
}

/**
 * Checks whether a particle system at the given plug index has reached its maximum capacity.
 * 
 * @param plugIndex The logical index of the particle system plug to check.
 * @param dataBlock The data block containing the particle system state.
 * @return True if the particle system is full, false otherwise (defaults to true on error).
 */
inline bool scatteringNode::isFullValue(int plugIndex, MDataBlock& dataBlock)
{
	MStatus status;
	bool value = true;

	MArrayDataHandle mhValue = dataBlock.inputArrayValue(mIsFull, &status);
	if (status == MS::kSuccess)
	{
		status = mhValue.jumpToElement(plugIndex);
		if (status == MS::kSuccess)
		{
			MDataHandle hValue = mhValue.inputValue(&status);
			if (status == MS::kSuccess)
				value = hValue.asBool();
		}
	}

	return(value);
}

#define McheckErr(stat, msg)		\
	if ( MS::kSuccess != stat )		\
		{								\
		cerr << msg;				\
		return MS::kFailure;		\
		}

#endif // SCATTERING_EMITTER_H