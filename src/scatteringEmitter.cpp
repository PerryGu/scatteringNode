
// ==========================================================================
// scatteringNode - Maya plugin for point cloud scattering
// Copyright (c) Guy Perry
// ==========================================================================


#include "scatteringEmitter.h"

// Standard library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <mutex>
#include <random>
#include <string>
#include <thread>

// Third-party libraries
#include <Open3D/Open3D.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>

// Maya API (implementation specifics beyond the header)
#include <maya/MArrayDataBuilder.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnArrayAttrsData.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnData.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnGenericAttribute.h>
#include <maya/MFnLambertShader.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnVectorArrayData.h>
#include <maya/MIOStream.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MRenderUtil.h>
#include <maya/MRenderView.h>
#include <maya/MSelectionList.h>


MTypeId scatteringNode::id(0x80014);
MObject scatteringNode::pointsCreate_Attr;
MObject scatteringNode::getNormalsFromMesh_Attr;
MObject scatteringNode::savePCDPLY_Attr;
MObject scatteringNode::saveAsciiBinary_attr;
MObject scatteringNode::pclFilePath_Attr;
MObject scatteringNode::saveToFile_Attr;
MObject scatteringNode::outputTypeDisplay_Attr;
MObject scatteringNode::targetGeo_Attr;
MObject scatteringNode::targetGeoMatrix_Attr;
MObject scatteringNode::density_Attr;
MObject scatteringNode::projectionDirection_Attr;
MObject scatteringNode::colorControl_Attr;
MObject scatteringNode::colorR_Attr;
MObject scatteringNode::colorG_Attr;
MObject scatteringNode::colorB_Attr;
MObject scatteringNode::preRandomPos_Attr;
MObject scatteringNode::randomPos_Attr;
MObject scatteringNode::randomPosX_Attr;
MObject scatteringNode::randomPosY_Attr;
MObject scatteringNode::randomPosZ_Attr;
MObject scatteringNode::voxelWidth_Attr;
MObject scatteringNode::outputPos_Attr;
MObject scatteringNode::pointNumber_Attr;
MObject scatteringNode::particleDriver_Attr;
MObject scatteringNode::outputMesh_Attr;
MObject scatteringNode::inColor_Attr;
MObject scatteringNode::surfaceVolume_Attr;
MObject scatteringNode::normalDirection_Attr;
MObject scatteringNode::numTasks_Attr;


/**
 * Returns a reference to a thread-local random number generator.
 * Each thread gets its own generator instance, seeded with a combination of
 * thread ID and current time to ensure good randomness.
 * 
 * @return Reference to thread-local std::mt19937 generator.
 */
static std::mt19937& getThreadLocalRNG()
{
	thread_local std::mt19937 rng;
	thread_local bool initialized = false;
	
	if (!initialized)
	{
		// Seed with thread ID and current time for good randomness
		unsigned int seed = static_cast<unsigned int>(
			std::chrono::high_resolution_clock::now().time_since_epoch().count()
		);
		std::hash<std::thread::id> hasher;
		seed ^= static_cast<unsigned int>(hasher(std::this_thread::get_id()));
		rng.seed(seed);
		initialized = true;
	}
	
	return rng;
}

/**
 * Mutex to protect Maya's thread-unsafe MFnMesh::allIntersections() calls.
 * All threads share this mutex to serialize intersection operations.
 */
static std::mutex g_intersectionMutex;


/**
 * Constructor.
 */
scatteringNode::scatteringNode()
{
}


/**
 * Destructor.
 */
scatteringNode::~scatteringNode()
{
}


/**
 * Factory method required by Maya for MPx nodes.
 */
void *scatteringNode::creator()
{
	return new scatteringNode;
}


/**
 * Main evaluation entry point for the emitter node.
 * Reads all user attributes, prepares the scattering task data, launches
 * threaded computations, and fills either the particle system or mesh output.
 */
MStatus scatteringNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	//MGlobal::displayInfo("** compute **");

	MStatus status;
	thisNode = thisMObject();

	//-- create points Attr  ----------
	short dataHandle_pointsCreate = dataBlock.inputValue(pointsCreate_Attr).asShort();


	//-- get the Display Type (Particle or mesh ) -----------
	short dataHandle_outputTypeDisplay = dataBlock.inputValue(outputTypeDisplay_Attr).asShort();

	//-- get the density Attr ---------------------------
	short points_density = dataBlock.inputValue(density_Attr).asInt();
	points_density = std::max<short>(points_density, static_cast<short>(2));

	//-- get the projectionDirection  Attr ---------------------------
	short projectionDirection = dataBlock.inputValue(projectionDirection_Attr).asShort();

	//-- pre random Pos Attr ---------------------------
	double preRandomPos = dataBlock.inputValue(preRandomPos_Attr).asDouble();

	//-- random Pos Attr ---------------------------
	MDataHandle randomPos = dataBlock.inputValue(randomPos_Attr);
	double randomPosX = randomPos.child(randomPosX_Attr).asDouble();
	double randomPosY = randomPos.child(randomPosY_Attr).asDouble();
	double randomPosZ = randomPos.child(randomPosZ_Attr).asDouble();

	//-- read and writ RGB to a PCD file Attr -----------------------------------------
	short readWritRGB = 1;

	//-- get the colorControl Attr ---------------------------
	short colorControl = dataBlock.inputValue(colorControl_Attr).asShort();

	//-- get the colorR Attr ---------------------------
	float colorR = dataBlock.inputValue(colorR_Attr).asFloat();

	//-- get the colorG Attr ---------------------------
	float colorG = dataBlock.inputValue(colorG_Attr).asFloat();

	//-- get the colorB Attr ---------------------------
	float colorB = dataBlock.inputValue(colorB_Attr).asFloat();

	//-- get Normals From Mesh ----------
	short getNormalsFromMesh = dataBlock.inputValue(getNormalsFromMesh_Attr).asShort();

	//-- get the voxel Width ---------------------------
	float dataHandle_voxelWidth = dataBlock.inputValue(voxelWidth_Attr).asFloat();

	//-- Save PCD / PLY File Attr ----------
	short savePCDPLY = dataBlock.inputValue(savePCDPLY_attr).asShort();

	//-- Save Binary / Ascii File Attr ----------
	short saveAsciiBinary = dataBlock.inputValue(saveAsciiBinary_attr).asShort();


	//-- Save data To File Attr ----------
	short dataHandle_saveToFile = dataBlock.inputValue(saveToFile_Attr).asShort();


	//-- get surfaceVolume Attr  ----------
	short surfaceVolume = dataBlock.inputValue(surfaceVolume_Attr).asShort();

	//-- get normalDirection  Attr  ----------
	short normalDirection = dataBlock.inputValue(normalDirection_Attr).asShort();


	//-- File path Attr ----------
	MString dataHandle_filePath = dataBlock.inputValue(pclFilePath_Attr).asString();

	//-- get all target shape meshes ---------------------------
	MArrayDataHandle hArraydataHandle_targetGeo = dataBlock.inputArrayValue(targetGeo_Attr);

	//--- get target Matrix  meshes  Attribute  -----------------------
	MArrayDataHandle hArraydataHandle_targetGeoMatrix = dataBlock.inputArrayValue(targetGeoMatrix_Attr);


	//-- get the inColor ---------------------------
	MDataHandle inColor_data = dataBlock.inputValue(inColor_Attr);

	//-- get the number of Tasks ---------------------------
	int numTasks = dataBlock.inputValue(numTasks_Attr).asInt();


	//-- call getTheParticleSys proc  to get the particle class ---------------
	if (particle_sys.name() == "")
	{
		getTheParticleSys();

	}

	// Get deltaTime, currentTime and startTime.
	// If deltaTime <= 0.0, or currentTime <= startTime,
	// do not emit new pariticles and return.
	//

	// Generate random number using thread-safe RNG
	std::mt19937& rng = getThreadLocalRNG();
	std::uniform_int_distribution<int> dist(1, 10);
	int randNumber = dist(rng);
	dataBlock.outputValue(particleDriver_Attr).setMTime(float(randNumber));

	//--Emitter stuff------------------------------------------------ -
	//-- Determine if we are requesting the output plug for this emitter node.
	if (!(plug == mOutput))
		return(MS::kUnknownParameter);

	//-- Get the logical index of the element this plug refers to,
	//-- because the node can be emitting particles into more 
	//-- than one particle shape.
	int multiIndex = plug.logicalIndex(&status);
	McheckErr(status, "ERROR in plug.logicalIndex.\n");

	//-- Get output data arrays (position, velocity, or parentId)
	//-- that the particle shape is holding from the previous frame.
	MArrayDataHandle hOutArray = dataBlock.outputArrayValue(mOutput, &status);
	McheckErr(status, "ERROR in hOutArray = block.outputArrayValue.\n");

	//-- Create a builder to aid in the array construction efficiently.
	MArrayDataBuilder bOutArray = hOutArray.builder(&status);
	McheckErr(status, "ERROR in bOutArray = hOutArray.builder.\n");

	//-- Get the appropriate data array that is being currently evaluated.
	MDataHandle hOut = bOutArray.addElement(multiIndex, &status);
	McheckErr(status, "ERROR in hOut = bOutArray.addElement.\n");

	//-- Create the data and apply the function set,
	//-- particle array initialized to length zero, 
	MFnArrayAttrsData fnOutput;
	MObject dOutput = fnOutput.create(&status);
	McheckErr(status, "ERROR in fnOutput.create.\n");


	//-- Check if the particle object has reached it's maximum,
	//-- hence is full. If it is full then just return with zero particles.
	bool beenFull = isFullValue(multiIndex, dataBlock);
	if (beenFull)
	{
		return(MS::kSuccess);
	}


	MVectorArray fnOutPos = fnOutput.vectorArray("position", &status);


	//-- clear sampleColorsMesh_list -------------------
	sampleColorsMesh_list.clear();
	

	//-- call data struct ---------------------------------
	TaskData taskData;
	BoundingBoxData boundingBoxData;
	vertexNormals_list.clear();

	//** Create points set to off (NO POINTS IN THE SCENE) *************************************
	if (dataHandle_pointsCreate == 0) {
		pathFile_check = "";
		//-- call "createScatteringMesh" to clear the old mesh -----------------
		if (pointArrayPos_mesh.length() > 1) {
			pointArrayPos_mesh.clear();
			createScatteringMesh(pointArrayPos_mesh, newOutputData, dataHandle_voxelWidth);
		}
		//-- call "emit" to clear the old particle -----------------	
		if (pointArrayPos_global.length() > 1) {
			pointArrayPos_global.clear();
			emit(fnOutPos);
		}
	}

	//** create points from mehs in the scene ***********************************************
	if (dataHandle_pointsCreate == 1) {
		pathFile_check = "";
		unsigned int numTargets = hArraydataHandle_targetGeo.elementCount();
		unsigned int logicalIndex;

		//-- clear all data  from pointArrayPos_global array -----------------
		pointArrayPos_global.clear();

		//-- gather data for the threading ---------------------------------------	
		taskData.taskData_pointsDensity = points_density;
		taskData.points.setLength(numTargets);
		taskData.taskData_normalDirection = normalDirection;


		//-- loop on all connected mesh objects ----------------------------
		for (unsigned int i = 0; i < numTargets; ++i)
		{
			//---- input mesh shape target -----------------------------------------
			hArraydataHandle_targetGeoMatrix.jumpToArrayElement(i);
			logicalIndex = hArraydataHandle_targetGeoMatrix.elementIndex();
			jumpToElement(hArraydataHandle_targetGeoMatrix, logicalIndex);

			//---input mesh matrix target-------------------------------------------------------------
			//if (hArraydataHandle_targetGeoMatrix.outputValue().type() == MFnData::kMatrix) {
			MMatrix matrixHandle = hArraydataHandle_targetGeoMatrix.inputValue().asMatrix();

			//---- input mesh shape target -----------------------------------------
			hArraydataHandle_targetGeo.jumpToArrayElement(i);
			logicalIndex = hArraydataHandle_targetGeo.elementIndex();
			jumpToElement(hArraydataHandle_targetGeo, logicalIndex);

			if (hArraydataHandle_targetGeo.outputValue().type() == MFnData::kMesh)
			{
				//-- Compute the bounding box around the mesh's vertices ------------
				MPlug mesh_mplug = MPlug(thisNode, targetGeo_Attr);
				mesh_mplug.selectAncestorLogicalIndex(logicalIndex, targetGeo_Attr);
				MPlugArray mPlugArray = MPlugArray();
				mesh_mplug.connectedTo(mPlugArray, true, true);
				//-- TO Prevent MAYA crashing ------------
				if (mPlugArray.length() == 0)
				{
					return MS::kInvalidParameter;
				}

				//-- get connected mesh ----------
				MObject mObj = mPlugArray[0].node();
				MDagPath dagPath = MDagPath();
				dagPath = dagPath.getAPathTo(mObj);
				MMatrix mExclusive = dagPath.exclusiveMatrix();
				MFnDagNode mesh_dagNode(dagPath);
				MBoundingBox meshBbox = mesh_dagNode.boundingBox();
				meshBbox.transformUsing(mExclusive);

				taskData.taskData_dagPath = dagPath;
				taskData.taskData_matrixHandle = matrixHandle;
				taskData.taskData_meshBbox = meshBbox;

				//----------------------------------
				//-- get lengths of a mesh bounding box ----
				double xlen = meshBbox.max().x - meshBbox.min().x;
				double ylen = meshBbox.max().y - meshBbox.min().y;
				double zlen = meshBbox.max().z - meshBbox.min().z;
				boundingBoxData.xlen = xlen;
				boundingBoxData.ylen = ylen;
				boundingBoxData.zlen = zlen;

				//--find which length is the minimal ----
				double minEdge = xlen;
				if (minEdge > ylen)
				{
					minEdge = ylen;
				}

				if (minEdge > zlen)
				{
					minEdge = zlen;
				}

				float min_x = meshBbox.min().x;
				float min_y = meshBbox.min().y;
				float min_z = meshBbox.min().z;
				float max_x = meshBbox.max().x;
				float max_y = meshBbox.max().y;
				float max_z = meshBbox.max().z;
				boundingBoxData.min_x = min_x;
				boundingBoxData.min_y = min_y;
				boundingBoxData.min_z = min_z;
				boundingBoxData.max_x = max_x;
				boundingBoxData.max_y = max_y;
				boundingBoxData.max_z = max_z;

				//-- calculate spacing between points  ------------------------------------------
				float borderOffset = 0.01f;
				float step = (float)((minEdge - 2 * minEdge * 0.1) / (points_density - 1));
				boundingBoxData.borderOffset = borderOffset;
				boundingBoxData.step = step;

				taskData.taskData_boundingBoxData = boundingBoxData;


				//-- affter cheking if update is - transformaion or shape deform --------------
				//-- if update is shape deform --------------------------
				MFloatVectorArray rayDirectionPointsArray;
				if (eval_state == 1) {
					taskData.taskData_eval_state = eval_state;
					taskData.taskData_preRandomPos = preRandomPos;
					MFloatPointArray boundingBoxPointsArray;
					MVector rayDirection;
					pointArrayPos_stat[i].clear();
					pointArrayPos_dyn[i].clear();


					//-- algorithm Type Projection  ---------------------------------------------------------
					//-- call "generateProjectionPoints" to  generate new Plane of  Point Position -------------------------
					ProjectionPointsData projectionPointsData = generateProjectionPoints(dagPath, points_density, preRandomPos, surfaceVolume, boundingBoxData, projectionDirection);

					//-- no intersections - create only the plane points -------------------------------------------------------------
					if (surfaceVolume == 0) {
						pointArrayPos_dyn[i] = projectionPointsData.raySourcePointsArray;
					}

					//-- call the intersections to get all the intersections with the mesh -----------------
					if (surfaceVolume == 1 || surfaceVolume == 2) {
						//--- call 'pointsDivideToThreads' to generate one list of point to eace Thread----------------
						taskData = pointsDivideToThreads(taskData, numTasks, projectionPointsData.raySourcePointsArray, projectionPointsData.rayDirectionPointsArray, eval_state);

						//-- call the intersections to get all the intersections with the mesh -----------------
						//-- Thread -------------------------------------------------------------------------------
						taskData.taskData_surfaceVolume = surfaceVolume;
						taskData.taskData_projectionDirection = projectionDirection;
						ThreadData* pThreadData = createThreadData(numTasks, &taskData);
						MThreadPool::newParallelRegion(createTasks, (void*)pThreadData);
						delete[] pThreadData;

						//--append new points poition to the  pointArrayPos_dyn --------------------------------
						for (unsigned int p = 0; p < taskData.taskData_pointsArrayToDraw.size(); ++p) {
							for (unsigned int x = 0; x < taskData.taskData_pointsArrayToDraw[p].length(); ++x) {
								pointArrayPos_dyn[i].append(taskData.taskData_pointsArrayToDraw[p][x]);
							}
						}
					}

					//-- get points on vertexs ----------------------------------
					if (surfaceVolume == 3) {
						MFnMesh mfnMesh;
						MFloatVectorArray vertexNormals;
						mfnMesh.setObject(dagPath);
						mfnMesh.getPoints(pointArrayPos_dyn[i], MSpace::kWorld);

						//-get the vertex normals ---------------------
						if (getNormalsFromMesh == 1)
						{
							bool angleWeighted = 0;
							mfnMesh.getVertexNormals(angleWeighted, vertexNormals, MSpace::kWorld); 

							//-- append each mesh normals to normal list -------------------------
							for (unsigned int v = 0; v < vertexNormals.length(); ++v)
							{
								vertexNormals_list.append(vertexNormals[v]);
							}
						}
					}

					//*******************************************************************************************
					//-- get position in the world  for each point and save the pos to a new list ---------------
					//-- invers the metrix points pos -----------------------------------------------
					MMatrix handle_meshMatrixInverse = matrixHandle.inverse();
					for (unsigned int p = 0; p < pointArrayPos_dyn[i].length(); ++p)
					{
						MFloatPoint inversePointPos = MPoint(pointArrayPos_dyn[i][p].x, pointArrayPos_dyn[i][p].y, pointArrayPos_dyn[i][p].z) * handle_meshMatrixInverse;
						pointArrayPos_stat[i].append(inversePointPos);
					}
				}

				//-- if transfotm change (no deformatin) ----------------	
				if (eval_state == 0)
				{
					//--- call 'pointsDivideToThreads' to generate one list of point to eace Thread----------------
					pointArrayPos_dyn[i].clear();
					rayDirectionPointsArray.clear();
					taskData = pointsDivideToThreads(taskData, numTasks, pointArrayPos_stat[i], rayDirectionPointsArray, eval_state);

					//-- call point Transformation to only set new position without raycast () --------
					taskData.taskData_eval_state = eval_state;
					taskData.taskData_meshMatrix = matrixHandle;
					taskData.taskData_randomPosX = randomPosX;
					taskData.taskData_randomPosY = randomPosY;
					taskData.taskData_randomPosZ = randomPosZ;
					ThreadData* pThreadData = createThreadData(numTasks, &taskData);
					MThreadPool::newParallelRegion(createTasks, (void*)pThreadData);
					delete[] pThreadData;

					//--append new points poition to the  pointArrayPos_dyn --------------------------------
					for (unsigned int p = 0; p < taskData.taskData_pointsArrayToDraw.size(); ++p) {
						for (unsigned int x = 0; x < taskData.taskData_pointsArrayToDraw[p].length(); ++x) {
							pointArrayPos_dyn[i].append(taskData.taskData_pointsArrayToDraw[p][x]);
						}
					}
				}

				//== set all points of all connected objecrs to global array ===============================		
				for (unsigned int p = 0; p < pointArrayPos_dyn[i].length(); ++p)
				{
					pointArrayPos_global.append(pointArrayPos_dyn[i][p]);
				}

				//=== set color =========================================================================
				//== Convenient way to get rid of Un-necessary Attribute - "colorControl_Attr" ==========
				if (colorControl == 0)
				{
					readWritRGB = 0;
				}
				//else
				//-- colore off --------------
				if (colorControl == 0)
				{
					//-- clear sampleColorsMesh_list -------------------
					sampleColorsMesh_list.clear();
				}
				//-- colore from mesh --------------
				if ( colorControl == 2)
				{
					//-- set colorControl_Attr Attr to value 1 - "colorFromFile" --------------
					colorControl = 2;
					MFnDependencyNode fnNode(thisNode, &status);
					MPlug colorControl_plug = fnNode.findPlug("colorControl", false, &status);
					colorControl_plug.setShort(colorControl);
				}
				//-- sampe colors from connected mesh -----------------------------------
				if (colorControl == 2)
				{
					MStringArray shaderName = getConnectedShade(dagPath);
					
					if (pointArrayPos_dyn[i].length() > 0)
					{
						//-- textuer exist ------------------
						if (shaderName[1] == "textuer")
						{
							sampleColorFromMesh(dagPath, pointArrayPos_dyn[i], shaderName[0]);
						}

						//-- No textuer only shade color  ------------------
						if (shaderName[1] == "shader")
						{
							//-- get shader plug to extract the colors ----------------
							MSelectionList node_list;
							node_list.add(shaderName[0]);
							MObject textuer_obj;
							node_list.getDependNode(0, textuer_obj);
							MFnDependencyNode textuer_node(textuer_obj, &status);
							MPlug textuer_plugColor = textuer_node.findPlug("color", false, &status);
							if (textuer_plugColor.isCompound()) {
								float colR= 0, colG = 0, colB = 0;
								MPlug plug_r = textuer_plugColor.child(0);
								MPlug plug_g = textuer_plugColor.child(1);
								MPlug plug_b = textuer_plugColor.child(2);
								plug_r.getValue(colR);
								plug_g.getValue(colG);
								plug_b.getValue(colB);

								//-- appebd color to  sampleColorsMesh_list ----------------------
								for (unsigned int c = 0; c < pointArrayPos_dyn[i].length(); c += 1) {
									sampleColorsMesh_list.append(MVector(colR, colG, colB));
								}
							}
						}
					}
					//-- call setParticleColor ------------------
					setParticleColor(colorR, colorG, colorB, colorControl);
				}

				//-- set RGB color ----------------------------
				if (colorControl == 3)
				{
					//-- call setColor proc ---------------
					setParticleColor(colorR, colorG, colorB, colorControl);
				}
			}
		}
	}


	//=== load from file ==========================================================================================
	//-- create points from file  ----------------------------------
	else if (dataHandle_pointsCreate == 2) {
		//-- only if save to file is OFF  ----
		if (dataHandle_saveToFile == 0)
		{
			//-- check if file path exists -----------------------------------------
			if (dataHandle_filePath != "")
			{
				pathFile = dataHandle_filePath.asChar();
				//-- check if already bin load (to prevent reload file again and again..) --------------------------------
				if (pathFile != pathFile_check)
				{
					pointArrayPos_global.clear();

					//-- call "emit" to clear the old particle -----------------					
					emit(fnOutPos);

					//-- load the PCL File ----------------------------------------
					path_check(pathFile, readWritRGB);
					
					//=== set color ================================================================
					if ( colorControl == 1)
					{
						//-- set colorControl_Attr Attr to value 1 - "colorFromFile" --------------
						colorControl = 1;
						MFnDependencyNode fnNode(thisNode, &status);
						MPlug colorControl_plug = fnNode.findPlug("colorControl", false, &status);
						colorControl_plug.setShort(colorControl);
					}

					//-- call setColor proc ---------------
					setParticleColor(colorR, colorG, colorB, colorControl);
				}
			}
		}
	}

	//-- set output number of points created  ---------------------------------------
	dataBlock.outputValue(pointNumber_Attr).setInt(pointArrayPos_global.length());

	//-- set ran number to Particle sys currentTime (for eval) ---------------------------------------
	//**  if "dataHandle_outputTypeDisplay" set to 0 *****************************************
	//-- clean all the mesh list data to remove all scattering Mesh ---------------------------
	if (dataHandle_outputTypeDisplay == 0) {

		//-- to clear the mesh ---------------------------------
		if (pointArrayPos_mesh.length() > 1)
		{
			pointArrayPos_mesh.clear();
			createScatteringMesh(pointArrayPos_mesh, newOutputData, dataHandle_voxelWidth);

			//---Set the mesh output data -------------------------
			MDataHandle dataHandle_outputMesh = dataBlock.outputValue(outputMesh_Attr);
			dataHandle_outputMesh.set(newOutputData);
			dataHandle_outputMesh.setClean();
		}

		//== call emit Particle ====================
		emit(fnOutPos);
	}


	//== if "dataHandle_outputTypeDisplay" set to 1 ===================================
	//--  Create a mesh data container, which will store our new voxelized mesh ----
	if (dataHandle_outputTypeDisplay == 1) {
		//-- call create Scattering Mesh  -----------------------------------------
		pointArrayPos_mesh = pointArrayPos_global;
		MFnMeshData dataCreator;
		newOutputData = dataCreator.create();
		//-- call createScatteringMesh to create the mesh and set position for ezch cube element of the mesh ---------
		createScatteringMesh(pointArrayPos_mesh, newOutputData, dataHandle_voxelWidth);

		//-- Set the mesh output data -------------------------
		MDataHandle dataHandle_outputMesh = dataBlock.outputValue(outputMesh_Attr);
		dataHandle_outputMesh.set(newOutputData);
		dataHandle_outputMesh.setClean();

	}

	//-- Save to file trigger ----------------------------------
	if (dataHandle_saveToFile == 1) {

		//-- BUGE FIX ----------------------------------------------------
		//-- Can't save file with RGB data AND Normal data together --------
		//-- to skip the problem i set the color Attr to None when Set Normal is On ----
		if (getNormalsFromMesh == 1)
		{
			MFnDependencyNode fnNode(thisNode, &status);
			MPlug read_and_writ_RGB_plug = fnNode.findPlug("read_and_writ_RGB", false, &status);
			read_and_writ_RGB_plug.setShort(1);
		}

		if (dataHandle_filePath != "")
		{
			pathFile = dataHandle_filePath.asChar();
			saveToFile(pathFile, readWritRGB, savePCDPLY, saveAsciiBinary);
		}

		else {
			MGlobal::displayInfo("** No File Path To Save! **");
		}

		//-- set SaveToFile Attr to off (deafault) --------------
		MFnDependencyNode fnPrNode(thisNode, &status);
		MPlug saveToFile_plug = fnPrNode.findPlug("SaveToFile", false, &status);
		saveToFile_plug.setShort(0);

		MGlobal::displayInfo("** Save File **");
		//-- call set color -------------------
		setParticleColor(colorR, colorG, colorB, colorControl);
	}


	//=============================================================================================
	//---Set the mesh output data -------------------------
	MDataHandle dataHandle_outputMesh = dataBlock.outputValue(outputMesh_Attr);
	dataHandle_outputMesh.set(newOutputData);

	//-- Update the data block with new dOutput and set plug clean.
	hOut.set(dOutput);
	dataBlock.setClean(plug);


	return(MS::kSuccess);

}

/**
 * Evenly distributes the projection points (and optionally their ray
 * directions) across N worker tasks in preparation for threading.
 */
TaskData scatteringNode::pointsDivideToThreads(TaskData taskData, int numTasks, MFloatPointArray pointsPosArray, MFloatVectorArray pointsVecArray, short eval_state)
{
	MStatus status;
	//MGlobal::displayInfo(MString("** pointsDivideToThreads ") );
	unsigned int numPoints = pointsPosArray.length();
	unsigned int taskLength = (numPoints + numTasks - 1) / numTasks;
	unsigned int startPoint = 0;
	unsigned int endPoint = taskLength;
	unsigned int thisTasks = 0;
	unsigned int count = 0;

	taskData.taskData_pointsPosArray.clear();
	taskData.taskData_pointsVecArray.clear();
	taskData.taskData_pointsArrayToDraw.clear();

	taskData.taskData_pointsPosArray.resize(numTasks);
	if (eval_state == 1) {
		taskData.taskData_pointsVecArray.resize(numTasks);
	}

	taskData.taskData_pointsArrayToDraw.resize(numTasks);
	taskData.taskData_pointsPosArray[thisTasks].clear();
	taskData.taskData_pointsArrayToDraw[thisTasks].clear();

	while (thisTasks < numTasks)
	{

		while (startPoint < endPoint)
		{
			if (count >= numPoints)
			{
				break;
			}
			taskData.taskData_pointsPosArray[thisTasks].append(pointsPosArray[startPoint]);
			if (eval_state == 1) {
				taskData.taskData_pointsVecArray[thisTasks].append(pointsVecArray[startPoint]);
			}

			//-- algorithmType is 2 - no need for directien data - (eval_state == 0) -transfotm change(no deformatin)----
			startPoint = startPoint + 1;
			count = count + 1;
		}

		startPoint = endPoint;
		endPoint = taskLength * (thisTasks + 2);
		thisTasks = thisTasks + 1;

	}

	return taskData;
}


//-- call ThreadData - Here we determine how many Threads And how many points each Thread gets 
//-- From which point index begins and at which point index ends for each Thread --------------------------
/**
 * Allocates and initializes ThreadData structs describing the ranges each
 * worker thread should process.
 */
ThreadData* scatteringNode::createThreadData(int numTasks, TaskData* pTaskData)
{
	//MGlobal::displayInfo(MString("**createThreadData**"));

	ThreadData* pThreadData = new ThreadData[numTasks];
	unsigned int numPoints = pTaskData->points.length();

	unsigned int taskLength = (numPoints + numTasks - 1) / numTasks;
	unsigned int start = 0;
	unsigned int end = taskLength;

	int lastTask = numTasks - 1;

	for (int i = 0; i < numTasks; ++i)
	{
		if (i == lastTask)
		{
			end = numPoints;
		}
		pThreadData[i].start = start;
		pThreadData[i].end = end;
		pThreadData[i].numTasks = numTasks;
		pThreadData[i].pData = pTaskData;
		pThreadData[i].currentTask = i;

		start += taskLength;
		end += taskLength;
	}

	return pThreadData;
}


//--  Get Connected Shader / Colors From Meshs and return it's name    ----------------------------------------
/**
 * Returns the names of shading networks/texture nodes connected to the given mesh.
 */
MStringArray scatteringNode::getConnectedShade(MDagPath dagPath)
{
	MStatus status;
	//MGlobal::displayInfo(MString("** getConnectedShade **") );

	MStringArray shaderName_list;

	//-- set dagPath to mfnMesh -----------------
	MFnMesh mfnMesh;
	mfnMesh.setObject(dagPath);

	//-- Get connected shading node�s name ---------------
	MObjectArray shaders;
	MIntArray indices;
	mfnMesh.getConnectedShaders(0, shaders, indices);
	MString shaderName;
	for (uint i = 0; i < shaders.length(); i++)
	{
		MPlugArray connections;
		MFnDependencyNode shaderGroup(shaders[i]);
		MPlug shaderPlug = shaderGroup.findPlug("surfaceShader");
		shaderPlug.connectedTo(connections, true, true);

		for (uint u = 0; u < connections.length(); u++)
		{
			if (connections[u].node().hasFn(MFn::kLambert))
			{
				MPlugArray plugs;
				MFnLambertShader lambertShader(connections[u].node());

				lambertShader.findPlug("color").connectedTo(plugs, true, true);
				if (plugs.length() > 0)
				{
					for (uint y = 0; y < plugs.length(); y++)
					{
						shaderName = plugs[y].name().asChar();
						shaderName_list.append(shaderName);
						shaderName_list.append("textuer");
					}
				}
				else
				{
					shaderName_list.append(connections[u].name());
					shaderName_list.append("shader");
				}
			}
		}
	}

	return shaderName_list;
}


//-- sample Color From Mesh and return color for each point -------------------------------------------
/**
 * Evaluates a shading network at the provided world-space points to retrieve
 * per-point color information.
 */
MStatus scatteringNode::sampleColorFromMesh(MDagPath dagPath, MFloatPointArray pointsPos, MString shaderName)
{
	//MGlobal::displayInfo(MString("** sampleColorFromMesh ** ") + dagPath.fullPathName() + " " + shaderName);

	MStatus status;

	//-- set dagPath to mfnMesh -----------------
	MFnMesh mfnMesh;
	mfnMesh.setObject(dagPath);

	//-- get UV from positions -------------------
	float2 uvPoint;
	MSpace::Space space = MSpace::kWorld;
	const MString * uvSet = NULL;
	int * closestPolygon = NULL;
	MFloatArray uCoords, vCoords;
	MFloatPointArray   refPoints;
	for (unsigned int i = 0; i < pointsPos.length(); i += 1) {
		MPoint pointPos(pointsPos[i]);
		mfnMesh.getUVAtPoint(pointPos, uvPoint, space, uvSet, closestPolygon);
		uCoords.append((float)uvPoint[0]);
		vCoords.append((float)uvPoint[1]);
		refPoints.append(pointsPos[i]);
	}
	
	//-- call sampleShadingNetwork to geo color from position ------------------------
	int 	numSamples = pointsPos.length();
	bool 	useShadowMaps = FALSE;
	bool 	reuseMaps = FALSE;
	MFloatMatrix   cameraMatrix;
	MFloatVectorArray 	resultColors;
	MFloatVectorArray  	resultTransparencies;

	MRenderUtil::sampleShadingNetwork(shaderName,
		numSamples,
		useShadowMaps,
		reuseMaps,
		cameraMatrix,
		&pointsPos,
		&uCoords,
		&vCoords,
		NULL,
		&refPoints,
		NULL,
		NULL,
		NULL,
		resultColors,
		resultTransparencies);


	//-- append results to sampleColorsMesh array --------------------- 
	for (unsigned int i = 0; i < resultColors.length(); i += 1) {
		sampleColorsMesh_list.append(resultColors[i]);
	}


	return status;

}

/**
 * Maya thread-pool entry point. Creates one task per thread and launches the parallel region.
 */
void scatteringNode::createTasks(void* pData, MThreadRootTask* pRoot)
{
	//MGlobal::displayInfo("** createTasks **");
	ThreadData* pThreadData = (ThreadData*)pData;

	if (pThreadData)
	{
		int numTasks = pThreadData->numTasks;
		for (int i = 0; i < numTasks; ++i)
		{
			MThreadPool::createTask(threadEvaluate, (void*)&pThreadData[i], pRoot);
		}
		MThreadPool::executeAndJoin(pRoot);
	}

}

/**
 * Worker-thread body that performs ray intersection tests and populates point arrays.
 */
MThreadRetVal scatteringNode::threadEvaluate(void *pParam)
{

	MStatus status;
	ThreadData* pThreadData = (ThreadData*)(pParam);
	TaskData* pData = pThreadData->pData;

	//-- gather data for the each thread task ----------------
	std::vector<MFloatPointArray>& pointsPosArray = pData->taskData_pointsPosArray;
	std::vector<MVectorArray>& pointsVecArray = pData->taskData_pointsVecArray;
	std::vector<MFloatPointArray>& pointsArrayToDraw = pData->taskData_pointsArrayToDraw;
	short surfaceVolume = pData->taskData_surfaceVolume;
	short projectionDirection = pData->taskData_projectionDirection;
	unsigned int currentTask = pThreadData->currentTask;
	short evalState = pData->taskData_eval_state;
	double preRandomPos = pData->taskData_preRandomPos;
	double randomPosX = pData->taskData_randomPosX;
	double randomPosY = pData->taskData_randomPosY;
	double randomPosZ = pData->taskData_randomPosZ;

	MFloatPointArray insidePoints;

	if (evalState == 1)
	{
		//-- gather data for the each thread task ----------------
		BoundingBoxData boundingBoxData = pData->taskData_boundingBoxData;

		MDagPath& dagPath = pData->taskData_dagPath;
		int pointsDensity = pData->taskData_pointsDensity;
		bool normalDirection = pData->taskData_normalDirection;

		//--find which length is the minimal ----
		double xlen = boundingBoxData.max_x - boundingBoxData.min_x;
		double ylen = boundingBoxData.max_y - boundingBoxData.min_y;
		double zlen = boundingBoxData.max_z - boundingBoxData.min_z;

		//-- calculate spacing between points  ------------------------------------------
		float borderOffset = boundingBoxData.borderOffset;
		float step = boundingBoxData.step;
		float min_x = boundingBoxData.min_x;
		float min_y = boundingBoxData.min_y;
		float min_z = boundingBoxData.min_z;
		float max_x = boundingBoxData.max_x;
		float max_y = boundingBoxData.max_y;
		float max_z = boundingBoxData.max_z;

		double randX = 0, randY = 0, randZ = 0;

		MFnMesh mfnMesh;
		mfnMesh.setObject(dagPath);
		bool idsSorted = false;
		MSpace::Space space = MSpace::kWorld;
		int maxParam = 999999;
		bool testBothDirections = false;
		bool sortHits = TRUE;
		float tolerance = 0.000001f;
		MFloatPointArray hitPointsArray;
		MFloatArray hitRayParams;
		MIntArray hitFaces;
		MIntArray iaHitTriangles;
		MFloatArray faHitBary1;
		MFloatArray faHitBary2;
		float dotA = 1;
		float dotB = 1;
		MVector normalVector;
		MFloatPoint point;
		MFloatVector rayDirection;

		for (unsigned int i = 0; i < pointsPosArray[currentTask].length(); i += 1) {
			if (pointsPosArray[currentTask].length() != 0) {
				point = pointsPosArray[currentTask][i];
				//-- get the direction -----------------------------
				rayDirection = pointsVecArray[currentTask][i];

				//--------------------------------------------------
				// block of parameters for MFnMesh.allIntersections()
				//--------------------------------------------------
				//-- The Intersections command --------------------------
				hitPointsArray.clear();
				hitRayParams.clear();
				hitFaces.clear();
				iaHitTriangles.clear();
				faHitBary1.clear();
				faHitBary2.clear();

				// Protect Maya's thread-unsafe allIntersections() call with a mutex
				bool hit;
				{
					std::lock_guard<std::mutex> lock(g_intersectionMutex);
					hit = mfnMesh.allIntersections(
						point,
						rayDirection,
						NULL,
						NULL,
						idsSorted,
						space,
						maxParam,
						testBothDirections,
						NULL,
						sortHits,
						hitPointsArray,
						&hitRayParams,
						&hitFaces,
						&iaHitTriangles,
						&faHitBary1,
						&faHitBary2,
						tolerance);
				}

				//-- algorithmType Projection  -----------------
				if (hit == 1) {

					//-- only object surface --------------------------------------
					if (surfaceVolume == 1) {
						for (unsigned int p = 0; p < hitPointsArray.length(); ++p) {
							insidePoints.append(hitPointsArray[p]);
						}
					}

					//-- fill object Volume --------------------------------------
					if (surfaceVolume == 2) {
						if (hitPointsArray.length() % 2 == 0)
						{
							for (unsigned int x = 0; x < hitPointsArray.length(); x += 2) {
								//-- if normalDirection is OFF ------------------------------
								if (normalDirection == 0)
								{
									dotA = -1;
									dotB = 1;
								}
								//-- if normalDirection is On ------------------------------
								if (normalDirection == 1) {
									mfnMesh.getPolygonNormal(hitFaces[x], normalVector, MSpace::kWorld);
									dotA = rayDirection.normal() * normalVector.normal();
									mfnMesh.getPolygonNormal(hitFaces[x + 1], normalVector, MSpace::kWorld);
									dotB = rayDirection.normal() * normalVector.normal();
								}

								if (dotA < 0 && dotB > 0) {
									MPoint inPoint = MPoint(hitPointsArray[x]);
									MPoint outPoint = MPoint(hitPointsArray[x + 1]);
									double inToB_dis = inPoint.distanceTo(outPoint);
									//-- projection Direction X ----------
									if (projectionDirection == 0) {
										for (float x = inPoint.x + inToB_dis * borderOffset; x < outPoint.x; x += step) {
										//-- add PreRandom -------------
										if (preRandomPos > 0)
										{
											std::mt19937& rng = getThreadLocalRNG();
											std::uniform_real_distribution<double> dist(0.0, 1.0);
											double f = dist(rng);
											randX = (f * ((1 - (-1)) + 1) + (-1)) *0.1;
											insidePoints.append((double)inPoint.x + randX, inPoint.y, inPoint.z);
										}
											else
											{
												insidePoints.append(x, inPoint.y, inPoint.z);
											}
										}
									}

									//-- projection Direction Y ----------
									if (projectionDirection == 1) {
										for (float y = inPoint.y + inToB_dis * borderOffset; y < outPoint.y; y += step) {
											//-- add PreRandom --------------
											if (preRandomPos > 0)
											{
												std::mt19937& rng = getThreadLocalRNG();
												std::uniform_real_distribution<double> dist(0.0, 1.0);
												double f = dist(rng);
												randY = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
												insidePoints.append(inPoint.x, (double)inPoint.y + randY, inPoint.z);
											}
											else
											{
												insidePoints.append(inPoint.x, y, inPoint.z);
											}
										}
									}

									//-- projection Direction Z ----------
									if (projectionDirection == 2) {
										for (float z = inPoint.z + inToB_dis * borderOffset; z < outPoint.z; z += step) {
											//-- add PreRandom --------------
											if (preRandomPos > 0)
											{
												std::mt19937& rng = getThreadLocalRNG();
												std::uniform_real_distribution<double> dist(0.0, 1.0);
												double f = dist(rng);
												randZ = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
												insidePoints.append(inPoint.x, inPoint.y, (double)inPoint.z + randZ);
											}
											else
											{
												insidePoints.append(inPoint.x, inPoint.y, z);
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		for (unsigned int p = 0; p < insidePoints.length(); ++p) {
			pointsArrayToDraw[currentTask].append(insidePoints[p]);
		}
	}

	//--------------------------------------------------------------------------------
	//-- move the poins without raycast - only with Transformation ----------------------
	if (evalState == 0)
	{
		float randX = 0, randY = 0, randZ = 0;
		MMatrix matrixHandle = pData->taskData_matrixHandle;
		for (unsigned int i = 0; i < pointsPosArray[currentTask].length(); i += 1) {
			if (pointsPosArray[currentTask].length() != 0) {
				MPoint point = pointsPosArray[currentTask][i];
				MFloatPoint newPointPos = point * matrixHandle;
				//-- Add Post randm position to points ------------------
				if (randomPosX > 0 || randomPosY > 0 || randomPosZ > 0) {
					std::mt19937& rng = getThreadLocalRNG();
					std::uniform_real_distribution<double> dist(0.0, 1.0);
					// --rand x------
					double f = dist(rng);
					randX = (randomPosX*-1) + f * (randomPosX - (randomPosX*-1));
					//-- rand y ------
					f = dist(rng);
					randY = (randomPosY*-1) + f * (randomPosY - (randomPosY*-1));
					//-- rand z ------
					f = dist(rng);
					randZ = (randomPosZ*-1) + f * (randomPosZ - (randomPosZ*-1));

					insidePoints.append(MFloatPoint((double)newPointPos.x + randX, (double)newPointPos.y + randY, (double)newPointPos.z + randZ));
				}
				else {
					insidePoints.append(newPointPos);
				}
			}
		}

		for (unsigned int p = 0; p < insidePoints.length(); ++p) {
			pointsArrayToDraw[currentTask].append(insidePoints[p]);
		}
	}


	return 0;
}


/**
 * Determines whether the provided path refers to a single file or directory
 * and routes to the appropriate load routine (supports batch directory import).
 */
MStatus scatteringNode::path_check(MString pathFile, short readWritRGB)
{
	MStatus status;
	//MGlobal::displayInfo(MString("** path_check *** ") + pathFile);

	namespace bfs = boost::filesystem;
	bool file_or_folder = bfs::is_regular_file(pathFile.asChar());

	colorFromFile_list.clear();

	//-- the path is folder --------------
	if (file_or_folder == false)
	{
		bfs::path apk_path(pathFile.asChar());
		bfs::recursive_directory_iterator end;
		for (bfs::recursive_directory_iterator i(apk_path); i != end; ++i)
		{
			const bfs::path fileInDir = (*i);
			load_PCD_PLY_File(fileInDir.c_str(), readWritRGB, pointArrayPos_global);
		}
	}

	//-- the path is single file --------------
	else
	{
		load_PCD_PLY_File(pathFile, readWritRGB, pointArrayPos_global);
	}
		
	return MS::kSuccess;

}


/**
 * Loads a point-cloud (PCD/PLY) from disk into the provided array using Open3D.
 * Optionally captures RGB color information alongside XYZ positions.
 */
MStatus scatteringNode::load_PCD_PLY_File(MString pathFile, short readWritRGB, MVectorArray& pointArrayPos_global)
{
	MStatus status;

	//MGlobal::displayInfo(MString("** load_PCD_PLY_File fileType -** ") + pathFile);
	
	using namespace open3d;
	auto cloud_ptr = std::make_shared<geometry::PointCloud>();

	io::ReadPointCloud(pathFile.asChar(), *cloud_ptr);
	//-- check number of points ------------------
	int pointAmuont = cloud_ptr->points_.size();
	//-- check if color exsist ------------------
	bool hasColors = cloud_ptr->HasColors();

	//-- with RGB data ----------------------------------------------
	if (readWritRGB == 1) {
		for (unsigned int i = 0; i < pointAmuont; ++i)
		{
			//-- set position --------
			pointArrayPos_global.append(MVector(cloud_ptr->points_[i].x(), cloud_ptr->points_[i].y(), cloud_ptr->points_[i].z()));


			//-- set color --------
			if (hasColors == true) {
				colorFromFile_list.append(MVector(cloud_ptr->colors_[i].x(), cloud_ptr->colors_[i].y(), cloud_ptr->colors_[i].z()));
			}
			else
			{
				colorFromFile_list.append(MVector(0.5, 0.5, 0.5));
			}
		}
	}
	//-- Without RGB data ----------------------------------------------
	if (readWritRGB == 0) {

		for (unsigned int i = 0; i < pointAmuont; ++i)
		{
			//-- set position --------
			pointArrayPos_global.append(MVector(cloud_ptr->points_[i].x(), cloud_ptr->points_[i].y(), cloud_ptr->points_[i].z()));
		}
	}


	return MS::kSuccess;

}


//-- get the particle_sys from particleDriver connection -------------------------------
/**
 * Lazily resolves the MFnParticleSystem connected to this emitter so we can
 * push positions/colors into it during compute().
 */
MStatus scatteringNode::getTheParticleSys()
{
	MStatus status;
	MDagModifier dagMod;
	MFnDependencyNode fnPrNode(thisNode);
	MPlug particle_plug = fnPrNode.findPlug("ParticleDriver", true);
	MPlugArray mPlugArray;
	particle_plug.connectedTo(mPlugArray, false, true);
	if (mPlugArray.length() == 0) {
		return MS::kInvalidParameter;
	}
	MObject particle_obj = mPlugArray[0].node();
	particle_sys.setObject(particle_obj);

	//MTime time(30.0);
	//particle_sys.evaluateDynamics(time, true);
	//particle_sys.saveInitialState();
	return MS::kSuccess;

}

/**
 * Pushes color data into the connected particle system. Depending on the mode,
 * it either uses constant RGB values or sampled colors from the mesh/file.
 */
MStatus scatteringNode::setParticleColor(float colorR, float colorG, float colorB, short colorControl)
{
	MStatus status;

	//MGlobal::displayInfo(MString("** setParticleColor.** ") + colorControl);
	colorsOnParticles_list.clear();

	//-- set color from file (if exist) ---------------
	if (colorControl == 1)
	{
		if (colorFromFile_list.length() > 0)
		{
			particle_sys.setPerParticleAttribute("rgbPP", colorFromFile_list);
		}

		colorsOnParticles_list = colorFromFile_list;
	}


	//-- set color from mesh sampe ---------------
	if (colorControl == 2)
	{
		if (sampleColorsMesh_list.length() > 0)
		{
			particle_sys.setPerParticleAttribute("rgbPP", sampleColorsMesh_list);
		}

		colorsOnParticles_list = sampleColorsMesh_list;
	}


	//-- set color from RGB Attributes ---------------
	if (colorControl == 3)
	{
		MVectorArray colorFromRGB_list;
		for (unsigned int i = 0; i < pointArrayPos_global.length(); ++i)
		{
			MVector color(colorR, colorG, colorB);
			colorFromRGB_list.append(color);
		}

		particle_sys.setPerParticleAttribute("rgbPP", colorFromRGB_list);
		
		colorsOnParticles_list = colorFromRGB_list;
	}


	return MS::kSuccess;
}

/*
MStatus scatteringNode::setMeshColor(float colorR, float colorG, float colorB)
{
	MStatus status;

	//-- if only R G B attributs  ---------------------- 
	if (pointArrayColor.length() == 0)
	{
		MColorArray colorArray;
		MIntArray ver;
		MColor color(colorR, colorG, colorB);
		int index = 0;
		for (unsigned int i = 0; i < pointArrayPos_global.length(); ++i)
		{
			for (unsigned int c = 0; c < 8; ++c)
			{
				colorArray.append(color);
				ver.append(index + c);
			}
			index += 8;
			meshFn_obj.setVertexColors(colorArray, ver);
		}

	}

	return MS::kSuccess;
}
*/

/**
 * Serializes the generated point cloud (positions + optional color/normal data)
 * to disk in either PCD or PLY format using the requested encoding.
 */
MStatus scatteringNode::saveToFile(MString pathFile, short readWritRGB, short savePCDPLY, short saveAsciiBinary)
{
	MStatus status;
	//MGlobal::displayInfo(MString(" saveToFile To: ") + pathFile);

	using namespace open3d;
	geometry::PointCloud pointcloud;

	//--  Vertices position loop -----------------
	for (unsigned int i = 0; i < pointArrayPos_global.length(); ++i)
	{
		pointcloud.points_.push_back(Eigen::Vector3d(pointArrayPos_global[i].x, pointArrayPos_global[i].y, pointArrayPos_global[i].z));
	}

	if (readWritRGB == 1)
	{
		//-- set colors ----------------------
		if (colorsOnParticles_list.length() > 0)
		{
			for (unsigned int i = 0; i < colorsOnParticles_list.length(); ++i)
			{
				pointcloud.colors_.push_back(Eigen::Vector3d(colorsOnParticles_list[i].x, colorsOnParticles_list[i].y, colorsOnParticles_list[i].z));
			}
		}
	}

	//-- set normals ----------------------
	if (vertexNormals_list.length() > 0)
	{
		for (unsigned int i = 0; i < vertexNormals_list.length(); ++i)
		{
			pointcloud.normals_.push_back(Eigen::Vector3d(vertexNormals_list[i].x, vertexNormals_list[i].y, vertexNormals_list[i].z));
		}
	}

	//== saving the data ==================================================
	//-- check file extenion - ply / pcd ---------------
	MString fileType;
	if (savePCDPLY == 0){
		fileType = "pcd";
	}
	else { 
		fileType = "ply"; 
	}

	if (savePCDPLY == 1 && saveAsciiBinary == 2)
	{
		saveAsciiBinary = 1;
	}

	//-- check if extantion exsist (if not create from args) ------------------
	MStringArray splitStr;
	pathFile.split('.', splitStr);

	//-- boost join strings ------------
	std::vector<std::string> list;
	list.push_back(splitStr[0].asChar());
	list.push_back(fileType.asChar());
	std::string thePath = boost::algorithm::join(list, ".");

	MGlobal::displayInfo(MString(" thePath ") + thePath.c_str());

	//-- ascii ------------------------------
	if (saveAsciiBinary == 0) {
		io::WritePointCloud(thePath.c_str(), pointcloud, { true });
	}
	//-- binary -----------------
	if (saveAsciiBinary == 1) {
		io::WritePointCloud(thePath.c_str(), pointcloud, { false, false });
	}
	//-- compressed ------------------
	if (saveAsciiBinary == 2) {
		io::WritePointCloud(thePath.c_str(), pointcloud, { false, true });
	}
	

	return MS::kSuccess;
}

/**
 * Copies the generated point array into the outgoing particle attribute block.
 */
void scatteringNode::emit(MVectorArray &outPosAry)// holding new particles position
{
	//MGlobal::displayInfo(MString("* emit **") + pointArrayPos_global.length());
	outPosAry = pointArrayPos_global;
}



//-- Create a mesh containing one cubic polygon for each voxel in the pVoxelPositions list. 
/**
 * Builds a voxelized mesh representation of the scattered points by instancing
 * cubes for each entry in pointArrayPos_mesh.
 */
void scatteringNode::createScatteringMesh(MVectorArray& pointArrayPos_mesh, MObject& pOutMeshData, float pVoxelWidth)
{
	//MGlobal::displayInfo(MString("** XXXX createScatteringMesh *** ") + pointArrayPos_mesh.length());
	unsigned int numVoxels = pointArrayPos_mesh.length();
	const int numVerticesPerVoxel = 8; //-- a cube has eight vertices --
	const int numPolygonsPerVoxel = 6; //-- a cube has six faces --
	const int numVerticesPerPolygon = 4;  //-- four vertices are required to define a face of a cube ---
	const int numPolygonConnectsPerVoxel = numPolygonsPerVoxel * numVerticesPerPolygon; // 24

	//-- Initialize the required arrays used to create the mesh in MFnMesh.create() --------
	int totalVertices = numVoxels * numVerticesPerVoxel;
	MFloatPointArray vertexArray;
	vertexArray.setLength(totalVertices);
	int vertexIndexOffset = 0;

	int totalPolygons = numVoxels * numPolygonsPerVoxel;
	MIntArray polygonCounts;
	polygonCounts.setLength(totalPolygons);
	int polygonCountsIndexOffset = 0;

	int totalPolygonConnects = numVoxels * numPolygonConnectsPerVoxel;
	MIntArray polygonConnects;
	polygonConnects.setLength(totalPolygonConnects);
	int polygonConnectsIndexOffset = 0;



	//-- Populate the required arrays used in MFnMesh.create() -----
	for (unsigned int i = 0; i < numVoxels; ++i)
	{
		MVector voxelPosition = pointArrayPos_mesh[i];
		//-- Add a new cube to the arrays ---
		createCube(voxelPosition, pVoxelWidth,
			vertexArray, vertexIndexOffset, numVerticesPerVoxel,
			polygonCounts, polygonCountsIndexOffset, numPolygonsPerVoxel, numVerticesPerPolygon,
			polygonConnects, polygonConnectsIndexOffset);


		//-- Increment the respective index offsets.
		vertexIndexOffset += numVerticesPerVoxel;
		polygonCountsIndexOffset += numPolygonsPerVoxel;
		polygonConnectsIndexOffset += numPolygonConnectsPerVoxel;
	}

	//--- Create the mesh now that the arrays have been populated.The mesh is stored in pOutMeshData ----
	MFnMesh meshFn;
	meshFn.create(totalVertices, totalPolygons, vertexArray, polygonCounts, polygonConnects, pOutMeshData);

}


//-- create the Voxel Cubes --------------------------------------------------
/**
 * Appends a cube (voxel) to the mesh data arrays at the requested world-space
 * position. Used internally by createScatteringMesh.
 */
void scatteringNode::createCube(MVector pVoxelPosition, float pWidth,
	MFloatPointArray& pVertexArray, int pVertexIndexOffset, int pNumVerticesPerVoxel,
	MIntArray& pPolygonCountArray, int pPolygonCountIndexOffset, int pNumPolygonsPerVoxel, int pNumVerticesPerPolygon,
	MIntArray& pPolygonConnectsArray, int pPolygonConnectsIndexOffset)
{
	//-- Add a cubic polygon to the specified arrays ----
	//-- We are using half the given width to compute the vertices of the cube.
	float halfWidth = pWidth / 2.0;


	//-- Declare the eight corners of the cube.The cube is centered at pVoxelPosition ---------
	MFloatPointArray vertices;
	MFloatPoint vtx_1(-halfWidth + float(pVoxelPosition.x), -halfWidth + float(pVoxelPosition.y), -halfWidth + float(pVoxelPosition.z)); // 0	
	MFloatPoint vtx_2(halfWidth + float(pVoxelPosition.x), -halfWidth + float(pVoxelPosition.y), -halfWidth + float(pVoxelPosition.z));  // 1
	MFloatPoint vtx_3(halfWidth + float(pVoxelPosition.x), -halfWidth + float(pVoxelPosition.y), halfWidth + float(pVoxelPosition.z));   // 2
	MFloatPoint vtx_4(-halfWidth + float(pVoxelPosition.x), -halfWidth + float(pVoxelPosition.y), halfWidth + float(pVoxelPosition.z));  // 3
	MFloatPoint vtx_5(-halfWidth + float(pVoxelPosition.x), halfWidth + float(pVoxelPosition.y), -halfWidth + float(pVoxelPosition.z));  // 4
	MFloatPoint vtx_6(-halfWidth + float(pVoxelPosition.x), halfWidth + float(pVoxelPosition.y), halfWidth + float(pVoxelPosition.z));   // 5
	MFloatPoint vtx_7(halfWidth + float(pVoxelPosition.x), halfWidth + float(pVoxelPosition.y), halfWidth + float(pVoxelPosition.z));    // 6
	MFloatPoint vtx_8(halfWidth + float(pVoxelPosition.x), halfWidth + float(pVoxelPosition.y), -halfWidth + float(pVoxelPosition.z));   // 7	
	vertices.append(vtx_1);
	vertices.append(vtx_2);
	vertices.append(vtx_3);
	vertices.append(vtx_4);
	vertices.append(vtx_5);
	vertices.append(vtx_6);
	vertices.append(vtx_7);
	vertices.append(vtx_8);

	//-- Set up and array to assign vertices from points to each face 
	MPointArray polygonConnections;
	polygonConnections.append(MPoint(0, 12, 16));
	polygonConnections.append(MPoint(1, 19, 20));
	polygonConnections.append(MPoint(2, 9, 23));
	polygonConnections.append(MPoint(3, 8, 13));
	polygonConnections.append(MPoint(4, 15, 17));
	polygonConnections.append(MPoint(5, 11, 14));
	polygonConnections.append(MPoint(6, 10, 22));
	polygonConnections.append(MPoint(7, 18, 21));

	//MGlobal::displayInfo(MString("** pVertexIndexOffset ** ") + pVertexIndexOffset);

	//-- Store the eight corners of the cube in the vertex array ---
	for (int i = 0; i < pNumVerticesPerVoxel; ++i) //8
	{
		int vtxIndexOff = pVertexIndexOffset + i;
		pVertexArray.set(vertices[i], vtxIndexOff);

		//-- Assign the vertex in the pVertexArray to the relevant polygons --
		MFloatPoint polyCon(polygonConnections[i]);
		pPolygonConnectsArray.set(pVertexIndexOffset + i, pPolygonConnectsIndexOffset + polyCon[0]);
		pPolygonConnectsArray.set(pVertexIndexOffset + i, pPolygonConnectsIndexOffset + polyCon[1]);
		pPolygonConnectsArray.set(pVertexIndexOffset + i, pPolygonConnectsIndexOffset + polyCon[2]);
	}


	//-- Declare the number of vertices for each face -----
	for (unsigned int i = 0; i < pNumPolygonsPerVoxel; ++i)
	{
		//-- Set the number of vertices for the polygon at the given index.
		pPolygonCountArray.set(pNumVerticesPerPolygon, pPolygonCountIndexOffset + i);
	}
}

//-- Generate voxels grid based on bounding box of the mesh 
//and method returning true if voxel is inside mesh. ----
/**
 * Creates the grid of ray start points and directions used to probe the mesh.
 * Depending on projectionDirection, the grid is aligned to the X/Y/Z faces.
 */
ProjectionPointsData scatteringNode::generateProjectionPoints(MDagPath dagPath, short points_density, double preRandomPos, bool intersections, BoundingBoxData boundingBoxData, short projectionDirection)
{
	//MGlobal::displayInfo("** generateProjectionPoints **");
	//-- get lengths of a mesh bounding box ----
	float xlen = boundingBoxData.max_x - boundingBoxData.min_x;
	float ylen = boundingBoxData.max_y - boundingBoxData.min_y;
	float zlen = boundingBoxData.max_z - boundingBoxData.min_z;
	float borderOffset = boundingBoxData.borderOffset;

	ProjectionPointsData projectionPointsData;
	MFloatPointArray raySourcePointsArray;
	MFloatVectorArray rayDirectionPointsArray;


	if (xlen == 0 || ylen == 0 || zlen == 0)
	{
		projectionPointsData.raySourcePointsArray = raySourcePointsArray;
		projectionPointsData.rayDirectionPointsArray = rayDirectionPointsArray;
		return projectionPointsData;
	}

	//-- set ran number ---------------------------------------
	double randX = 0, randY = 0, randZ = 0;

	int div_x;
	int div_z;
	int div_y;
	double minEdge;
	MFloatPoint raySource;
	MFloatVector rayDirection;

	//-- Shoot from X side --------------------
	if (projectionDirection == 0) {
		div_z = points_density;
		minEdge = zlen;
		if (minEdge > ylen)
		{
			minEdge = ylen;
			div_y = points_density;
			div_z = (zlen / ylen) * points_density;
		}
		else {
			minEdge = zlen;
			div_z = points_density;
			div_y = (ylen / zlen) * points_density;
		}

		for (unsigned int i = 0; i < div_z; ++i){
			double z = ((((boundingBoxData.min_z + borderOffset) - (boundingBoxData.max_z - borderOffset)) / (div_z - 1)) * i + (boundingBoxData.max_z - borderOffset));
			for (unsigned int v = 0; v < div_y; ++v) {
				double y = ((((boundingBoxData.min_y + borderOffset) - (boundingBoxData.max_y - borderOffset)) / (div_y - 1)) * v + (boundingBoxData.max_y - borderOffset));
				MFloatPoint pointSource(MVector(MPoint(boundingBoxData.min_x, y, z)));
				MVector pointDir = MVector(MPoint(boundingBoxData.max_x, y, z));

				//-- PreRandom X Plane ------
				if (preRandomPos > 0)
				{
					std::mt19937& rng = getThreadLocalRNG();
					std::uniform_real_distribution<double> dist(0.0, 1.0);
					//-- rand z ------
					double f = dist(rng);
					randZ = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
					//-- rand y ------
					f = dist(rng);
					randY = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
					raySource = MFloatPoint(pointSource.x, (double)pointSource.y + randY, (double)pointSource.z + randZ);
					rayDirection = MVector(pointDir - raySource);
					raySourcePointsArray.append(raySource);
					rayDirectionPointsArray.append(rayDirection);
				}

				else {
					rayDirection = MVector(pointDir - pointSource);
					raySourcePointsArray.append(pointSource);
					rayDirectionPointsArray.append(rayDirection);
				}
			}
		}
	}
	//-- Shoot from Y side --------------------
	if (projectionDirection == 1) {
		div_z = points_density;
		minEdge = zlen;
		if (minEdge > xlen)
		{
			minEdge = xlen;
			div_x = points_density;
			div_z = (zlen / xlen) * points_density;
		}
		else {
			minEdge = zlen;
			div_z = points_density;
			div_x = (xlen / zlen) * points_density;
		}
		for (unsigned int i = 0; i < div_x; ++i){
			double x = ((((boundingBoxData.min_x + borderOffset) - (boundingBoxData.max_x - borderOffset)) / (div_x - 1)) * i + (boundingBoxData.max_x - borderOffset));
			for (unsigned int v = 0; v < div_z; ++v) {
				double z = ((((boundingBoxData.min_z + borderOffset) - (boundingBoxData.max_z - borderOffset)) / (div_z - 1)) * v + (boundingBoxData.max_z - borderOffset));
				MFloatPoint pointSource(MVector(MPoint(x, boundingBoxData.min_y, z)));
				MVector pointDir = MVector(MPoint(x, boundingBoxData.max_y, z));
				//-- PreRandom Z Plane ------
				if (preRandomPos > 0)
				{
					std::mt19937& rng = getThreadLocalRNG();
					std::uniform_real_distribution<double> dist(0.0, 1.0);
					//-- rand x ------
					double f = dist(rng);
					randX = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
					//-- rand y ------
					f = dist(rng);
					randZ = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
					raySource = MFloatPoint((double)pointSource.x + randX, pointSource.y, (double)pointSource.z + randZ);
					rayDirection = MVector(pointDir - pointSource);
					raySourcePointsArray.append(raySource);
					rayDirectionPointsArray.append(rayDirection);
				}
				else
				{
					rayDirection = MVector(pointDir - pointSource);
					raySourcePointsArray.append(pointSource);
					rayDirectionPointsArray.append(rayDirection);
				}
			}
		}
	}
	//-- Shoot from Z side --------------------
	if (projectionDirection == 2) {
		div_y = points_density;
		minEdge = ylen;
		if (minEdge > xlen)
		{
			minEdge = xlen;
			div_x = points_density;
			div_y = (ylen / xlen) * points_density;
		}
		else {
			minEdge = ylen;
			div_y = points_density;
			div_x = (xlen / ylen) * points_density;
		}
		for (unsigned int i = 0; i < div_x; ++i){
			double x = ((((boundingBoxData.min_x + borderOffset) - (boundingBoxData.max_x - borderOffset)) / (div_x - 1)) * i + (boundingBoxData.max_x - borderOffset));
			for (unsigned int v = 0; v < div_y; ++v) {
				double y = ((((boundingBoxData.min_y + borderOffset) - (boundingBoxData.max_y - borderOffset)) / (div_y - 1)) * v + (boundingBoxData.max_y - borderOffset));
				MFloatPoint pointSource(MVector(MPoint(x, y, boundingBoxData.min_z)));
				MVector pointDir = MVector(MPoint(x, y, boundingBoxData.max_z));
				//-- PreRandom Z Plane ------
				if (preRandomPos > 0)
				{
					std::mt19937& rng = getThreadLocalRNG();
					std::uniform_real_distribution<double> dist(0.0, 1.0);
					//-- rand x ------
					double f = dist(rng);
					randX = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
					//-- rand y ------
					f = dist(rng);
					randZ = (preRandomPos*-1) + f * (preRandomPos - (preRandomPos*-1));
					raySource = MFloatPoint((double)pointSource.x + randX, pointSource.y, (double)pointSource.z + randZ);
					rayDirection = MVector(pointDir - pointSource);
					raySourcePointsArray.append(raySource);
					rayDirectionPointsArray.append(rayDirection);
				}
				else
				{
					rayDirection = MVector(pointDir - pointSource);
					raySourcePointsArray.append(pointSource);
					rayDirectionPointsArray.append(rayDirection);
				}
			}
		}
	}

	projectionPointsData.raySourcePointsArray = raySourcePointsArray;
	projectionPointsData.rayDirectionPointsArray = rayDirectionPointsArray;

	return projectionPointsData;
}


/**
 * Maya callback used to mark related plugs dirty when an attribute changes.
 * Helps ensure we recompute only when necessary.
 */
MStatus scatteringNode::setDependentsDirty(const MPlug &plugBeingDirtied, MPlugArray &affectedPlugs)
{
	MStatus status;
	//MGlobal::displayInfo(MString("setDependentsDirty "));

	//-- check if "InputGeoMatrix" is update ---------------------
	if (plugBeingDirtied.partialName() == "InputGeoMatrix") {
		MPlug outArrayGeoMatrixPlug(thisMObject(), targetGeoMatrix_Attr);
		if (plugBeingDirtied.isElement()) {
			// First dirty the output output element first.
			// Of course, dirty output element itself
			MPlug elemPlug = outArrayGeoMatrixPlug.elementByLogicalIndex(plugBeingDirtied.logicalIndex());
			affectedPlugs.append(elemPlug);
			// We also need to dirty the parent.
			affectedPlugs.append(outArrayGeoMatrixPlug);
			eval_state = 0;
		}
	}

	//-- check if "InputGeo" is update ---------------------
	if (plugBeingDirtied.partialName() == "InputGeo") {
		MPlug outArrayMeshPlug(thisMObject(), targetGeo_Attr);
		if (plugBeingDirtied.isElement()) {
			// First dirty the output output element first.
			// Of course, dirty output element itself
			MPlug elemPlug = outArrayMeshPlug.elementByLogicalIndex(plugBeingDirtied.logicalIndex());
			affectedPlugs.append(elemPlug);
			// We also need to dirty the parent.
			affectedPlugs.append(outArrayMeshPlug);
			eval_state = 1;
		}
	}

	//-- check if "Density" is update ---------------------
	if (plugBeingDirtied.partialName() == "Density") {
		eval_state = 1;
	}

	//-- check if "projectionDirection" is update ---------------------
	if (plugBeingDirtied.partialName() == "projectionDirection") {
		eval_state = 1;
	}

	//-- check if "surface_volume" is update ---------------------
	if (plugBeingDirtied.partialName() == "surface_volume") {
		eval_state = 1;
	}

	//-- check if "NormalDirection" is update ---------------------
	if (plugBeingDirtied.partialName() == "normalDirection") {
		eval_state = 1;
	}

	//-- check if "preRandomPos pos" is update ---------------------
	if (plugBeingDirtied.partialName() == "preRandomPos") {
		eval_state = 1;
	}

	//-- check if "Random pos" is update ---------------------
	if (plugBeingDirtied.partialName() == "randomPosX") {
		eval_state = 0;
	}
	if (plugBeingDirtied.partialName() == "randomPosY") {
		eval_state = 0;
	}
	if (plugBeingDirtied.partialName() == "randomPosZ") {
		eval_state = 0;
	}


	return MS::kSuccess;
}



/**
 * Helper that safely jumps to a logical index on a multi attribute array handle.
 */
MStatus scatteringNode::jumpToElement(MArrayDataHandle& hArray, unsigned int index)
{
	MStatus status;
	status = hArray.jumpToElement(index);
	if (MFAIL(status))
	{
		MArrayDataBuilder builder = hArray.builder(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		builder.addElement(index, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = hArray.set(builder);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = hArray.jumpToElement(index);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	return status;
}


MStatus scatteringNode::initialize()
{
	MFnGenericAttribute genericAttr;
	MFnNumericAttribute nAttr;
	MFnEnumAttribute eAttr;
	MFnTypedAttribute typedAttributeFn;
	MFnUnitAttribute unAttribute;
	MFnMatrixAttribute mFnMatrixAtt;
	MFnCompoundAttribute cAttr;

	//-- create points from mesh  ---------------------------------------------------
	pointsCreate_Attr = eAttr.create("PointsCreate", "PointsCreate");
	eAttr.addField("off", 0);
	eAttr.addField("fromMesh", 1);
	eAttr.addField("fromFile", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(pointsCreate_Attr);

	//-- add output Type Display ---------------------------------------------------
	outputTypeDisplay_Attr = eAttr.create("OutputTypeDisplay", "outputTypeDisplay");
	eAttr.addField("particle", 0);
	eAttr.addField("mesh", 1);
	eAttr.addField("off", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(outputTypeDisplay_Attr);

	//-- add Projection Direction ---------------------------------------------------
	projectionDirection_Attr = eAttr.create("projectionDirection", "projectionDirection");
	eAttr.addField("x", 0);
	eAttr.addField("y", 1);
	eAttr.addField("z", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(projectionDirection_Attr);


	//-- add the density Atter ---------------------
	density_Attr = nAttr.create("Density", "Density", MFnNumericData::kInt, 5);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(1);
	addAttribute(density_Attr);

	//-- Pre random pos -------
	preRandomPos_Attr = nAttr.create("preRandomPos", "preRandomPos", MFnNumericData::kDouble, 0.0);
	nAttr.setReadable(false);
	nAttr.setWritable(false);
	nAttr.setStorable(true);
	nAttr.setKeyable(false);
	nAttr.setMin(0);
	nAttr.setMax(1);
	addAttribute(preRandomPos_Attr);

	//-- random x pos -------
	randomPosX_Attr = nAttr.create("randomPosX", "randomPosX", MFnNumericData::kDouble, 0.0);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	addAttribute(randomPosX_Attr);

	//-- random y pos -------
	randomPosY_Attr = nAttr.create("randomPosY", "randomPosY", MFnNumericData::kDouble, 0.0);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	addAttribute(randomPosY_Attr);

	//-- random z pos -------
	randomPosZ_Attr = nAttr.create("randomPosZ", "randomPosZ", MFnNumericData::kDouble, 0.0);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	addAttribute(randomPosZ_Attr);

	//-- add random position compound ------------------------
	randomPos_Attr = cAttr.create("randomPos", "randomPos");
	cAttr.addChild(randomPosX_Attr);
	cAttr.addChild(randomPosY_Attr);
	cAttr.addChild(randomPosZ_Attr);
	cAttr.setReadable(true);
	cAttr.setWritable(true);
	cAttr.setStorable(true);
	cAttr.setKeyable(true);
	addAttribute(randomPos_Attr);


	//-- add the colors  Attr ---------------------
	colorControl_Attr = eAttr.create("colorControl", "colorControl");
	eAttr.addField("colorOff", 0);
	eAttr.addField("colorFromFile", 1);
	eAttr.addField("colorFromMesh", 2);
	eAttr.addField("colorFromRGB", 3);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(colorControl_Attr);

	//-- add the colors R  ---------------------
	colorR_Attr = nAttr.create("colorR", "colorR", MFnNumericData::kFloat, 0);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	nAttr.setMax(1);
	addAttribute(colorR_Attr);

	//-- add the colors G  ---------------------
	colorG_Attr = nAttr.create("colorG", "colorG", MFnNumericData::kFloat, 0);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	nAttr.setMax(1);
	addAttribute(colorG_Attr);

	//-- add the colors B  ---------------------
	colorB_Attr = nAttr.create("colorB", "colorB", MFnNumericData::kFloat, 0);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0);
	nAttr.setMax(1);
	addAttribute(colorB_Attr);


	//-- create normals to save file Attr ---------------------------------------------------
	getNormalsFromMesh_Attr = eAttr.create("GetNormalsFromMesh", "GetNormalsFromMesh");
	eAttr.addField("off", 0);
	eAttr.addField("on", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(getNormalsFromMesh_Attr);


	//-- add the density Atter ---------------------
	voxelWidth_Attr = nAttr.create("voxelWidth", "voxelWidth", MFnNumericData::kFloat, 0.5);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setSoftMin(0.01);
	addAttribute(voxelWidth_Attr);


	//-- save file format -  Binary / Ascii ---------------------------------------------------
	saveAsciiBinary_attr = eAttr.create("Save_Ascii_Binary", "Save_Ascii_Binary");
	eAttr.addField("Ascii", 0);
	eAttr.addField("Binary", 1);
	eAttr.addField("BinaryCompressed", 2);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(saveAsciiBinary_attr);


	//-- save file format - PCD / PLY ---------------------------------------------------
	savePCDPLY_attr = eAttr.create("Save_PCD_PLY", "Save_PCD_PLY");
	eAttr.addField("PCD", 0);
	eAttr.addField("PLY", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(savePCDPLY_attr);

	

	/*
	//-- read and writ RGB to a PCD file Attr ---------------------------------------------------
	readWritRGB_attr = eAttr.create("read_and_writ_RGB", "read_and_writ_RGB");
	eAttr.addField("on", 0);
	eAttr.addField("off", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(readWritRGB_attr);
	*/

	//-- set to save file  ---------------------------------------------------
	saveToFile_Attr = eAttr.create("SaveToFile", "SaveToFile");
	eAttr.addField("off", 0);
	eAttr.addField("Save", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(saveToFile_Attr);


	//-- create Intersections Attr ---------------------------------------------------
	surfaceVolume_Attr = eAttr.create("surface_volume", "surface_volume");
	eAttr.addField("no_intersections", 0);
	eAttr.addField("surface", 1);
	eAttr.addField("volume", 2);
	eAttr.addField("on_vertexs", 3);
	eAttr.setDefault(1);
	eAttr.setReadable(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(surfaceVolume_Attr);


	//-- create Nrmal Diaction For Volume Accuracy check Attr ---------------------------------------------------
	normalDirection_Attr = eAttr.create("NormalDirForVolAccuracy", "NormalDirectionForVolumeAccuracy");
	eAttr.addField("off", 0);
	eAttr.addField("on", 1);
	eAttr.setDefault(0);
	eAttr.setReadable(true);
	eAttr.setChannelBox(true);
	eAttr.setWritable(true);
	eAttr.setStorable(true);
	eAttr.setKeyable(true);
	eAttr.setConnectable(true);
	addAttribute(normalDirection_Attr);


	//-- inColor_Attr -------------------------------
	inColor_Attr = nAttr.create("inColor", "inColor", MFnNumericData::k3Float);
	nAttr.setKeyable(false);
	nAttr.setStorable(false);
	nAttr.setWritable(false);
	nAttr.setChannelBox(false);
	nAttr.setConnectable(true);
	addAttribute(inColor_Attr);


	//-- output number of points ---------------------------------------------------------
	pointNumber_Attr = nAttr.create("NumberOfPoints", "numberOfPoints", MFnNumericData::kInt);
	nAttr.setKeyable(true);
	nAttr.setStorable(false);
	nAttr.setWritable(true);
	nAttr.setChannelBox(true);
	nAttr.setConnectable(true);
	addAttribute(pointNumber_Attr);

	//-- set Mesh outMesh input Attribute ----------
	targetGeo_Attr = genericAttr.create("InputGeo", "InputGeo");
	genericAttr.addAccept(MFnData::kMesh);
	genericAttr.setReadable(false);
	genericAttr.setStorable(false);
	genericAttr.setConnectable(true);
	genericAttr.setArray(true);
	genericAttr.setIndexMatters(false);
	genericAttr.setUsesArrayDataBuilder(true);
	genericAttr.setDisconnectBehavior(MFnAttribute::DisconnectBehavior::kDelete);
	addAttribute(targetGeo_Attr);


	//-- set Mesh target Matrix Atter -----------------------------------------------
	targetGeoMatrix_Attr = mFnMatrixAtt.create("InputGeoMatrix", "InputGeoMatrix");
	mFnMatrixAtt.setReadable(false);
	mFnMatrixAtt.setStorable(false);
	mFnMatrixAtt.setConnectable(true);
	mFnMatrixAtt.setArray(true);
	mFnMatrixAtt.setIndexMatters(false);
	mFnMatrixAtt.setUsesArrayDataBuilder(true);
	addAttribute(targetGeoMatrix_Attr);


	//-- set the file path --------------------------------------------------
	pclFilePath_Attr = typedAttributeFn.create("pointCloudFile", "pcF", MFnData::kString);
	typedAttributeFn.setStorable(true);
	typedAttributeFn.setWritable(true);
	typedAttributeFn.setKeyable(false);
	typedAttributeFn.setReadable(true);
	addAttribute(pclFilePath_Attr);


	//-- output Particle draiver ---------------------------------------------------------
	particleDriver_Attr = unAttribute.create("ParticleDriver", "particleDriver", MFnUnitAttribute::kTime);
	unAttribute.setReadable(true);
	unAttribute.setStorable(false);
	unAttribute.setConnectable(true);
	addAttribute(particleDriver_Attr);


	//-- output Mesh  ---------------------------------------------------
	outputMesh_Attr = typedAttributeFn.create("OutputMesh", "outM", MFnData::kMesh);
	typedAttributeFn.setWritable(true);
	typedAttributeFn.setReadable(true);
	typedAttributeFn.setStorable(false);
	addAttribute(outputMesh_Attr);

	//-- Threading Attr ------------------------------------------------------
	numTasks_Attr = nAttr.create("numTasks", "numTasks", MFnNumericData::kInt, 8);
	nAttr.setMin(1);
	nAttr.setReadable(true);
	nAttr.setWritable(true);
	nAttr.setChannelBox(true);
	addAttribute(numTasks_Attr);



	//--- Attributes Effect  -----------------------------
	attributeAffects(targetGeo_Attr, particleDriver_Attr);
	attributeAffects(density_Attr, pointNumber_Attr);
	attributeAffects(projectionDirection_Attr, pointNumber_Attr);
	attributeAffects(projectionDirection_Attr, particleDriver_Attr);
	attributeAffects(projectionDirection_Attr, outputMesh_Attr);
	attributeAffects(pointsCreate_Attr, particleDriver_Attr);
	attributeAffects(density_Attr, particleDriver_Attr);
	attributeAffects(outputTypeDisplay_Attr, particleDriver_Attr);
	attributeAffects(pointsCreate_Attr, outputMesh_Attr);
	attributeAffects(voxelWidth_Attr, outputMesh_Attr);
	attributeAffects(voxelWidth_Attr, particleDriver_Attr);
	attributeAffects(targetGeo_Attr, outputMesh_Attr);
	attributeAffects(targetGeoMatrix_Attr, outputMesh_Attr);
	attributeAffects(targetGeoMatrix_Attr, particleDriver_Attr);
	attributeAffects(outputTypeDisplay_Attr, outputMesh_Attr);
	attributeAffects(density_Attr, outputMesh_Attr);
	attributeAffects(saveToFile_Attr, outputMesh_Attr);
	attributeAffects(saveToFile_Attr, particleDriver_Attr);
	attributeAffects(colorControl_Attr, particleDriver_Attr);
	attributeAffects(colorControl_Attr, outputMesh_Attr);
	attributeAffects(colorR_Attr, particleDriver_Attr);
	attributeAffects(colorG_Attr, particleDriver_Attr);
	attributeAffects(colorB_Attr, particleDriver_Attr);
	attributeAffects(colorR_Attr, outputMesh_Attr);
	attributeAffects(colorG_Attr, outputMesh_Attr);
	attributeAffects(colorB_Attr, outputMesh_Attr);
	attributeAffects(preRandomPos_Attr, outputMesh_Attr);
	attributeAffects(preRandomPos_Attr, particleDriver_Attr);
	attributeAffects(randomPos_Attr, outputMesh_Attr);
	attributeAffects(randomPos_Attr, particleDriver_Attr);
	attributeAffects(randomPosX_Attr, particleDriver_Attr);
	attributeAffects(randomPosY_Attr, particleDriver_Attr);
	attributeAffects(randomPosZ_Attr, particleDriver_Attr);
	attributeAffects(randomPosX_Attr, outputMesh_Attr);
	attributeAffects(randomPosY_Attr, outputMesh_Attr);
	attributeAffects(randomPosZ_Attr, outputMesh_Attr);
	attributeAffects(surfaceVolume_Attr, outputMesh_Attr);
	attributeAffects(surfaceVolume_Attr, particleDriver_Attr);
	attributeAffects(surfaceVolume_Attr, pointNumber_Attr);
	attributeAffects(normalDirection_Attr, outputMesh_Attr);
	attributeAffects(normalDirection_Attr, particleDriver_Attr);
	attributeAffects(normalDirection_Attr, pointNumber_Attr);
	attributeAffects(numTasks_Attr, outputMesh_Attr);
	attributeAffects(numTasks_Attr, particleDriver_Attr);
	attributeAffects(numTasks_Attr, pointNumber_Attr);
	attributeAffects(savePCDPLY_attr, outputMesh_Attr);
	attributeAffects(savePCDPLY_attr, particleDriver_Attr);



	return(MS::kSuccess);
}