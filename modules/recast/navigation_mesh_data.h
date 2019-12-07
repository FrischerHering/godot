#ifndef NAVIGATION_MESH_DATA_H
#define NAVIGATION_MESH_DATA_H

#include "core/resource.h"

#include <DetourNavMeshBuilder.h>
#include <Recast.h>

class NavigationMeshData : public Resource {
	GDCLASS(NavigationMeshData, Resource);

protected:
	rcPolyMesh *_polymesh;
	rcPolyMeshDetail *_meshdetail;

	dtNavMeshCreateParams *_create_params;

public:
	NavigationMeshData();
	~NavigationMeshData();

	rcPolyMesh *get_polymesh() const;
	rcPolyMeshDetail *get_meshdetail() const;
	void reset();

	dtNavMeshCreateParams *get_create_params() const;
};

#endif // NAVIGATION_MESH_DATA_H