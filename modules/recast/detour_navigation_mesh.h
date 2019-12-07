#ifndef DETOUR_NAVIGATION_MESH_H
#define DETOUR_NAVIGATION_MESH_H

#include "core/resource.h"
#include "navigation_mesh_data.h"

#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <Recast.h>

class DetourNavigationMesh : public Resource {
	GDCLASS(DetourNavigationMesh, Resource);

protected:
	static void _bind_methods();

	Ref<NavigationMeshData> _data_ref;

	dtNavMesh *_dtNavMesh;
	bool _is_valid;

public:
	enum PolyFlags {
		POLYFLAGS_WALK = 0x01 // Ability to walk (ground, grass, road)
	};

	DetourNavigationMesh();
	~DetourNavigationMesh();

	void reset();

	dtNavMesh *get_dt_navmesh() const;
	bool is_valid() const;

	void set_data(Ref<NavigationMeshData> p_data);
};

#endif // DETOUR_NAVIGATION_MESH_H