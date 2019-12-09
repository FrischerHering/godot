#ifndef DETOUR_NAVIGATION_H
#define DETOUR_NAVIGATION_H

#include "core/array.h"
#include "detour_navigation_mesh.h"
#include "scene/3d/spatial.h"

#include <DetourNavMeshQuery.h>

struct PolySearchResult {
	dtPolyRef poly_ref = 0;
	float point[3] = { 0.f, 0.f, 0.f };
	bool success = false;

	PolySearchResult() = default;
};

class DetourNavigation : public Spatial {
	GDCLASS(DetourNavigation, Spatial);

	Ref<DetourNavigationMesh> _navigation_mesh_ref;

	dtNavMeshQuery *_navmesh_query;
	bool _navmesh_set;

	dtQueryFilter _filter;

protected:
	static void _bind_methods();
	void _notification(int p_what);

	NodePath geometry_root_node_path;

public:
	DetourNavigation();
	~DetourNavigation();

	void reset();

	bool set_navigation_mesh(Ref<DetourNavigationMesh> p_navigation_mesh);
	bool is_valid() const;

	PoolVector<Vector3> find_path(Vector3 p_start, Vector3 p_end);

	Ref<DetourNavigationMesh> get_navigation_mesh() const;
	dtNavMeshQuery *get_dt_navmesh_query();
	PolySearchResult find_nearest_poly(Vector3 p_position);

	bool rebuild_navigation_mesh();

	// ////////////////////////////////////////////////////////////////////////

	void set_geometry_root_node(NodePath node_path);
	NodePath get_geometry_root_node() const;
};

#endif // DETOUR_NAVIGATION_H