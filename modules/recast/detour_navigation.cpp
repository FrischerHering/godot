#include "detour_navigation.h"
#include "core/list.h"
#include "core/vector.h"
#include "navigation_mesh_generator.h"

DetourNavigation::DetourNavigation() {
	_navmesh_query = dtAllocNavMeshQuery();
	_navmesh_set = false;
}

DetourNavigation::~DetourNavigation() {
	dtFreeNavMeshQuery(_navmesh_query);
}

void DetourNavigation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("find_path", "start_point", "end_point"), &DetourNavigation::find_path);
	ClassDB::bind_method(D_METHOD("set_navigation_mesh", "navigation_mesh"), &DetourNavigation::set_navigation_mesh);
	ClassDB::bind_method(D_METHOD("rebuild_navigation_mesh"), &DetourNavigation::rebuild_navigation_mesh);

	ClassDB::bind_method(D_METHOD("set_geometry_root_node", "root_node"), &DetourNavigation::set_geometry_root_node);
	ClassDB::bind_method(D_METHOD("get_geometry_root_node"), &DetourNavigation::get_geometry_root_node);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "geometry_root_node"), "set_geometry_root_node", "get_geometry_root_node");

	ADD_SIGNAL(MethodInfo("navigation_mesh_changed"));
}

void DetourNavigation::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			rebuild_navigation_mesh();
			break;
	}
}

void DetourNavigation::reset() {
	dtFreeNavMeshQuery(_navmesh_query);
	_navmesh_query = dtAllocNavMeshQuery();
	_navmesh_set = false;
}

bool DetourNavigation::set_navigation_mesh(Ref<DetourNavigationMesh> p_navigation_mesh) {
	ERR_FAIL_COND_V(!p_navigation_mesh->is_valid(), false);

	reset();
	ERR_FAIL_COND_V(_navmesh_query->init(p_navigation_mesh->get_dt_navmesh(), 2048) != DT_SUCCESS, false);

	_navigation_mesh_ref = p_navigation_mesh;
	_navmesh_set = true;
	emit_signal("navigation_mesh_changed");

	return true;
}

bool DetourNavigation::is_valid() const {
	return _navmesh_set;
}

PoolVector<Vector3> DetourNavigation::find_path(Vector3 p_start, Vector3 p_end) {
	PoolVector<Vector3> ret_path;
	ERR_FAIL_COND_V(!is_valid(), ret_path);

	PolySearchResult start = find_nearest_poly(p_start);
	PolySearchResult end = find_nearest_poly(p_end);

	ERR_FAIL_COND_V(!start.success || !end.success, ret_path);

	dtPolyRef poly_path[128];
	int poly_path_length = 0;
	if (_navmesh_query->findPath(start.poly_ref, end.poly_ref, start.point, end.point, &_filter, poly_path, &poly_path_length, 128) != DT_SUCCESS) {
		// if no path can be found, just return empty path without raising error
		// if obstacles block all paths findPath() also fails
		return ret_path;
	}
	float path[3 * 128];
	int path_length = 0;
	ERR_FAIL_COND_V(_navmesh_query->findStraightPath(start.point, end.point, poly_path, poly_path_length, path, 0, 0, &path_length, 128) != DT_SUCCESS, ret_path);

	for (int i = 0; i < path_length; ++i) {
		Vector3 v;
		v.x = path[(i * 3) + 0];
		v.y = path[(i * 3) + 1];
		v.z = path[(i * 3) + 2];
		ret_path.append(v);
	}

	return ret_path;
}

Ref<DetourNavigationMesh> DetourNavigation::get_navigation_mesh() const {
	return _navigation_mesh_ref;
}

dtNavMeshQuery *DetourNavigation::get_dt_navmesh_query() {
	return _navmesh_query;
}

PolySearchResult DetourNavigation::find_nearest_poly(Vector3 p_position) {
	PolySearchResult ret;
	ERR_FAIL_COND_V(!is_valid(), ret);

	float position[] = { p_position.x, p_position.y, p_position.z };
	float half_extents[] = { 40.f, 40.f, 40.f };

	ERR_FAIL_COND_V(_navmesh_query->findNearestPoly(position, half_extents, &_filter, &ret.poly_ref, ret.point) != DT_SUCCESS, ret);
	ERR_FAIL_COND_V(ret.poly_ref == 0, ret);

	ret.success = true;
	return ret;
}

bool DetourNavigation::rebuild_navigation_mesh() {
	static NavigationGenerator navgen;
	// TODO: better way for navgen parameters
	Ref<NavigationMesh> godot_navmesh(memnew(NavigationMesh));
	Ref<DetourNavigationMesh> navmesh = navgen.generate_mesh(godot_navmesh, get_node(geometry_root_node_path));
	// TODO: emit signal about rebuilt navigation
	return set_navigation_mesh(navmesh);
}

// ////////////////////////////////////////////////////////////////////////////

void DetourNavigation::set_geometry_root_node(NodePath p_node) {
	geometry_root_node_path = p_node;
}

NodePath DetourNavigation::get_geometry_root_node() const {
	return geometry_root_node_path;
}