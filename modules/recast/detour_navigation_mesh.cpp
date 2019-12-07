#include "detour_navigation_mesh.h"

DetourNavigationMesh::DetourNavigationMesh() {
	_dtNavMesh = dtAllocNavMesh();
	_is_valid = false;
}

DetourNavigationMesh::~DetourNavigationMesh() {
	dtFreeNavMesh(_dtNavMesh);
}

void DetourNavigationMesh::reset() {
	dtFreeNavMesh(_dtNavMesh);
	_dtNavMesh = dtAllocNavMesh();
	_is_valid = false;
}

void DetourNavigationMesh::_bind_methods() {
	ClassDB::bind_method(D_METHOD("is_valid"), &DetourNavigationMesh::is_valid);
	ClassDB::bind_method(D_METHOD("set_data", "navmesh_data"), &DetourNavigationMesh::set_data);
}

dtNavMesh *DetourNavigationMesh::get_dt_navmesh() const {
	return _dtNavMesh;
}

bool DetourNavigationMesh::is_valid() const {
	return _is_valid;
}

void DetourNavigationMesh::set_data(Ref<NavigationMeshData> p_data) {
	reset();

	unsigned char *data;
	int data_size = 0;
	ERR_FAIL_COND(!dtCreateNavMeshData(p_data->get_create_params(), &data, &data_size));
	ERR_FAIL_COND(data_size == 0);
	ERR_FAIL_COND(_dtNavMesh->init(data, data_size, DT_TILE_FREE_DATA) != DT_SUCCESS);

	_data_ref = p_data;
	_is_valid = true;
}