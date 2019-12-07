#include "navigation_mesh_data.h"

NavigationMeshData::NavigationMeshData() {
	_polymesh = rcAllocPolyMesh();
	_meshdetail = rcAllocPolyMeshDetail();
	_create_params = memnew(dtNavMeshCreateParams());
	memset(_create_params, 0, sizeof(*_create_params));
}

NavigationMeshData::~NavigationMeshData() {
	rcFreePolyMesh(_polymesh);
	rcFreePolyMeshDetail(_meshdetail);
	memfree(_create_params);
}

rcPolyMesh *NavigationMeshData::get_polymesh() const {
	return _polymesh;
}
rcPolyMeshDetail *NavigationMeshData::get_meshdetail() const {
	return _meshdetail;
}

void NavigationMeshData::reset() {
	rcFreePolyMesh(_polymesh);
	rcFreePolyMeshDetail(_meshdetail);
	_polymesh = rcAllocPolyMesh();
	_meshdetail = rcAllocPolyMeshDetail();
}

dtNavMeshCreateParams *NavigationMeshData::get_create_params() const {
	return _create_params;
}