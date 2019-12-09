/*************************************************************************/
/*  navigation_mesh_generator.cpp                                        */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#include "navigation_mesh_generator.h"
#include "core/math/quick_hull.h"
#include "core/os/thread.h"
#include "editor/editor_settings.h"
#include "scene/3d/collision_shape.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/physics_body.h"
#include "scene/resources/box_shape.h"
#include "scene/resources/capsule_shape.h"
#include "scene/resources/concave_polygon_shape.h"
#include "scene/resources/convex_polygon_shape.h"
#include "scene/resources/cylinder_shape.h"
#include "scene/resources/plane_shape.h"
#include "scene/resources/primitive_meshes.h"
#include "scene/resources/shape.h"
#include "scene/resources/sphere_shape.h"

#ifdef MODULE_CSG_ENABLED
#include "modules/csg/csg_shape.h"
#endif

#ifdef MODULE_GRIDMAP_ENABLED
#include "modules/gridmap/grid_map.h"
#endif

EditorNavigationMeshGenerator *EditorNavigationMeshGenerator::singleton = NULL;

void NavigationGenerator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("generate_data", "nav_mesh", "root_node"), &NavigationGenerator::generate_data);
	ClassDB::bind_method(D_METHOD("generate_mesh", "nav_mesh", "root_node"), &NavigationGenerator::generate_mesh);
	ClassDB::bind_method(D_METHOD("generate", "nav_mesh", "root_node"), &NavigationGenerator::generate);
}

void NavigationGenerator::_add_vertex(const Vector3 &p_vec3, Vector<float> &p_verticies) {
	p_verticies.push_back(p_vec3.x);
	p_verticies.push_back(p_vec3.y);
	p_verticies.push_back(p_vec3.z);
}

void NavigationGenerator::_add_mesh(const Ref<Mesh> &p_mesh, const Transform &p_xform, Vector<float> &p_verticies, Vector<int> &p_indices) {
	int current_vertex_count = 0;

	for (int i = 0; i < p_mesh->get_surface_count(); i++) {
		current_vertex_count = p_verticies.size() / 3;

		if (p_mesh->surface_get_primitive_type(i) != Mesh::PRIMITIVE_TRIANGLES)
			continue;

		int index_count = 0;
		if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX) {
			index_count = p_mesh->surface_get_array_index_len(i);
		} else {
			index_count = p_mesh->surface_get_array_len(i);
		}

		ERR_CONTINUE((index_count == 0 || (index_count % 3) != 0));

		int face_count = index_count / 3;

		Array a = p_mesh->surface_get_arrays(i);

		PoolVector<Vector3> mesh_vertices = a[Mesh::ARRAY_VERTEX];
		PoolVector<Vector3>::Read vr = mesh_vertices.read();

		if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX) {

			PoolVector<int> mesh_indices = a[Mesh::ARRAY_INDEX];
			PoolVector<int>::Read ir = mesh_indices.read();

			for (int j = 0; j < mesh_vertices.size(); j++) {
				_add_vertex(p_xform.xform(vr[j]), p_verticies);
			}

			for (int j = 0; j < face_count; j++) {
				// CCW
				p_indices.push_back(current_vertex_count + (ir[j * 3 + 0]));
				p_indices.push_back(current_vertex_count + (ir[j * 3 + 2]));
				p_indices.push_back(current_vertex_count + (ir[j * 3 + 1]));
			}
		} else {
			face_count = mesh_vertices.size() / 3;
			for (int j = 0; j < face_count; j++) {
				_add_vertex(p_xform.xform(vr[j * 3 + 0]), p_verticies);
				_add_vertex(p_xform.xform(vr[j * 3 + 2]), p_verticies);
				_add_vertex(p_xform.xform(vr[j * 3 + 1]), p_verticies);

				p_indices.push_back(current_vertex_count + (j * 3 + 0));
				p_indices.push_back(current_vertex_count + (j * 3 + 1));
				p_indices.push_back(current_vertex_count + (j * 3 + 2));
			}
		}
	}
}

void NavigationGenerator::_add_faces(const PoolVector3Array &p_faces, const Transform &p_xform, Vector<float> &p_verticies, Vector<int> &p_indices) {
	int face_count = p_faces.size() / 3;
	int current_vertex_count = p_verticies.size() / 3;

	for (int j = 0; j < face_count; j++) {
		_add_vertex(p_xform.xform(p_faces[j * 3 + 0]), p_verticies);
		_add_vertex(p_xform.xform(p_faces[j * 3 + 1]), p_verticies);
		_add_vertex(p_xform.xform(p_faces[j * 3 + 2]), p_verticies);

		p_indices.push_back(current_vertex_count + (j * 3 + 0));
		p_indices.push_back(current_vertex_count + (j * 3 + 2));
		p_indices.push_back(current_vertex_count + (j * 3 + 1));
	}
}

void NavigationGenerator::_parse_geometry(Transform p_accumulated_transform, Node *p_node, Vector<float> &p_verticies, Vector<int> &p_indices, int p_generate_from, uint32_t p_collision_mask, bool p_recurse_children) {

	if (Object::cast_to<MeshInstance>(p_node) && p_generate_from != NavigationMesh::PARSED_GEOMETRY_STATIC_COLLIDERS) {

		MeshInstance *mesh_instance = Object::cast_to<MeshInstance>(p_node);
		Ref<Mesh> mesh = mesh_instance->get_mesh();
		if (mesh.is_valid()) {
			_add_mesh(mesh, p_accumulated_transform * mesh_instance->get_transform(), p_verticies, p_indices);
		}
	}

#ifdef MODULE_CSG_ENABLED
	if (Object::cast_to<CSGShape>(p_node) && p_generate_from != NavigationMesh::PARSED_GEOMETRY_STATIC_COLLIDERS) {

		CSGShape *csg_shape = Object::cast_to<CSGShape>(p_node);
		Array meshes = csg_shape->get_meshes();
		if (!meshes.empty()) {
			Ref<Mesh> mesh = meshes[1];
			if (mesh.is_valid()) {
				_add_mesh(mesh, p_accumulated_transform * csg_shape->get_transform(), p_verticies, p_indices);
			}
		}
	}
#endif

	if (Object::cast_to<StaticBody>(p_node) && p_generate_from != NavigationMesh::PARSED_GEOMETRY_MESH_INSTANCES) {
		StaticBody *static_body = Object::cast_to<StaticBody>(p_node);

		if (static_body->get_collision_layer() & p_collision_mask) {

			for (int i = 0; i < p_node->get_child_count(); ++i) {
				Node *child = p_node->get_child(i);
				if (Object::cast_to<CollisionShape>(child)) {
					CollisionShape *col_shape = Object::cast_to<CollisionShape>(child);

					Transform transform = p_accumulated_transform * static_body->get_transform() * col_shape->get_transform();

					Ref<Mesh> mesh;
					Ref<Shape> s = col_shape->get_shape();

					BoxShape *box = Object::cast_to<BoxShape>(*s);
					if (box) {
						Ref<CubeMesh> cube_mesh;
						cube_mesh.instance();
						cube_mesh->set_size(box->get_extents() * 2.0);
						mesh = cube_mesh;
					}

					CapsuleShape *capsule = Object::cast_to<CapsuleShape>(*s);
					if (capsule) {
						Ref<CapsuleMesh> capsule_mesh;
						capsule_mesh.instance();
						capsule_mesh->set_radius(capsule->get_radius());
						capsule_mesh->set_mid_height(capsule->get_height() / 2.0);
						mesh = capsule_mesh;
					}

					CylinderShape *cylinder = Object::cast_to<CylinderShape>(*s);
					if (cylinder) {
						Ref<CylinderMesh> cylinder_mesh;
						cylinder_mesh.instance();
						cylinder_mesh->set_height(cylinder->get_height());
						cylinder_mesh->set_bottom_radius(cylinder->get_radius());
						cylinder_mesh->set_top_radius(cylinder->get_radius());
						mesh = cylinder_mesh;
					}

					SphereShape *sphere = Object::cast_to<SphereShape>(*s);
					if (sphere) {
						Ref<SphereMesh> sphere_mesh;
						sphere_mesh.instance();
						sphere_mesh->set_radius(sphere->get_radius());
						sphere_mesh->set_height(sphere->get_radius() * 2.0);
						mesh = sphere_mesh;
					}

					ConcavePolygonShape *concave_polygon = Object::cast_to<ConcavePolygonShape>(*s);
					if (concave_polygon) {
						_add_faces(concave_polygon->get_faces(), transform, p_verticies, p_indices);
					}

					ConvexPolygonShape *convex_polygon = Object::cast_to<ConvexPolygonShape>(*s);
					if (convex_polygon) {
						Vector<Vector3> varr = Variant(convex_polygon->get_points());
						Geometry::MeshData md;

						Error err = QuickHull::build(varr, md);

						if (err == OK) {
							PoolVector3Array faces;

							for (int j = 0; j < md.faces.size(); ++j) {
								Geometry::MeshData::Face face = md.faces[j];

								for (int k = 2; k < face.indices.size(); ++k) {
									faces.push_back(md.vertices[face.indices[0]]);
									faces.push_back(md.vertices[face.indices[k - 1]]);
									faces.push_back(md.vertices[face.indices[k]]);
								}
							}

							_add_faces(faces, transform, p_verticies, p_indices);
						}
					}

					if (mesh.is_valid()) {
						_add_mesh(mesh, transform, p_verticies, p_indices);
					}
				}
			}
		}
	}

#ifdef MODULE_GRIDMAP_ENABLED
	if (Object::cast_to<GridMap>(p_node) && p_generate_from != NavigationMesh::PARSED_GEOMETRY_STATIC_COLLIDERS) {
		GridMap *gridmap_instance = Object::cast_to<GridMap>(p_node);
		Array meshes = gridmap_instance->get_meshes();
		Transform xform = gridmap_instance->get_transform();
		for (int i = 0; i < meshes.size(); i += 2) {
			Ref<Mesh> mesh = meshes[i + 1];
			if (mesh.is_valid()) {
				_add_mesh(mesh, p_accumulated_transform * xform * meshes[i], p_verticies, p_indices);
			}
		}
	}
#endif

	if (Object::cast_to<Spatial>(p_node)) {
		Spatial *spatial = Object::cast_to<Spatial>(p_node);
		p_accumulated_transform = p_accumulated_transform * spatial->get_transform();
	}

	if (p_recurse_children) {
		for (int i = 0; i < p_node->get_child_count(); i++) {
			_parse_geometry(p_accumulated_transform, p_node->get_child(i), p_verticies, p_indices, p_generate_from, p_collision_mask, p_recurse_children);
		}
	}
}

void NavigationGenerator::_build_recast_navigation_mesh(Ref<NavigationMesh> p_nav_mesh, Ref<NavigationMeshData> p_nav_mesh_data,
		Vector<float> &vertices, Vector<int> &indices) {
	rcContext ctx;

	const float *verts = vertices.ptr();
	const int nverts = vertices.size() / 3;
	const int *tris = indices.ptr();
	const int ntris = indices.size() / 3;

	float bmin[3], bmax[3];
	rcCalcBounds(verts, nverts, bmin, bmax);

	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.cs = p_nav_mesh->get_cell_size();
	cfg.ch = p_nav_mesh->get_cell_height();
	cfg.walkableSlopeAngle = p_nav_mesh->get_agent_max_slope();
	cfg.walkableHeight = (int)Math::ceil(p_nav_mesh->get_agent_height() / cfg.ch);
	cfg.walkableClimb = (int)Math::floor(p_nav_mesh->get_agent_max_climb() / cfg.ch);
	cfg.walkableRadius = (int)Math::ceil(p_nav_mesh->get_agent_radius() / cfg.cs);
	cfg.maxEdgeLen = (int)(p_nav_mesh->get_edge_max_length() / p_nav_mesh->get_cell_size());
	cfg.maxSimplificationError = p_nav_mesh->get_edge_max_error();
	cfg.minRegionArea = (int)(p_nav_mesh->get_region_min_size() * p_nav_mesh->get_region_min_size());
	cfg.mergeRegionArea = (int)(p_nav_mesh->get_region_merge_size() * p_nav_mesh->get_region_merge_size());
	cfg.maxVertsPerPoly = (int)p_nav_mesh->get_verts_per_poly();
	cfg.detailSampleDist = p_nav_mesh->get_detail_sample_distance() < 0.9f ? 0 : p_nav_mesh->get_cell_size() * p_nav_mesh->get_detail_sample_distance();
	cfg.detailSampleMaxError = p_nav_mesh->get_cell_height() * p_nav_mesh->get_detail_sample_max_error();

	cfg.bmin[0] = bmin[0];
	cfg.bmin[1] = bmin[1];
	cfg.bmin[2] = bmin[2];
	cfg.bmax[0] = bmax[0];
	cfg.bmax[1] = bmax[1];
	cfg.bmax[2] = bmax[2];

	rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

	rcHeightfield *hf = rcAllocHeightfield();

	ERR_FAIL_COND(!hf);
	ERR_FAIL_COND(!rcCreateHeightfield(&ctx, *hf, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch));

	{
		Vector<unsigned char> tri_areas;
		tri_areas.resize(ntris);

		ERR_FAIL_COND(tri_areas.size() == 0);

		memset(tri_areas.ptrw(), 0, ntris * sizeof(unsigned char));
		rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, verts, nverts, tris, ntris, tri_areas.ptrw());

		ERR_FAIL_COND(!rcRasterizeTriangles(&ctx, verts, nverts, tris, tri_areas.ptr(), ntris, *hf, cfg.walkableClimb));
	}

	if (p_nav_mesh->get_filter_low_hanging_obstacles())
		rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf);
	if (p_nav_mesh->get_filter_ledge_spans())
		rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
	if (p_nav_mesh->get_filter_walkable_low_height_spans())
		rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf);

	rcCompactHeightfield *chf = rcAllocCompactHeightfield();

	ERR_FAIL_COND(!chf);
	ERR_FAIL_COND(!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf));

	rcFreeHeightField(hf);
	hf = 0;

	ERR_FAIL_COND(!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf));

	if (p_nav_mesh->get_sample_partition_type() == NavigationMesh::SAMPLE_PARTITION_WATERSHED) {
		ERR_FAIL_COND(!rcBuildDistanceField(&ctx, *chf));
		ERR_FAIL_COND(!rcBuildRegions(&ctx, *chf, 0, cfg.minRegionArea, cfg.mergeRegionArea));
	} else if (p_nav_mesh->get_sample_partition_type() == NavigationMesh::SAMPLE_PARTITION_MONOTONE) {
		ERR_FAIL_COND(!rcBuildRegionsMonotone(&ctx, *chf, 0, cfg.minRegionArea, cfg.mergeRegionArea));
	} else {
		ERR_FAIL_COND(!rcBuildLayerRegions(&ctx, *chf, 0, cfg.minRegionArea));
	}

	rcContourSet *cset = rcAllocContourSet();

	ERR_FAIL_COND(!cset);
	ERR_FAIL_COND(!rcBuildContours(&ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset));

	rcPolyMesh *poly_mesh = p_nav_mesh_data->get_polymesh();
	ERR_FAIL_COND(!poly_mesh);
	ERR_FAIL_COND(!rcBuildPolyMesh(&ctx, *cset, cfg.maxVertsPerPoly, *poly_mesh));

	rcPolyMeshDetail *detail_mesh = p_nav_mesh_data->get_meshdetail();
	ERR_FAIL_COND(!detail_mesh);
	ERR_FAIL_COND(!rcBuildPolyMeshDetail(&ctx, *poly_mesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *detail_mesh));

	// Update poly flags from areas.
	for (int i = 0; i < poly_mesh->npolys; ++i) {
		if (poly_mesh->areas[i] == RC_WALKABLE_AREA) {
			poly_mesh->flags[i] = DetourNavigationMesh::POLYFLAGS_WALK;
		}
	}

	dtNavMeshCreateParams *create_params = p_nav_mesh_data->get_create_params();
	create_params->verts = poly_mesh->verts;
	create_params->vertCount = poly_mesh->nverts;
	create_params->polys = poly_mesh->polys;
	create_params->polyFlags = poly_mesh->flags;
	create_params->polyAreas = poly_mesh->areas;
	create_params->polyCount = poly_mesh->npolys;
	create_params->nvp = poly_mesh->nvp;
	create_params->detailMeshes = detail_mesh->meshes;
	create_params->detailVerts = detail_mesh->verts;
	create_params->detailVertsCount = detail_mesh->nverts;
	create_params->detailTris = detail_mesh->tris;
	create_params->detailTriCount = detail_mesh->ntris;
	create_params->walkableHeight = cfg.walkableHeight;
	create_params->walkableRadius = cfg.walkableRadius;
	create_params->walkableClimb = cfg.walkableClimb;
	rcVcopy(create_params->bmin, poly_mesh->bmin);
	rcVcopy(create_params->bmax, poly_mesh->bmax);
	create_params->cs = cfg.cs;
	create_params->ch = cfg.ch;
	create_params->buildBvTree = true;

	rcFreeCompactHeightfield(chf);
	chf = 0;
	rcFreeContourSet(cset);
	cset = 0;
}

void EditorNavigationMeshGenerator::_convert_detail_mesh_to_native_navigation_mesh(const rcPolyMeshDetail *p_detail_mesh, Ref<NavigationMesh> p_nav_mesh) {

	PoolVector<Vector3> nav_vertices;

	for (int i = 0; i < p_detail_mesh->nverts; i++) {
		const float *v = &p_detail_mesh->verts[i * 3];
		nav_vertices.append(Vector3(v[0], v[1], v[2]));
	}
	p_nav_mesh->set_vertices(nav_vertices);

	for (int i = 0; i < p_detail_mesh->nmeshes; i++) {
		const unsigned int *m = &p_detail_mesh->meshes[i * 4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const unsigned int ntris = m[3];
		const unsigned char *tris = &p_detail_mesh->tris[btris * 4];
		for (unsigned int j = 0; j < ntris; j++) {
			Vector<int> nav_indices;
			nav_indices.resize(3);
			// Polygon order in recast is opposite than godot's
			nav_indices.write[0] = ((int)(bverts + tris[j * 4 + 0]));
			nav_indices.write[1] = ((int)(bverts + tris[j * 4 + 2]));
			nav_indices.write[2] = ((int)(bverts + tris[j * 4 + 1]));
			p_nav_mesh->add_polygon(nav_indices);
		}
	}
}

EditorNavigationMeshGenerator *EditorNavigationMeshGenerator::get_singleton() {
	return singleton;
}

EditorNavigationMeshGenerator::EditorNavigationMeshGenerator() {
	singleton = this;
}

EditorNavigationMeshGenerator::~EditorNavigationMeshGenerator() {
}

Ref<NavigationMeshData> NavigationGenerator::generate_data(Ref<NavigationMesh> p_nav_mesh, Node *p_node) {
	ERR_FAIL_COND_V(!p_nav_mesh.is_valid(), NULL);

	Vector<float> vertices;
	Vector<int> indices;

	List<Node *> parse_nodes;

	if (p_nav_mesh->get_source_geometry_mode() == NavigationMesh::SOURCE_GEOMETRY_NAVMESH_CHILDREN) {
		parse_nodes.push_back(p_node);
	} else {
		p_node->get_tree()->get_nodes_in_group(p_nav_mesh->get_source_group_name(), &parse_nodes);
	}

	Transform navmesh_xform = Object::cast_to<Spatial>(p_node)->get_transform().affine_inverse();
	for (const List<Node *>::Element *E = parse_nodes.front(); E; E = E->next()) {
		int geometry_type = p_nav_mesh->get_parsed_geometry_type();
		uint32_t collision_mask = p_nav_mesh->get_collision_mask();
		bool recurse_children = p_nav_mesh->get_source_geometry_mode() != NavigationMesh::SOURCE_GEOMETRY_GROUPS_EXPLICIT;
		_parse_geometry(navmesh_xform, E->get(), vertices, indices, geometry_type, collision_mask, recurse_children);
	}

	Ref<NavigationMeshData> navigation_mesh_data(memnew(NavigationMeshData));
	if (vertices.size() > 0 && indices.size() > 0) {
		_build_recast_navigation_mesh(p_nav_mesh, navigation_mesh_data, vertices, indices);
		return navigation_mesh_data;
	}

	return NULL;
}

Ref<DetourNavigationMesh> NavigationGenerator::generate_mesh(Ref<NavigationMesh> p_nav_mesh, Node *p_node) {
	ERR_FAIL_COND_V(!p_nav_mesh.is_valid(), NULL);
	Ref<NavigationMeshData> data = generate_data(p_nav_mesh, p_node);
	ERR_FAIL_COND_V(!data.is_valid(), NULL);
	Ref<DetourNavigationMesh> navigation_mesh(memnew(DetourNavigationMesh));
	navigation_mesh->set_data(data);
	return navigation_mesh;
}

Node *NavigationGenerator::generate(Ref<NavigationMesh> p_nav_mesh, Node *p_node) {
	ERR_FAIL_COND_V(!p_nav_mesh.is_valid(), NULL);
	Ref<DetourNavigationMesh> navigation_mesh = generate_mesh(p_nav_mesh, p_node);
	ERR_FAIL_COND_V(!navigation_mesh->is_valid(), NULL);
	DetourNavigation *navigation = memnew(DetourNavigation);
	navigation->set_navigation_mesh(navigation_mesh);
	return navigation;
}

void EditorNavigationMeshGenerator::bake(Ref<NavigationMesh> p_nav_mesh, Node *p_node) {
	static NavigationGenerator generator;
	Ref<NavigationMeshData> data = generator.generate_data(p_nav_mesh, p_node);
	ERR_FAIL_COND(!data.is_valid());
	_convert_detail_mesh_to_native_navigation_mesh(data->get_meshdetail(), p_nav_mesh);
}

void EditorNavigationMeshGenerator::clear(Ref<NavigationMesh> p_nav_mesh) {
	if (p_nav_mesh.is_valid()) {
		p_nav_mesh->clear_polygons();
		p_nav_mesh->set_vertices(PoolVector<Vector3>());
	}
}

void EditorNavigationMeshGenerator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("bake", "nav_mesh", "root_node"), &EditorNavigationMeshGenerator::bake);
	ClassDB::bind_method(D_METHOD("clear", "nav_mesh"), &EditorNavigationMeshGenerator::clear);
}
