#include "agent.h"

void Agent::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_to_crowd", "crowd"), &Agent::add_to_crowd);
	ClassDB::bind_method(D_METHOD("remove_from_crowd"), &Agent::remove_from_crowd);

	ClassDB::bind_method(D_METHOD("request_move_target", "target"), &Agent::request_move_target);

	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &Agent::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &Agent::get_radius);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &Agent::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &Agent::get_height);
	ClassDB::bind_method(D_METHOD("set_max_acceleration", "max_acceleration"), &Agent::set_max_acceleration);
	ClassDB::bind_method(D_METHOD("get_max_acceleration"), &Agent::get_max_acceleration);
	ClassDB::bind_method(D_METHOD("set_max_speed", "max_speed"), &Agent::set_max_speed);
	ClassDB::bind_method(D_METHOD("get_max_speed"), &Agent::get_max_speed);
	ClassDB::bind_method(D_METHOD("set_collision_query_factor", "collision_query_factor"), &Agent::set_collision_query_factor);
	ClassDB::bind_method(D_METHOD("get_collision_query_factor"), &Agent::get_collision_query_factor);
	ClassDB::bind_method(D_METHOD("set_path_optimization_factor", "path_optimization_factor"), &Agent::set_path_optimization_factor);
	ClassDB::bind_method(D_METHOD("get_path_optimization_factor"), &Agent::get_path_optimization_factor);
	ClassDB::bind_method(D_METHOD("set_separation_weight", "separation_weight"), &Agent::set_separation_weight);
	ClassDB::bind_method(D_METHOD("get_separation_weight"), &Agent::get_separation_weight);
	ClassDB::bind_method(D_METHOD("set_obstacle_avoidance", "obstacle_avoidance"), &Agent::set_obstacle_avoidance);
	ClassDB::bind_method(D_METHOD("get_obstacle_avoidance"), &Agent::get_obstacle_avoidance);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "radius"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "height"), "set_height", "get_height");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_acceleration"), "set_max_acceleration", "get_max_acceleration");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_speed"), "set_max_speed", "get_max_speed");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "collision_query_factor"), "set_collision_query_factor", "get_collision_query_factor");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "path_optimization_factor"), "set_path_optimization_factor", "get_path_optimization_factor");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "separation_weight"), "set_separation_weight", "get_separation_weight");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "obstacle_avoidance"), "set_obstacle_avoidance", "get_obstacle_avoidance");
}

void Agent::_notification(int p_what) {
	Node *parent_node = get_parent();

	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			ERR_FAIL_COND(!add_to_crowd(parent_node));
			break;
		case NOTIFICATION_EXIT_TREE:
			remove_from_crowd();
			break;
	}
}

Agent::Agent() {
	radius = 1.f;
	height = 2.f;
	max_acceleration = 10.f;
	max_speed = 4.f;

	collision_query_factor = 8.f;
	path_optimization_factor = 30.f;

	separation_weight = 0.f;
	obstacle_avoidance = Crowd::CROWD_OBST_AVOID_GOOD;

	_crowd_ref = NULL;
}

dtCrowdAgentParams Agent::create_dt_agent_params() const {
	dtCrowdAgentParams params;
	memset(&params, 0, sizeof(params));
	params.radius = radius;
	params.height = height;
	params.maxAcceleration = max_acceleration;
	params.maxSpeed = max_speed;
	params.collisionQueryRange = radius * collision_query_factor;
	params.pathOptimizationRange = radius * path_optimization_factor;
	params.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OBSTACLE_AVOIDANCE | DT_CROWD_SEPARATION | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO;
	params.separationWeight = separation_weight;
	params.obstacleAvoidanceType = (unsigned char)obstacle_avoidance;
	return params;
}

bool Agent::add_to_crowd(Node *p_crowd_node) {
	Crowd *p_crowd = Object::cast_to<Crowd>(p_crowd_node);
	ERR_FAIL_COND_V(p_crowd == NULL, false);

	ERR_FAIL_COND_V(!p_crowd->is_valid(), false);

	remove_from_crowd();
	if (p_crowd->add_agent(this)) {
		_crowd_ref = p_crowd;
		return true;
	}

	return false;
}

void Agent::remove_from_crowd() {
	if (_crowd_ref) {
		_crowd_ref->remove_agent(this);
		_crowd_ref = NULL;
	}
}

bool Agent::request_move_target(Vector3 p_target) {
	ERR_FAIL_COND_V(_crowd_ref == NULL, false);
	return _crowd_ref->request_move_target(this, p_target);
}

void Agent::set_radius(float p_radius) {
	radius = p_radius;
}
float Agent::get_radius() const {
	return radius;
}

void Agent::set_height(float p_height) {
	height = p_height;
}
float Agent::get_height() const {
	return height;
}

void Agent::set_max_acceleration(float p_max_acceleration) {
	max_acceleration = p_max_acceleration;
}
float Agent::get_max_acceleration() const {
	return max_acceleration;
}

void Agent::set_max_speed(float p_max_speed) {
	max_speed = p_max_speed;
}
float Agent::get_max_speed() const {
	return max_speed;
}

void Agent::set_collision_query_factor(float p_collision_query_factor) {
	collision_query_factor = p_collision_query_factor;
}
float Agent::get_collision_query_factor() const {
	return collision_query_factor;
}

void Agent::set_path_optimization_factor(float p_path_optimization_factor) {
	path_optimization_factor = p_path_optimization_factor;
}
float Agent::get_path_optimization_factor() const {
	return path_optimization_factor;
}

void Agent::set_separation_weight(float p_separation_weight) {
	separation_weight = p_separation_weight;
}
float Agent::get_separation_weight() const {
	return separation_weight;
}

void Agent::set_obstacle_avoidance(Crowd::ObstacleAvoidancePreset p_obstacle_avoidance) {
	obstacle_avoidance = p_obstacle_avoidance;
}
Crowd::ObstacleAvoidancePreset Agent::get_obstacle_avoidance() const {
	return obstacle_avoidance;
}