#include "crowd.h"
#include "agent.h"

#include <iostream>

void Crowd::_bind_methods() {
	ClassDB::bind_method(D_METHOD("create", "max_agents", "max_agent_radius", "navigation_mesh"), &Crowd::create);
	ClassDB::bind_method(D_METHOD("update_navigation", "navigation"), &Crowd::update_navigation);
	ClassDB::bind_method(D_METHOD("has_agent", "agent"), &Crowd::has_agent);
	ClassDB::bind_method(D_METHOD("is_valid"), &Crowd::is_valid);
	ClassDB::bind_method(D_METHOD("update", "delta"), &Crowd::update);
}

void Crowd::_notification(int p_what) {
	Node *parent_node = get_parent();

	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			ERR_FAIL_COND(!update_navigation(parent_node));
			break;
	}
}

void Crowd::reassign_agents() {
	for (Map<Agent *, int>::Element *E = _agents_to_id.front(); E; E = E->next()) {
		Agent *agent = E->key();
		dtCrowdAgentParams ap = agent->create_dt_agent_params();

		Vector3 position = agent->get_transform().origin;

		float pos[] = { position.x, position.y, position.z };
		int agent_id = _crowd->addAgent(pos, &ap);
		_agents_to_id[agent] = agent_id;
	}
}

Crowd::Crowd() {
	_crowd = dtAllocCrowd();
	_is_valid = false;

	_navigation = NULL;

	current_max_agents = 10;
	auto_grow_multiplier = 0.5f;

	current_max_radius = 1.f;
}

Crowd::~Crowd() {
	dtFreeCrowd(_crowd);
}

void Crowd::clear() {
	_agents_to_id.clear();
}

bool Crowd::create(int max_agents, float max_agent_radius, Node *p_navigation_node, bool clear_agents) {
	DetourNavigation *navigation = Object::cast_to<DetourNavigation>(p_navigation_node);
	ERR_FAIL_COND_V(navigation == NULL, false);

	ERR_FAIL_COND_V(!navigation->is_valid(), false);
	ERR_FAIL_COND_V(!_crowd->init(max_agents, max_agent_radius, navigation->get_navigation_mesh()->get_dt_navmesh()), false);

	current_max_agents = max_agents;
	current_max_radius = max_agent_radius;
	_navigation = navigation;

	// Setup local avoidance params to different qualities.
	dtObstacleAvoidanceParams params;
	// Use mostly default settings, copy from dtCrowd.
	memcpy(&params, _crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	_crowd->setObstacleAvoidanceParams(0, &params);

	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	_crowd->setObstacleAvoidanceParams(1, &params);

	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	_crowd->setObstacleAvoidanceParams(2, &params);

	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;
	_crowd->setObstacleAvoidanceParams(3, &params);

	if (clear_agents) {
		clear();
	} else {
		reassign_agents();
	}

	_is_valid = true;
	return true;
}

bool Crowd::add_agent(Node *p_agent_node) {
	Agent *p_agent = Object::cast_to<Agent>(p_agent_node);
	ERR_FAIL_COND_V(p_agent == NULL, false);

	ERR_FAIL_COND_V(!is_valid(), false);
	ERR_FAIL_COND_V(_agents_to_id.has(p_agent), false);

	extend_if_necessary(p_agent->get_radius());

	Vector3 spawn_position = p_agent->get_transform().origin;
	float pos[] = { spawn_position.x, spawn_position.y, spawn_position.z };
	dtCrowdAgentParams ap = p_agent->create_dt_agent_params();
	int agent_id = _crowd->addAgent(pos, &ap);

	ERR_FAIL_COND_V(agent_id < 0, false);

	_agents_to_id.insert(p_agent, agent_id);

	return true;
}

void Crowd::remove_agent(Node *p_agent_node) {
	ERR_FAIL_COND(!is_valid());

	Agent *p_agent = Object::cast_to<Agent>(p_agent_node);
	ERR_FAIL_COND(p_agent == NULL);

	if (has_agent(p_agent)) {
		int agent_id = _agents_to_id[p_agent];
		_crowd->removeAgent(agent_id);
		_agents_to_id.erase(p_agent);
	}
}

bool Crowd::request_move_target(Node *p_agent_node, Vector3 p_target) {
	Agent *p_agent = Object::cast_to<Agent>(p_agent_node);
	ERR_FAIL_COND_V(p_agent == NULL, false);

	ERR_FAIL_COND_V(!is_valid(), false);
	ERR_FAIL_COND_V(!_agents_to_id.has(p_agent), false);

	PolySearchResult target = _navigation->find_nearest_poly(p_target);
	ERR_FAIL_COND_V(!target.success, false);

	return _crowd->requestMoveTarget(_agents_to_id[p_agent], target.poly_ref, target.point);
}

bool Crowd::update_navigation(Node *p_navigation) {
	return create(current_max_agents, current_max_radius, p_navigation, false);
}

bool Crowd::has_agent(Node *p_agent_node) {
	Agent *p_agent = Object::cast_to<Agent>(p_agent_node);
	ERR_FAIL_COND_V(p_agent == NULL, false);

	return _agents_to_id.has(p_agent);
}

bool Crowd::is_valid() const {
	return _is_valid;
}

void Crowd::extend_if_necessary(float next_agent_radius) {
	if (_agents_to_id.size() < current_max_agents && next_agent_radius <= current_max_radius) {
		return;
	}

	std::cout << "Extending... " << current_max_agents << " " << current_max_radius << std::endl;

	int next_size = current_max_agents;
	float next_radius = current_max_radius;

	if (_agents_to_id.size() >= current_max_agents) {
		next_size = ceil((current_max_agents + 1) * (1.f + auto_grow_multiplier));
	}
	if (next_agent_radius > current_max_radius) {
		next_radius = next_agent_radius;
	}

	create(next_size, next_radius, _navigation, false);
}

void Crowd::update(float delta) {
	if (is_valid()) {
		_crowd->update(delta, NULL);
		for (Map<Agent *, int>::Element *E = _agents_to_id.front(); E; E = E->next()) {
			Agent *agent = E->key();

			int agent_id = _agents_to_id[agent];
			const dtCrowdAgent *ca = _crowd->getAgent(agent_id);
			Vector3 position(ca->npos[0], ca->npos[1], ca->npos[2]);
			Vector3 velocity(ca->vel[0], ca->vel[1], ca->vel[2]);

			Transform t = agent->get_transform();
			t.origin = position;
			agent->set_transform(t);
			// TODO auto-rotate
			// TODO pass velocity
		}
	}
}