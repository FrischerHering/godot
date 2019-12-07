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

void Crowd::reassign_agents() {
	for (Map<Ref<Agent>, int>::Element *E = _agents_to_id.front(); E; E = E->next()) {
		Ref<Agent> agent = E->key();
		dtCrowdAgentParams ap = agent->create_dt_agent_params();

		Vector3 position = agent->get_spawn_position();
		Ref<AgentState> state = agent->get_latest_state();
		if (state.is_valid()) {
			position = state->get_position();
		}

		float pos[] = { position.x, position.y, position.z };
		int agent_id = _crowd->addAgent(pos, &ap);
		_agents_to_id[agent] = agent_id;
	}
}

Crowd::Crowd() {
	_crowd = dtAllocCrowd();
	_is_valid = false;

	current_max_agents = 0;
	auto_grow_multiplier = 0.5f;

	current_max_radius = 1.f;
}

Crowd::~Crowd() {
	dtFreeCrowd(_crowd);
}

void Crowd::clear() {
	_agents_to_id.clear();
}

bool Crowd::create(int max_agents, float max_agent_radius, Ref<DetourNavigation> navigation, bool clear_agents) {
	ERR_FAIL_COND_V(!navigation.is_valid() || !navigation->is_valid(), false);
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

bool Crowd::add_agent(Ref<Agent> p_agent) {
	ERR_FAIL_COND_V(!p_agent.is_valid(), false);
	ERR_FAIL_COND_V(!is_valid(), false);
	ERR_FAIL_COND_V(_agents_to_id.has(p_agent), false);

	extend_if_necessary(p_agent->get_radius());

	Vector3 spawn_position = p_agent->get_spawn_position();
	float pos[] = { spawn_position.x, spawn_position.y, spawn_position.z };
	dtCrowdAgentParams ap = p_agent->create_dt_agent_params();
	int agent_id = _crowd->addAgent(pos, &ap);

	ERR_FAIL_COND_V(agent_id < 0, false);

	_agents_to_id.insert(p_agent, agent_id);

	return true;
}

void Crowd::remove_agent(Ref<Agent> p_agent) {
	ERR_FAIL_COND(!is_valid());

	if (has_agent(p_agent)) {
		int agent_id = _agents_to_id[p_agent];
		_crowd->removeAgent(agent_id);
		_agents_to_id.erase(p_agent);
	}
}

Ref<AgentState> Crowd::get_agent_state(Ref<Agent> p_agent) {
	Ref<AgentState> ret(memnew(AgentState));
	ERR_FAIL_COND_V(!is_valid(), ret);

	if (has_agent(p_agent)) {
		int agent_id = _agents_to_id[p_agent];
		const dtCrowdAgent *ca = _crowd->getAgent(agent_id);
		Vector3 position(ca->npos[0], ca->npos[1], ca->npos[2]);
		Vector3 velocity(ca->vel[0], ca->vel[1], ca->vel[2]);

		Vector3 target = position;
		if (ca->targetState == DT_CROWDAGENT_TARGET_VALID) {
			target = Vector3(ca->targetPos[0], ca->targetPos[1], ca->targetPos[2]);
		}

		ret->set_position(position);
		ret->set_velocity(velocity);
	}

	return ret;
}

bool Crowd::request_move_target(Ref<Agent> p_agent, Vector3 p_target) {
	ERR_FAIL_COND_V(!p_agent.is_valid(), false);
	ERR_FAIL_COND_V(!is_valid(), false);
	ERR_FAIL_COND_V(!_agents_to_id.has(p_agent), false);

	PolySearchResult target = _navigation->find_nearest_poly(p_target);
	ERR_FAIL_COND_V(!target.success, false);

	return _crowd->requestMoveTarget(_agents_to_id[p_agent], target.poly_ref, target.point);
}

bool Crowd::update_navigation(Ref<DetourNavigation> navigation) {
	return create(current_max_agents, current_max_radius, navigation, false);
}

bool Crowd::has_agent(Ref<Agent> p_agent) {
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
		for (Map<Ref<Agent>, int>::Element *E = _agents_to_id.front(); E; E = E->next()) {
			Ref<Agent> agent = E->key();
			agent->fetch_state();
		}
	}
}

// ////////////////////////////////////////////////////////////////////////////

void AgentState::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_position", "position"), &AgentState::set_position);
	ClassDB::bind_method(D_METHOD("get_position"), &AgentState::get_position);
	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &AgentState::set_velocity);
	ClassDB::bind_method(D_METHOD("get_velocity"), &AgentState::get_velocity);
	ClassDB::bind_method(D_METHOD("set_target", "target"), &AgentState::set_target);
	ClassDB::bind_method(D_METHOD("get_target"), &AgentState::get_target);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position"), "set_position", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity"), "set_velocity", "get_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target"), "set_target", "get_target");
}

void AgentState::set_position(Vector3 p_position) {
	position = p_position;
}

Vector3 AgentState::get_position() const {
	return position;
}

void AgentState::set_velocity(Vector3 p_velocity) {
	velocity = p_velocity;
}

Vector3 AgentState::get_velocity() const {
	return velocity;
}

void AgentState::set_target(Vector3 p_target) {
	target = p_target;
}

Vector3 AgentState::get_target() const {
	return target;
}