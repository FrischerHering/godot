#ifndef CROWD_H
#define CROWD_H

#include "core/map.h"
#include "core/object.h"
#include "core/string_name.h"
#include "detour_navigation.h"
#include "scene/main/node.h"

#include <DetourCrowd.h>

class Agent;
class AgentState;

class Crowd : public Reference {
	GDCLASS(Crowd, Reference);

	dtCrowd *_crowd;
	bool _is_valid;

	Ref<DetourNavigation> _navigation;
	Map<Ref<Agent>, int> _agents_to_id;

protected:
	static void _bind_methods();

	int current_max_agents;
	float auto_grow_multiplier;

	float current_max_radius;

	void reassign_agents();

public:
	enum ObstacleAvoidancePreset {
		CROWD_OBST_AVOID_LOW = 0,
		CROWD_OBST_AVOID_MEDIUM = 1,
		CROWD_OBST_AVOID_GOOD = 2,
		CROWD_OBST_AVOID_HIGH = 3
	};

public:
	Crowd();
	~Crowd();

	void clear();

	bool create(int max_agents, float max_agent_radius, Ref<DetourNavigation> navigation, bool clear_agents = true);
	bool add_agent(Ref<Agent> p_agent);
	void remove_agent(Ref<Agent> p_agent);
	Ref<AgentState> get_agent_state(Ref<Agent> p_agent);
	bool request_move_target(Ref<Agent> p_agent, Vector3 p_target);

	bool update_navigation(Ref<DetourNavigation> navigation);

	bool has_agent(Ref<Agent> p_agent);

	bool is_valid() const;
	void extend_if_necessary(float next_agent_radius);

	void update(float delta);
};
VARIANT_ENUM_CAST(Crowd::ObstacleAvoidancePreset);

/// Data struct that represents current state of an agent within crowd.
struct AgentState : Reference {
	GDCLASS(AgentState, Reference);

	Vector3 position;
	Vector3 velocity;

	Vector3 target;

public:
	void set_position(Vector3 p_position);
	Vector3 get_position() const;

	void set_velocity(Vector3 p_velocity);
	Vector3 get_velocity() const;

	void set_target(Vector3 p_target);
	Vector3 get_target() const;

protected:
	static void _bind_methods();
};

#endif // CROWD_H