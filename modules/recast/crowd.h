#ifndef CROWD_H
#define CROWD_H

#include "core/map.h"
#include "core/object.h"
#include "core/string_name.h"
#include "detour_navigation.h"
#include "scene/3d/spatial.h"
#include "scene/main/node.h"

#include <DetourCrowd.h>

class Agent;
class AgentState;

class Crowd : public Spatial {
	GDCLASS(Crowd, Spatial);

	dtCrowd *_crowd;
	bool _is_valid;

	DetourNavigation *_navigation;
	Map<Agent *, int> _agents_to_id;

protected:
	static void _bind_methods();
	void _notification(int p_what);

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

	bool create(int max_agents, float max_agent_radius, Node *p_navigation, bool clear_agents = true);
	bool add_agent(Node *p_agent);
	void remove_agent(Node *p_agent);

	bool request_move_target(Node *p_agent, Vector3 p_target);

	bool update_navigation(Node *p_navigation);

	bool has_agent(Node *p_agent);

	bool is_valid() const;
	void extend_if_necessary(float next_agent_radius);

	void update(float delta);
};
VARIANT_ENUM_CAST(Crowd::ObstacleAvoidancePreset);

#endif // CROWD_H