#ifndef AGENT_H
#define AGENT_H

#include "core/vector.h"
#include "crowd.h"
#include "scene/3d/spatial.h"
#include "scene/main/node.h"

#include <DetourCrowd.h>

class Agent : public Spatial {
	GDCLASS(Agent, Spatial);

protected:
	static void _bind_methods();
	void _notification(int p_what);

	float radius;
	float height;
	float max_acceleration;
	float max_speed;

	float collision_query_factor;
	float path_optimization_factor;

	float separation_weight;
	Crowd::ObstacleAvoidancePreset obstacle_avoidance;

	bool is_valid_crowd_ref() const;

private:
	Crowd *_crowd_ref;

public:
	Agent();

	bool add_to_crowd(Node *p_crowd);
	void remove_from_crowd();

	bool request_move_target(Vector3 p_target);

	dtCrowdAgentParams create_dt_agent_params() const;

	void set_radius(float p_radius);
	float get_radius() const;

	void set_height(float p_height);
	float get_height() const;

	void set_max_acceleration(float p_max_acceleration);
	float get_max_acceleration() const;

	void set_max_speed(float p_max_speed);
	float get_max_speed() const;

	void set_collision_query_factor(float p_collision_query_factor);
	float get_collision_query_factor() const;

	void set_path_optimization_factor(float p_path_optimization_factor);
	float get_path_optimization_factor() const;

	void set_separation_weight(float p_separation_weight);
	float get_separation_weight() const;

	void set_obstacle_avoidance(Crowd::ObstacleAvoidancePreset p_obstacle_avoidance);
	Crowd::ObstacleAvoidancePreset get_obstacle_avoidance() const;
};

#endif // AGENT_H