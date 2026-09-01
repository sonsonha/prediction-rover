std_msgs/Header header

string source_trajectory_id

uint8 decision
uint8 GO=0
uint8 STOP=1

uint8 reason
uint8 CURRENT_CLEAR=0
uint8 NO_CURRENT_PREDICTION=1
uint8 PREDICTION_STALE=2
uint8 COLLISION_CANDIDATE=3
uint8 ROLLOVER_EVIDENCE_INVALID=4
uint8 ROLLOVER_POLICY_TRIGGERED=5

bool prototype_policy
