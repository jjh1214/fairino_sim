# Result of one recipe step (move, grip, release, wait or io), checked before it runs.
uint8 OK=0
uint8 WARNING=1       # done, but closer to the environment than the warning distance
uint8 FAILED=2        # not done: collision, unreachable target, joint limit, nothing to grip, ...

string name           # step name given by the recipe
string kind           # move, grip, release, wait or io
uint8 status
string message        # human readable explanation
bool check_only       # checked only (check session); the robot did not move

float64 start_time    # cell time at the start of the step [s] (simulation, or virtual in a check)
float64 duration      # motion or waiting time [s]; 0 for grip, release and io
float64 min_clearance # smallest distance from the robot, tool and held object to the equipment along the path [m]; -1 if not computed
string closest_link   # robot link (or held object) at min_clearance
string closest_object # equipment object at min_clearance
string[] collisions   # "link <-> object" pairs at the first collision, empty if none
float64 collision_at  # fraction of the path (0..1), or of the waiting time, at the first collision; -1 if none
string[] allowed_contacts  # contacts that were allowed for this step, as "body <-> object"

float64[] start_joints # rad
float64[] end_joints   # rad
string held_object     # object held by the gripper after this step, empty if none
