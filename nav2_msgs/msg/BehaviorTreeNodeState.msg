# Current state of a single BT node (used in snapshots)
uint16 node_uid
string node_name
string node_type       # Action, Condition, Control, Decorator, SubTree
uint8 status           # Uses BehaviorTreeTransition constants
