# Periodic full snapshot of BT state — includes uid-to-name mapping + current status
# Published every few seconds so late-joining rosbag consumers can reconstruct state
builtin_interfaces/Time timestamp
string tree_id                        # BT ID or filename
BehaviorTreeNodeState[] nodes         # All nodes with name, type, and current status
