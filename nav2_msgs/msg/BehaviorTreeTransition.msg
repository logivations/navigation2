# Individual BT node status transition — efficient format for database/LLM pipelines
builtin_interfaces/Time timestamp
uint16 node_uid

# Status constants (shared with BehaviorTreeSnapshot)
uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCESS=2
uint8 FAILURE=3
uint8 SKIPPED=4

uint8 previous_status
uint8 current_status
