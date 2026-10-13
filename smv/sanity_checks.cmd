go
print_reachable_states
check_invar -p "!(system.x_d_stage_1 = 16 & system.y_d_stage_1 = 13)"
check_ctlspec -p "AG ((system.x_d_stage_1 = 16 & system.y_d_stage_1 = 13) -> AF (system.x_d_stage_1 = 9 & system.y_d_stage_1 = 10))"
check_ctlspec -p "AG ((system.x_d_stage_1 = 16 & system.y_d_stage_1 = 13) -> AF AG (system.x_d_stage_1 = 5 & system.y_d_stage_1 = 5))"
quit
