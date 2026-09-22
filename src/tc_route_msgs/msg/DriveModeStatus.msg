builtin_interfaces/Time stamp

uint8 MODE_AUTONOMOUS=1
uint8 MODE_MANUAL=2
uint8 mode

uint8 SOURCE_ZERO=0
uint8 SOURCE_AUTONOMOUS_CMD=1
uint8 SOURCE_MANUAL_CMD=2
uint8 output_source

bool joy_available
bool l1_pressed
bool ps_button_pressed
float32 ps_hold_progress_s

bool manual_input_active
bool manual_cmd_alive
bool autonomous_cmd_alive

bool auto_resume_pending
float32 auto_resume_remaining_s

float32 pending_autonomous_linear_x
float32 pending_autonomous_angular_z

float32 output_linear_x
float32 output_angular_z

string reason
