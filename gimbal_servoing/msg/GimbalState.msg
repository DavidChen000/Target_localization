Header header

#Expected gimbal yaw, pitch angles, unit：degree.
#float32[2] exp_angle
float32 exp_yaw
float32 exp_pitch

#Expected gimbal yaw, pitch angular speed，unit：degree/s.
#float32[2] exp_angular_speed
int16 exp_yaw_speed
int16 exp_pitch_speed

#Current gimbal yaw, pitch,roll  angles, unit：degree.
#float32[3] angle
float32 current_yaw
float32 current_pitch
float32 current_roll

#Current gimbal yaw, pitch,roll  angular speed，unit：degree/s.
#float32[3] angular_speed
float32 current_yaw_speed
float32 current_pitch_speed
float32 current_roll_speed