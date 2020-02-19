// Compute transformation matrix from body frame (index B) into NED frame (index O)
float M_OB_11 = cosf(att.theta)*cosf(att.psi);
float M_OB_12 = sinf(att.phi)*sinf(att.theta)*cosf(att.psi) - cosf(att.phi)*sinf(att.psi); 
float M_OB_13 = cosf(att.phi)*sinf(att.theta)*cosf(att.psi) + sinf(att.phi)*sinf(att.psi);
float M_OB_21 = cosf(att.theta)*sinf(att.psi);
float M_OB_22 = sinf(att.phi)*sinf(att.theta)*sinf(att.psi) + cosf(att.phi)*cosf(att.psi);
float M_OB_23 = cosf(att.phi)*sinf(att.theta)*sinf(att.psi) - sinf(att.phi)*cosf(att.psi);
float M_OB_31 = -sinf(att.theta);
float M_OB_32 = sinf(att.phi)*cosf(att.theta);
float M_OB_33 = cosf(att.phi)*cosf(att.theta);


// Transform lin. acceleration in NED (add gravity to the z-component)
indi.linear_accel_ft.x = M_OB_11*indi.linear_accel_f.x + M_OB_12*indi.linear_accel_f.y + M_OB_13*indi.linear_accel_f.z;
indi.linear_accel_ft.y = M_OB_21*indi.linear_accel_f.x + M_OB_22*indi.linear_accel_f.y + M_OB_23*indi.linear_accel_f.z;
indi.linear_accel_ft.z = M_OB_31*indi.linear_accel_f.x + M_OB_32*indi.linear_accel_f.y + M_OB_33*indi.linear_accel_f.z + 9.81f;

