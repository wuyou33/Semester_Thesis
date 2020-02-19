
#include <stdio.h>
#include <math.h>


struct Angles {
  float phi; 
  float theta;
  float psi;
};

void m_ob(struct Angles att, float matrix[3][3]) {

	// Compute transformation matrix from body frame (index B) into NED frame (index O)
	matrix[0][0] = cosf(att.theta)*cosf(att.psi);
	matrix[0][1] = sinf(att.phi)*sinf(att.theta)*cosf(att.psi) - cosf(att.phi)*sinf(att.psi); 
	matrix[0][2] = cosf(att.phi)*sinf(att.theta)*cosf(att.psi) + sinf(att.phi)*sinf(att.psi);
	matrix[1][0] = cosf(att.theta)*sinf(att.psi);
	matrix[1][1] = sinf(att.phi)*sinf(att.theta)*sinf(att.psi) + cosf(att.phi)*cosf(att.psi);
	matrix[1][2] = cosf(att.phi)*sinf(att.theta)*sinf(att.psi) - sinf(att.phi)*cosf(att.psi);
	matrix[2][0] = -sinf(att.theta);
	matrix[2][1] = sinf(att.phi)*cosf(att.theta);
	matrix[2][2] = cosf(att.phi)*cosf(att.theta);
}

int main () {

	struct Angles att = {
		.phi = 0.9,
		.theta = 0.8,
		.psi = 0.2
	};

	

	float matrix[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};


	m_ob(att, matrix);

	int r, c; 
	for(r=0; r<3; r++) {
		for(c=0; c<3; c++) {
			printf("M_OB%d%d: %.5f\n", r+1, c+1, matrix[r][c]);
		}
	}
	

	return 0;
}
