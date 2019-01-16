#include <Eigen/Dense>
#include <math.h>

#define PI 3.14159265359
#define DEG_TO_RAD 57.2957795131
using namespace Eigen;

/*******************************************************************************
 * 
 *	* * * * * * * * * *TFs&PARAMs * * * * * * * * * *
 *
 *	| TRANSFORMATION |	 THETA	 |	 ALPHA	 |	 D	 |	 R	 |
 *	| -------------  | --------- | --------- | ----- | ----- |
 *	|		Tw1		 |	  N/A	 |	  N/A	 |	N/A	 |	N/A	 |
 *	|		T12		 |	 theta2	 |	 alpha2	 |  d2   |  r2   |
 *	|		T23		 |	 theta3	 |	 alpha3	 |  d3   |  r3   |
 *	|		T34		 |	 theta4	 |	 alpha4	 |  d4   |  r4   |
 *	|		T4L		 |	  N/A	 |	  N/A	 |	N/A	 |	N/A	 |
 *	|		T15		 |	 theta2	 |	 alpha5	 |  d5   |  r5   |
 *	|		T56		 |	 theta6	 |	 alpha6	 |  d6   |  r6   |
 *	|		T67		 |	 theta7	 |	 alpha7	 |  d7   |  r7   |
 *	|		T7R		 |	  N/A	 |	  N/A	 |	N/A	 |	N/A	 |
 *
 *			T3L = T34 * T4L & T6R = T67 * T7R
 *			can be seen as single matrix
 *
 *
 *	* * * * * * * * * *PARAMETERS VALUEs * * * * * * * * * *
 *	 * * * gazebo simulation model kinematic
 *                                                           theta_n are notationed as "q_n" in inverse kinematic computation
 *	|  +theta2   |	  var	 | |	alpha2	 |	   0	 |
 *	|     d2     |	   0	 | |     -r2     |   -var	 |
 *
 *	|-pi/2-theta3|	  var	 | |	alpha3	 |	 -pi/2	 |
 *	|    +d3     |	 +var    | |      r3     |     0	 |
 *
 *	|  +theta4	 |	  var	 | |	alpha4	 |	   0	 |
 *	|    +d4     |	 +var	 | |      r4     |     0	 |
 *
 *	|  +theta5=2 |	  var	 | |	alpha5	 |	   0	 |
 *	|     d5     |	  0.7	 | |     +r5     |   +var	 |
 *
 *	|	theta6=3 |	  var	 | |	alpha6	 |	 -pi/2	 |
 *	|    +d6     |	 +var	 | |      r6     |     0	 |
 *
 *	|  +theta7=4 |	  var	 | |	alpha7	 |	   0	 |
 *	|    -d7     |	 -var	 | |      r7     |     0	 |

 *	!! Denavit-Hartenberg (DH) Transformations !!
 *
 *	T(n-1)<-n = 
 *	+-								 -+	   +-	    -+
 *	| ctn  -stn*can  stn*san | rn*ctn |	   |	 |   |
 *	| stn   ctn*can -ctn*san | rn*stn |	   |  R  | t |
 *	|  0	  san  	   can   |   dn   |  = |     |   |
 *	| -----------------------+--------|    |-----+---|
 *	|  0	   0		0	 |   1	  |    |0 0 0| 1 |
 *	+-								 -+    +-		-+
 * 	ctn = cos(THETAn) can =cos(ALPHAn) stn = sin(THETAn) san = sin(ALPHAn)
 *怎么样
 *	T(n-1)->n = 
 *	+-								    -+	   +-	    -+
 *	|   ctn       stn       0  |   -rn   |	   |	 |   |
 *	| -stn*can  ctn*can    san | -dn*san |	   |  R  | t |
 *	|    0	   -ctn*san    can | -dn*can |  =  |     |   |
 *	| -----------------------+-----------|     |-----+---|
 *	|    0	       0		0  |    1    |     |0 0 0| 1 |
 *	+-								    -+     +-		-+
 * 	ctn = cos(THETAn) can =cos(ALPHAn) stn = sin(THETAn) san = sin(ALPHAn)
 *
 *
 *	!!Translation&Yaw->Pitch->Roll Transformations !!
 *				  phi->theta->psi
 *
 *	T(n-1)<-n = 
 *	+-															-+
 *	|ct*cpsi  sphi*st*cpsi-cphi*spsi  cphi*st*cpsi+sphi*spsi | x |
 *	|ct*spsi  sphi*st*spsi+cphi*cpsi  cphi*st*spsi-sphi*cpsi | y |
 *	|  -st            sphi*ct                cphi*ct         | z |
 *	| -------------------------------------------------------+---|
 *	|    0                0                      0           | 1 |
 *	+-															-+
 *	c -> cos() s -> sin()

*************************************************************************************/
/***
 *  Any rotation's positive direction is counter-clockwise
***/


Matrix4f DH(float alpha, float theta, float d, float r){
    float ca = cosf( alpha ); float sa = sinf( alpha );
    float ct = cosf( theta ); float st = sinf( theta );

    Matrix4f T;
    T(0, 0) = ct; T(0, 1) = -st * ca; T(0, 2) = st * ca; T(0, 3) = r * ct;
    T(1, 0) = st; T(1, 1) = ct * ca; T(1, 2) = -ct * sa; T(1, 3) = r * st;
    T(2, 0) = 0.; T(2, 1) = sa; T(2, 2) = ca; T(2, 3) = d;
    T(3, 0) = 0.; T(3, 1) = 0.; T(3, 2) = 0.; T(3, 3) = 1.;

    return T;
}

Matrix4f inv_DH(float alpha, float theta, float d, float r){
    float ca = cosf( alpha ); float sa = sinf( alpha );
    float ct = cosf( theta ); float st = sinf( theta );

    Matrix4f IT;
    IT(0, 0) = ct; IT(0, 1) = st; IT(0, 2) = 0.; IT(0, 3) = -r;
    IT(1, 0) = -ca * st; IT(1, 1) = ct * ca; IT(1, 2) = sa; IT(1, 3) = -d * sa;
    IT(2, 0) = sa * st; IT(2, 1) = -sa * ct; IT(2, 2) = ca; IT(2, 3) = -d * ca;
    IT(3, 0) = 0.; IT(3, 1) = 0.; IT(3, 2) = 0.; IT(3, 3) = 1.;

    return IT;
}

Matrix4f RT(float tx, float ty, float tz, float phi, float theta, float psi){
    float ct = cosf( theta ); float st = sinf( theta );
    float cphi = cosf( phi ); float sphi = sinf( phi );
    float cpsi = cosf( psi ); float spsi = sinf( psi );

    Matrix4f rt;
    rt(0, 0) = ct * cpsi;                      rt(0, 1) = sphi * st * cpsi - cphi * spsi; 
    rt(0, 2) = cphi * st * cpsi+sphi * spsi;   rt(0, 3) = tx;
    rt(1, 0) = ct * spsi;                      rt(1, 1) = sphi * st * spsi + cphi * cpsi;
    rt(1, 2) = cphi * st * spsi - sphi * cpsi; rt(1, 3) = ty;
    rt(2, 0) =  - st;                          rt(2, 1) = sphi * ct; 
    rt(2, 2) = cphi * ct;                      rt(2, 3) = tz;
    rt(3, 0) = 0.;                             rt(3, 1) = 0.; 
    rt(3, 2) = 0.;                             rt(3, 3) = 1.;
    return rt;
}


float *Inv_Kinematic(float *gzPoint) {
    /***
    q2 = theta2;
         -PI  <= theta2 <= PI;
    q3 = PI/2 + theta3;
         -PI  <= theta3 <= -PI/2;
        -PI/2 <=   q3   <= 0;
    q4 = theta4;
        -PI/2 <= theta4 <= PI/2;
    ***/
    float *q = new float[3];

    // q2
    if (gzPoint[1] == 0) {
        if (gzPoint[0] > 0) q[0] = -PI / 2;
        else if ( gzPoint[0] < 0) q[0] = PI / 2;
        else q[0] = 0;
    }
    else if (gzPoint[1] > 0) {
        q[0] = -atanf(gzPoint[0] / gzPoint[1]);
    }
    else {
        if (gzPoint[0] > 0) q[0] = atanf(gzPoint[1] / gzPoint[0]) - PI / 2;
        else if ( gzPoint[0] < 0) q[0] = atanf(gzPoint[1] / gzPoint[0]) + PI / 2;
        else q[0] = PI;
    }
    // q3
    float s2 = sinf(q[0]); float c2 = cosf(q[0]);
    float a = -gzPoint[0] * c2 - gzPoint[1] * s2 - 136.91;
    float b = -gzPoint[0] * s2 + gzPoint[1] * c2;
    if (a == 0) exit (EXIT_FAILURE);
    //printf("a = %f, b = %f\n", a, b);
    float theta3 = asinf(44.35/ sqrtf(pow(a, 2) + pow(b, 2)) ) - atanf(b / a) - PI;
    q[1] = PI / 2 + theta3;

    // q4
    float c = -cosf(q[1] + theta3) * gzPoint[0] - sinf(q[1] + theta3) * gzPoint[1] - 136.91 * cosf(theta3);
    float d = 53.5 - gzPoint[2];
    float beta = atanf(d / c);
    q[2] = -beta;
    return q;
}

void AnglesToQaternion(float *q, float roll, float pitch, float yaw) { // Convert Euler angles in degrees to quaternions

    roll  = roll  * DEG_TO_RAD;
    pitch = pitch * DEG_TO_RAD;
    yaw   = yaw   * DEG_TO_RAD;

    float t0 = cosf(yaw * 0.5);
    float t1 = sinf(yaw * 0.5);
    float t2 = cosf(roll * 0.5);
    float t3 = sinf(roll * 0.5);
    float t4 = cosf(pitch * 0.5);
    float t5 = sinf(pitch * 0.5);

    q[0] = t0 * t2 * t4 + t1 * t3 * t5;
    q[1] = t0 * t3 * t4 - t1 * t2 * t5;
    q[2] = t0 * t2 * t5 + t1 * t3 * t4;
    q[3] = t1 * t2 * t4 - t0 * t3 * t5;

    printf(">> q-1: %f %f %f %f \n", q[0], q[1], q[2], q[3]);
}

float *AnglesToQaternion(float roll, float pitch, float yaw) { // Convert Euler angles in degrees to quaternions

    float *q = new float[4];
    // Or C-style: float *q = malloc(sizeof(float)*4);

    roll  = roll  * DEG_TO_RAD;
    pitch = pitch * DEG_TO_RAD;
    yaw   = yaw   * DEG_TO_RAD;

    float t0 = cosf(yaw * 0.5);
    float t1 = sinf(yaw * 0.5);
    float t2 = cosf(roll * 0.5);
    float t3 = sinf(roll * 0.5);
    float t4 = cosf(pitch * 0.5);
    float t5 = sinf(pitch * 0.5);

    q[0] = t0 * t2 * t4 + t1 * t3 * t5;
    q[1] = t0 * t3 * t4 - t1 * t2 * t5;
    q[2] = t0 * t2 * t5 + t1 * t3 * t4;
    q[3] = t1 * t2 * t4 - t0 * t3 * t5;

    printf(">> q-1: %f %f %f %f \n", q[0], q[1], q[2], q[3]);

    return q;
}


// int main() {
//     float q[4];
//     AnglesToQaternion(&q[0], 0, 30.0, 0); 
//     printf(">> q-2: %f %f %f %f \n", q[0], q[1], q[2], q[3]);
// }