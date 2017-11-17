#include "utilities.h"


utilities::utilities(){
}


Matrix3d utilities::rotx(double alpha){
	Matrix3d Rx;
	Rx << 	1,0,0,
		0,cos(alpha),-sin(alpha),
		0,sin(alpha), cos(alpha);
	return Rx;
}

Matrix3d utilities::roty(double beta){
	Matrix3d Ry;
	Ry << 	cos(beta),0,sin(beta),
		0,1,0,
		-sin(beta),0, cos(beta);
	return Ry;
}

Matrix3d utilities::rotz(double gamma){
	Matrix3d Rz;
	Rz << 	cos(gamma),-sin(gamma),0,
		sin(gamma),cos(gamma),0,
		0,0, 1;
	return Rz;
}



Matrix3d utilities::L_matrix(Matrix3d R_d, Matrix3d R_e)
{
	Matrix3d L = -0.5 * (skew(R_d.col(0))*skew(R_e.col(0)) + skew(R_d.col(1))*skew(R_e.col(1)) + skew(R_d.col(2))*skew(R_e.col(2)));
	return L;
}



Matrix3d utilities::skew(Vector3d v)
{
	Matrix3d S;
	S << 0,	-v[2],	 v[1],		//Skew-symmetric matrix
		v[2],	    0,	-v[0],
		-v[1],	 v[0], 	   0;
	return S;
}

Vector3d utilities::rotationMatrixError(Matrix4d Td, Matrix4d Te)
{
	
	Matrix3d R_e = Te.block(0,0,3,3);		//Matrix.slice<RowStart, ColStart, NumRows, NumCols>();	
	Matrix3d R_d = Td.block(0,0,3,3);
	
	Vector3d eo = 0.5 * (skew(R_e.transpose().col(0))*R_d.transpose().col(0) + skew(R_e.transpose().col(1))*R_d.transpose().col(1) + skew(R_e.transpose().col(2))*R_d.transpose().col(2)) ;
	return eo;
}

	
