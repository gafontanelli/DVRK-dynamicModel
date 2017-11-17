#include<cmath>


#include <Eigen/Dense>
using namespace std;  //calling the standard directory
using namespace Eigen;


class utilities{  

public:
	utilities();
	

	Matrix3d rotx(double alpha);

	Matrix3d roty(double beta);

	Matrix3d rotz(double gamma);

	Matrix3d L_matrix(Matrix3d R_d, Matrix3d R_e);
	Matrix3d skew(Vector3d v);
	Vector3d rotationMatrixError(Matrix4d Td, Matrix4d Te);


private:	

};	
