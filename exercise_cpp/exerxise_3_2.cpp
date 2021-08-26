#include <iostream>
#include "Eigen/Dense"
 
using namespace std;
using namespace Eigen;
 
// int main()
// {
//    Matrix3f A;
//    Vector3f b;
//    A << 1,2,3,  4,5,6,  7,8,10;
//    b << 3, 3, 4;
//    cout << "Here is the matrix A:\n" << A << endl;
//    cout << "Here is the vector b:\n" << b << endl;
//    Vector3f x = A.colPivHouseholderQr().solve(b);
//    cout << "The solution is:\n" << x << endl;
// }

int main()
{
   MatrixXf A = MatrixXf::Random(3, 2);
   cout << "Here is the matrix A:\n" << A << endl;
   VectorXf b = VectorXf::Random(3);
   cout << "Here is the right hand side b:\n" << b << endl;
   cout << "The least-squares solution is:\n"
        << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
}