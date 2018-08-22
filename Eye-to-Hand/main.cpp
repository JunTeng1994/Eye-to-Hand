#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <vector>
//#include <math.h>
#include <iostream>
#include <fstream>
//#include <string>

using namespace std;
using namespace Eigen;
using namespace cv;

bool ReadMatrix(string FileName, Eigen::Matrix4d& transMat)
{
    cv::FileStorage fs(FileName, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }
    cv::Mat cvtransMat;
    fs["TransMat"] >> cvtransMat;
    cv2eigen(cvtransMat, transMat);
    fs.release();
    return true;
}

bool SaveMatrix(string FileName, Eigen::Matrix4d transMat)
{
    cv::FileStorage fs(FileName, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        return false;
    }
    cv::Mat cvtransMat;
    eigen2cv(transMat, cvtransMat);
    fs << "TransMat" << cvtransMat;
    fs.release();
    return true;
}

bool getTrnasMatrix(string FileName, int N, vector<Eigen::Matrix4d>& trans)
{
    Eigen::Matrix4d matemp, matemp1, matemp2;

    if (!ReadMatrix("./data/"+FileName+"0.yaml", matemp1))
    {
        cout << "Fail to read matrix" << endl;
        return false;
    }

    for (int i = 1; i < N; i++)
    {
        if (!ReadMatrix("./data/"+FileName + to_string(i) + ".yaml", matemp2))
        {
            cout << "Fail to read matrix" << endl;
            return false;
        }
        matemp = matemp2*matemp1.inverse();
        trans.push_back(matemp);
        matemp1 = matemp2;
    }
    return true;
}

void sqrtm(Matrix3d& R, Matrix3cd& sqrtR)
{
    sqrtR = Matrix3cd::Zero();
    ComplexSchur<Matrix3d> schur(3);
    schur.compute(R);
    Matrix3cd Q = schur.matrixU();
    Matrix3cd T = schur.matrixT();
    Vector3cd diagT = T.diagonal();
    bool flag = true;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i != j && abs(T(i, j)) > 0.0000001)
            {
                flag = false;
                break;
            }
        }
    }
    if (flag)
    {
        Matrix3cd sqT;
        for (int i = 0; i < 9; i++)
        {
            sqT(i / 3, i % 3) = sqrt(diagT(i % 3));
        }
        sqrtR = (Q.array()*sqT.array()).matrix()*Q.transpose();
        sqrtR = (sqrtR + sqrtR.transpose())*0.5;
    }
    else
    {
        Matrix3cd U = Matrix3cd::Zero();
        for (int j = 0; j < 3; j++)
        {
            U(j, j) = sqrt(T(j, j));
            for (int i = j - 1; i >= 0; i--)
            {
                complex<double> s;
                s = (0.0, 0.0);
                for (int k = i + 1; k <= j - 1; k++)
                {
                    s += U(i, k)*U(k, j);
                }
                U(i, j) = (T(i, j) - s) / (U(i, i) + U(j, j));
            }
        }
        sqrtR = Q*U*Q.transpose();
    }
}

bool Navy_HandEye(Matrix4d& Hcg, vector<Matrix4d> Hgij, vector<Matrix4d> Hcij)
{
    if (Hgij.size() != Hcij.size())
    {
        cout << "Input Wrong Matries, Please Check Their Sizes  ..." << endl;
        return false;
    }

    Mat Temp_R;
    Matrix3d temp_Rotation;
    Mat rvec;
    Vector3d rVecHg, rVecHc;

    int n = Hgij.size();
    Matrix3d M = Matrix3d::Zero();

    for (int i = 0; i < n; i++)
    {
        temp_Rotation = (Hgij[i]).block(0, 0, 3, 3);
        eigen2cv(temp_Rotation, Temp_R);
        Rodrigues(Temp_R, rvec);
        cv2eigen(rvec, rVecHg);

        temp_Rotation = (Hcij[i]).block(0, 0, 3, 3);
        eigen2cv(temp_Rotation, Temp_R);
        Rodrigues(Temp_R, rvec);
        cv2eigen(rvec, rVecHc);

        M =  rVecHc*rVecHg.transpose() + M;
    }

    Matrix3d MtM = M.transpose()*M;
    Matrix3cd sqrtMM;
    sqrtm(MtM, sqrtMM);
    Hcg.block(0, 0, 3, 3) = (sqrtMM.real()).inverse()*M.transpose();

    MatrixXd C = MatrixXd::Zero(3 * n, 3);
    VectorXd d = VectorXd::Zero(3 * n);
    Vector3d temp_d;
    Matrix3d I3 = Matrix3d::Identity();
    for (int i = 0; i < n; i++)
    {
        temp_d = Hcg.block(0, 0, 3, 3)*Hcij[i].block(0, 3, 3, 1);
        Vector3d temp_t = Hgij[i].block(0, 3, 3, 1);
        d(3 * i) = temp_t(0) - temp_d(0);
        d(3 * i + 1) = temp_t(1) - temp_d(1);
        d(3 * i + 2) = temp_t(2) - temp_d(2);
        for (int j = 0; j < 3; j++)
        {
            Matrix3d temp_R = Hgij[i].block(0, 0, 3, 3);
            C(3 * i, j) = I3(0, j) - temp_R(0, j);
            C(3 * i + 1, j) = I3(1, j) - temp_R(1, j);
            C(3 * i + 2, j) = I3(2, j) - temp_R(2, j);
        }
    }
    Hcg.block(0, 3, 3, 1) = (C.transpose()*C).inverse()*C.transpose()*d;

    return true;
}

int main()
{
    vector<Eigen::Matrix4d> Hgij,Hcij;
    if(!getTrnasMatrix("A",11,Hgij)){
        cout<< "Error Occur ..." << endl;
        return -1;
    }
    if(!getTrnasMatrix("B",11,Hcij)){
        cout<< "Error Occur ..." << endl;
        return -1;
    }

    /*
    Eigen::Matrix4d R;

	R << -0.989992, -0.141120, 0.000000, 0,
		0.141120, -0.989992, 0.000000,0,
		0.000000, 0.000000, 1.000000,0,
		0, 0, 0, 1 ;
	Hgij.push_back(R);

	R << -0.989992, -0.138307, 0.028036, -26.9559,
		0.138307, -0.911449, 0.387470, -96.1332,
		-0.028036, 0.387470, 0.921456, 19.4872,
		0, 0, 0, 1;
	Hcij.push_back(R);

	R << 0.070737, 0.000000, 0.997495, -400.000,
		0.000000, 1.000000, 0.000000, 0.000000,
		-0.997495, 0.000000, 0.070737, 400.000,
		0, 0, 0, 1;
	Hgij.push_back(R);

	R << 0.070737, 0.198172, 0.997612, -309.543,
		-0.198172, 0.963323, -0.180936, 59.0244,
		-0.977612, -0.180936, 0.107415, 291.177,
		0, 0, 0, 1;
	Hcij.push_back(R);

	R << 1.000000, 0, 0, 30,
		0, 0.866025, -0.50000, 20,
		0, 0.500000, 0.866025, 50,
		0, 0, 0, 1;
	Hgij.push_back(R);

	R << 1.000000, 0, 0, 30,
		0, 0.866025, -0.50000, -23.728682,
		0, 0.500000, 0.866025, 67.665470,
		0, 0, 0, 1;
	Hcij.push_back(R);
     */

    Eigen::Matrix4d Hcg = Eigen::Matrix4d::Identity();
    Navy_HandEye(Hcg,Hgij,Hcij);

    cout << "The transform matrix is " << Hcg << endl;

    if(!SaveMatrix("EyeHand.yaml", Hcg)){
        cout << "Fail to save transform matrix ..." << endl;
        return -1;
    }

    return 0;
}