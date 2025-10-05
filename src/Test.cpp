// ----------------------------------------------------------------
// TESTING
// ----------------------------------------------------------------

#include "Test.h"
#include "Vector2.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Transform.h"
#include "AffineTransform.h"
#include "meshprocess\Mesh.h"
#include "meshprocess\ObjHandler.h"
#include "utility\RandomUtils.h"
#include <iostream>


void adg::assignmentTests()
{
	testQuaternionAssignment();
	testRotationMatrixAssignment();
	testRotateMesh();
	std::cout << "\n\n";
}

void adg::testQuaternionAssignment()
{
	std::cout << "---TEST QUATERNION-------------------------------------\n\n";

	Quaternion q1{};
	q1 = axisAngleToQuaternion(Vector3::up(), Scalar(180));
	std::cout << "Build quaternion from Axis angle: up vector and 180 deg.\nq1: " << cleanQuaternion(q1) << "\n\n";

	Vector3 v(Scalar(1), Scalar(0), Scalar(0));
	std::cout << "Apply quaternion to: \nv: " << v << "\nv rotated: " << cleanVector3(applyRotationOptimized(v, q1));

	Quaternion q2 = inverseQuaternion(q1);
	std::cout << "\n\nCheck inverse. \nq2: " << cleanQuaternion(q2) 
		      << "\nv rotated: " << cleanVector3(applyRotationOptimized(v, q2)) << "\n";

	std::cout << "\n----------------------------------------\n\n";

	Quaternion q3 = axisAngleToQuaternion(Vector3(Scalar(1), Scalar(0.7), Scalar(1.5)).normalized(), Scalar(60));
	Vector3 v2(Scalar(3), Scalar(2), Scalar(5));
	for (int i = 0; i < 6; ++i) {
		v2 = applyRotationOptimized(v2, q3);
	}
	std::cout << "Construct a rot around an arbitrary oblique axis, by 60 deg.\nq: " 
		      << cleanQuaternion(q3) 
		      << "\nv: " << Vector3(Scalar(3), Scalar(2), Scalar(5)) 
		      << "\n\nApply rotation 6 times:\nv rotated: "
		      << cleanVector3(v2) << "\n\n";
}

void adg::testRotationMatrixAssignment()
{
	std::cout << "\n---TEST ROTATION MATRIX---------------------------------\n"
		<< "\nConvert an arbitrary quaternion to a matrix 3x3\n\n";

	Quaternion q = axisAngleToQuaternion(Vector3(Scalar(0.6), Scalar(0.7), Scalar(1.5)).normalized(), 70);
	Vector3 v(Scalar(1), Scalar(0), Scalar(0));
	Matrix3x3 mat = quaternionToMatrix3x3(q);
	std::cout << "q: " << cleanQuaternion(q)
		<< "\nv: " << v << "\nApply rotation through quaternion:\nv rotated: "
		<< cleanVector3(applyRotationOptimized(v, q))
		<< "\nApply rotation through matrix:\nv rotated: "
		<< cleanVector3(applyMatrix3x3(mat, v)) << "\n\n";
}

void adg::testRotateMesh()
{
	std::cout << "\n---TEST ROTATE MESH-------------------------\n";

	Mesh myMesh;
	if (ObjHandler::load("assets/models/suzannen.obj", myMesh))
		std::cout << "\nFile loaded from: 'assets/models/suzannen.obj'\nmesh name: " << myMesh.m_data.name
		<< "\nmtl library: " << myMesh.m_data.mtl
		<< "\nvertexCount: " << myMesh.m_data.vertex_Count
		<< "\nnormalCount: " << myMesh.m_data.normals_Count
		<< "\nuvsCount: " << myMesh.m_data.uvs_Count
		<< "\nfaceCount: " << myMesh.m_data.face_Count;
	else
		std::cout << "\nFail";

	Vector3 translation(Scalar(0), Scalar(3), Scalar(0));
	Quaternion q1 = axisAngleToQuaternion(Vector3::right(), Scalar(-90));
	Scalar scale(2);
	Transform trs(translation, q1, scale);
	myMesh.applyTransform(trs);

	std::cout << "\n\nApplying transformation:\n";
	trs.print();
	std::cout << "\n\nGenerating obj called 'muzu'...\n";

	ObjHandler::save("assets/models/", "muzu", myMesh);
}

void adg::adgTests()
{
	testQuaternionConstruction();
	testQuaternionAxisAngleConversions();
	testQuaternionEulerAnglesConversions();
	testQuaternionMatrixConversions();
	testMatrixAxisAngleConversions();
	testMatrixEulerAnglesConversions();
	testSlerp();
	testAffineTransform();
	std::cout << "\n---END TEST---------------------------------------------\n\n";
}

void adg::testQuaternionConstruction()
{
	std::cout << "---TEST QUATERNION CONSTRUCTION-------------------------\n\n";

	Quaternion q1{};
	q1 = axisAngleToQuaternion(Vector3::up(), Scalar(180));
	std::cout << "Build quaternion from Axis angle: up vector and 180 deg." << cleanQuaternion(q1) << std::endl;

	Quaternion q2 = eulerToQuaternion(Scalar(0), Scalar(0), Scalar(180));
	std::cout << "Build quaternion from Euler Angles (version 1)." << cleanQuaternion(q2) << std::endl;

	Quaternion q3 = eulerToQuaternion_v2(Scalar(0), Scalar(0), Scalar(180));
	std::cout << "Build quaternion from Euler Angles (version 2)." << cleanQuaternion(q3) << std::endl;
}

void adg::testQuaternionAxisAngleConversions()
{
	std::cout << "\n---TEST QUATERNION <-> AXIS ANGLE CONVERSIONS-------------------------\n\n";
	
	Quaternion q1{};
	q1 = axisAngleToQuaternion(Vector3::forward(), Scalar(45));
	Vector3 axisAngle = quaternionToAxisAngle(q1);
	Scalar angle = toDegrees(axisAngle.norm());
	axisAngle.normalize();
	std::cout 
		<< "Build quaternion from Axis angle:\naxis: (0, 0, 1)\nangle: 45 deg.\n"
		<< "q: " << cleanQuaternion(q1) << "\n\n"
		<< "Return to Axis Angle: \n"
		<< "axis: " << axisAngle << "\nangle: " << angle << "\n";

	Vector3 v = Vector3::right();
	Vector3 v1 = cleanVector3(applyRotationOptimized(v, q1));
	std::cout 
		<< "\nTest Rotation: \nv: "
		<< v << "\nv rotated: "
		<< v1
		<< "\nApply again to compute a 90 degree rotation: \n"
		<< "v rotated: " << cleanVector3(applyRotationOptimized(v1, q1)) << "\n";
}

void adg::testQuaternionEulerAnglesConversions()
{
	std::cout << "\n---TEST QUATERNION<->EULER ANGLES CONVERSIONS--------------\n\n";

	Quaternion q = eulerToQuaternion(Scalar(20), Scalar(60), Scalar(40));
	Quaternion q_v2 = eulerToQuaternion_v2(Scalar(20), Scalar(60), Scalar(40));
	Vector3 v = Vector3::right();

	// check with other notation
	Quaternion q_pitch = axisAngleToQuaternion(Vector3::right(), Scalar(20));
	Quaternion q_yaw   = axisAngleToQuaternion(Vector3::up(), Scalar(60));
	Quaternion q_roll  = axisAngleToQuaternion(Vector3::forward(), Scalar(40));
	Quaternion qumulated = (q_yaw * q_pitch * q_roll).normalized();

	// return to eulerAngles from quaternion
	Vector3 euler_v1 = quaternionToEulerAngles(q);
	Vector3 euler_v2 = quaternionToEulerAngles(q_v2);

	std::cout
		<< "Build quaternion from Euler Angles: (20, 60, 40),\nthen apply rotation to v: "
		<< v << "\n\n"
		<< "Using version 1:\nq: "
		<< cleanQuaternion(q) << "\n"
		<< "Apply rotation:\n"
		<< "v rotated: " << cleanVector3(applyRotationOptimized(v, q)) << "\n\n"
		<< "Using version 2:\nq: "
		<< cleanQuaternion(q_v2) << "\n"
		<< "Apply rotation to v:\n"
		<< "v rotated: " << cleanVector3(applyRotationOptimized(v, q_v2)) << "\n\n"
		<< "Apply rotation using axisAngleToQuaternion to check:\n"
		<< "v rotated: " << cleanVector3(applyRotationOptimized(v, qumulated)) << "\n\n"
		<< "Return to Euler Angles:\n"
		<< "q (v_1) to euler: " << cleanVector3(euler_v1) << "\n"
		<< "q (v_2) to euler: " << cleanVector3(euler_v2) << "\n\n";
}

void adg::testQuaternionMatrixConversions()
{
	std::cout << "\n---TEST QUATERNION<->MATRIX CONVERSIONS--------------\n\n";
	Quaternion q = axisAngleToQuaternion(Vector3::up(), Scalar(-90));
	Matrix3x3 m = quaternionToMatrix3x3(q);
	Vector3 v = Vector3::right();
	Quaternion q_rec = matrix3x3ToQuaternion(m);
	
	std::cout
		<< "Convert Quaternion to a matrix:\n"
		<< "q: " << cleanQuaternion(q) << "\n"
		<< "m:\n" << cleanMatrix3x3(m)
		<< "\nTest Rotation (-90 deg around y axis) to v: " << v << "\n"
		<< "v rotated using q: " << cleanVector3(applyRotationOptimized(v, q)) << "\n"
		<< "v rotated using m: " << cleanVector3(applyMatrix3x3(m, v)) << "\n\n"
		<< "From matrix to quaternion:\nq_rec: " << cleanQuaternion(q_rec) << "\n";
}

void adg::testMatrixAxisAngleConversions()
{
	std::cout << "\n---TEST AXIS ANGLE to MATRIX CONVERSION--------------\n\n";

	Vector3 axis = Vector3(Scalar(3), Scalar(1), Scalar(7)).normalized();
	Scalar angle = Scalar(60);

	Quaternion q = cleanQuaternion(axisAngleToQuaternion(axis, angle));
	Matrix3x3 m1 = cleanMatrix3x3(quaternionToMatrix3x3(q));
	Matrix3x3 m2 = cleanMatrix3x3(axisAngleToRotationMatrix(axis, angle));

	// Check returning back
	Vector3 axisAngle_m2 = m2.toAxisAngle();
	Scalar angle_m2 = axisAngle_m2.norm();
	Vector3 axis_m2 = axisAngle_m2.normalized();

	std::cout
		<< "Build a rotation matrix from axis angle.\n"
		<< "axis: " << axis << "\n"
		<< "angle: " << angle << "\n\n"
		<< "matrix:\n" << m1 << "\n"
		<< "Check construction building first a quaternion and then converting it to a matrix\n\n"
		<< "matrix from quaternion:\n" << m2 << "\n"
		<< "Matrices are equal? " << (areEqual(m1, m2) ? "Yes" : "No") << "\n\n"
		<< "Check returning back to axis angle:\n"
		<< "axis_rec: " << axis_m2 << "\n"
		<< "angle_rec: " << toDegrees(angle_m2) << "\n\n";
}

void adg::testMatrixEulerAnglesConversions()
{
	std::cout << "\n---TEST EULER ANGLES to MATRIX CONVERSION--------------\n\n";

	Vector3 eulers = Vector3(Scalar(11), Scalar(22), Scalar(33));
	Quaternion q_e = cleanQuaternion(eulerToQuaternion(eulers.x, eulers.y, eulers.z));
	Matrix3x3 m1_e = cleanMatrix3x3(quaternionToMatrix3x3(q_e));
	Matrix3x3 m2_e = cleanMatrix3x3(eulerToRotationMatrix(eulers.x, eulers.y, eulers.z));

	// Check returning back
	Vector3 eulersRec = cleanVector3(m2_e.toEulerAngles());

	std::cout
		<< "Build a rotation matrix from eulers angles: \n"
		<< eulers << "\n"
		<< "\nmatrix:\n" << m1_e << "\n"
		<< "Check construction building first a quaternion and then converting it to a matrix\n\n"
		<< "matrix from quaternion:\n" << m2_e << "\n"
		<< "Matrices are equal? " << (areEqual(m1_e, m2_e) ? "Yes" : "No") << "\n\n"
		<< "Check returning back to eulers:\n"
		<< eulersRec << "\n\n";
}

void adg::testSlerp()
{
	std::cout << "\n---TEST SPHERICAL LERP---------------------------------\n\n";

	Vector3 axis = Vector3(Scalar(3), Scalar(1), Scalar(7)).normalized();
	Scalar angle = Scalar(70);
	Scalar halfAngle = angle / Scalar(2);
	Quaternion q1 = axisAngleToQuaternion(axis, Scalar(0));
	Quaternion q2 = axisAngleToQuaternion(axis, angle);
	Quaternion q3 = slerp(q1, q2, Scalar(0.5));
	Quaternion q4 = axisAngleToQuaternion(axis, halfAngle);
	Vector3 v = Vector3::up();

	std::cout
		<< "Construct initial orientation:\n"
		<< "q_initial: " << cleanQuaternion(q1) << "\n\n"
		<< "Construct rotation around\n" 
		<< "axis: " << axis << "\nangle: " << angle << " deg" << "\n"
		<< "q_rotation: " << cleanQuaternion(q2) << "\n\n"
		<< "Use slerp to find the intermediate quaternion (f= 0.5)\n"
		<< "q_intermediate: " << cleanQuaternion(q3) << "\n\n"
		<< "Apply rotation using q_intermediate and vector v: "
		<< v << "\n"
		<< "v rotated: " << cleanVector3(applyRotationOptimized(v, q3)) << "\n\n"
		<< "Check rotation by directly constructing the intermediate quaternion (angle: " << halfAngle << ")\n"
		<< "q_directIntermediate: " << cleanQuaternion(q4) << "\n"
		<< "v rotated: " << cleanVector3(applyRotationOptimized(v, q4)) << "\n";
}

void adg::testAffineTransform()
{
	std::cout << "\n---TEST AFFINE TRANSFORM---------------------------------\n\n";

	Quaternion q = cleanQuaternion(axisAngleToQuaternion(Vector3::right(), Scalar(50)));
	Matrix3x3 m = cleanMatrix3x3(quaternionToMatrix3x3(q));
	Scalar scale = Scalar(2);
	Vector3 t(Scalar(3), Scalar(4), Scalar(2));
	AffineTransform af(t, q, scale);

	std::cout << "Build an affine transform from:\n"
		<< "matrix:\n" << m
		<< "scale: " << scale << "\n"
		<< "translation: " << t << "\n\n";

	af.print();

	std::cout << "\n\nExport as Column Matrix4x4 for OpenGl:\n";
	float array[16]{};
	exportAsColumnMatrix4x4(af, array);
	printMatrix4x4(array);

	std::cout << "\nExport as Row Matrix4x4 for DirectX:\n";
	float array2[16]{};
	exportAsRowMatrix4x4(af, array2);
	printMatrix4x4(array2);
}

// Extra
/*
* With column vectors the basis vectors that form a linear transformation, changing coord space,
* are displayed by colums, transposing the matrix will bring back to the previous coord space.
* Vectors are pre-multiplied by matrices, so to apply trans A, B, C, the order will be CBAv
*
* With row vectors is the opposite.
* The basis vectors that represent the transformation are row vectors in the matrix
* The order to apply A, B, C is left from right: vABC
*
* In general a vector can be expressed as a linear combination of a set of basis vectors
* To apply the same transformation to column or row vectors we need to change the matrix and
* the product
*/
void adg::testColumnRowRepresentations()
{
	// Considering a vector, either row or column
	Vector3 v(Scalar(0), Scalar(0), Scalar(1));

	// Computing linear transformation example: 30 degree rotation around x axis
	Quaternion q = cleanQuaternion(axisAngleToQuaternion(Vector3::right(), Scalar(30)));
	// Converting to matrix
	Matrix3x3 m1 = cleanMatrix3x3(quaternionToMatrix3x3(q));
	std::cout << "Linear transformation: 30 degree rotation around x axis\n" << m1 << "\n";

	// Building matrix from scratch
	Matrix3x3 m2(Vector3::right(),
		Vector3(Scalar(0), cos(toRadians(Scalar(30))), sin(toRadians(Scalar(30)))),
		Vector3(Scalar(0), -sin(toRadians(Scalar(30))), cos(toRadians(Scalar(30)))));

	// To use row-major representation we need to transpose the matrix and then multiply the matrix with
	// a row vector
	Matrix3x3 m3 = m1.transposed();

	std::cout
		<< "Check previous matrix with the same matrix constructed from scratch:\n"
		<< m2 << "\n\n"
		<< "Apply matrix using column - major representation:\n"
		<< "v: " << v << "\n"
		<< "v rotated: " << cleanVector3(applyMatrix3x3(m1, v)) << "\n\n"
		<< "Apply matrix to v using row - major representation:\n"
		<< "v rotated: " << cleanVector3(applyMatrix3x3RowRep(v, m3)) << "\n";
}

void adg::testMultiplicationOrder()
{
	std::cout << "---TEST MATRIX MULTIPLICATION------------------------\n\n";

	Matrix3x3 m = cleanMatrix3x3(axisAngleToRotationMatrix(Vector3::up(), Scalar(90)));
	Matrix3x3 mx = cleanMatrix3x3(eulerToRotationMatrix(Scalar(10), Scalar(0), Scalar(0)));

	Vector3 v(Scalar(4), Scalar(5), Scalar(7));
	v.normalize();

	Vector3 v1 = cleanVector3(applyMatrix3x3(mx, v)); // pitch, then yaw
	Vector3 v2 = cleanVector3(applyMatrix3x3(m, v1));

	Matrix3x3 cumul = cleanMatrix3x3(multiplyMatrices(m, mx)); // pitch, than yaw
	Vector3 v3 = cleanVector3(applyMatrix3x3(cumul, v));

	Matrix3x3 mDirect = cleanMatrix3x3(eulerToRotationMatrix(Scalar(10), Scalar(90), Scalar(0)));
	Vector3 v4 = cleanVector3(applyMatrix3x3(mDirect, v));

	Quaternion q = cleanQuaternion(axisAngleToQuaternion(Vector3::up(), Scalar(90)));
	Quaternion q2 = cleanQuaternion(axisAngleToQuaternion(Vector3::right(), Scalar(10)));
	Quaternion q3 = cleanQuaternion((q * q2).normalized()); // pitch, than yaw
	Quaternion q4 = cleanQuaternion(eulerToQuaternion_v2(Scalar(10), Scalar(90), Scalar(0)));

	Vector3 v5 = cleanVector3(applyRotationOptimized(v, q2)); // pitch, than yaw
	Vector3 v6 = cleanVector3(applyRotationOptimized(v5, q));

	Matrix3x3 qToMCumul = cleanMatrix3x3(quaternionToMatrix3x3(q3));
	Matrix3x3 qToMDirect = cleanMatrix3x3(quaternionToMatrix3x3(q4));

	bool CumulEquals = areEqual(cumul, qToMCumul);
	bool DirectEquals = areEqual(mDirect, qToMDirect);
	bool Mixed = areEqual(cumul, qToMDirect);

	std::cout
		<< "v: " << v << "\n"
		<< "Apply matrix first m then mx: " << v2 << "\n"
		<< "Apply mCumul: " << v3 << "\n"
		<< "Apply mDirect: " << v4 << "\n"
		<< "Apply first q then q2: " << v6 << "\n"
		<< "Apply qCumul: " << cleanVector3(applyRotationOptimized(v, q3)) << "\n"
		<< "Apply qDirect: " << cleanVector3(applyRotationOptimized(v, q4)) << "\n"
		<< "CumulEquals? " << (CumulEquals ? "Yes" : "No") << "\n"
		<< "DirectEquals? " << (DirectEquals ? "Yes" : "No") << "\n"
		<< "CumulEqualsDirect? " << (Mixed ? "Yes" : "No") << "\n";
}

void adg::testGimbal()
{
	std::cout << "\n---TEST GIMBAL---------------------------------\n\n";
	Matrix3x3 m_Yaw = cleanMatrix3x3(axisAngleToRotationMatrix(Vector3::up(), Scalar(90)));
	Matrix3x3 m_Pitch = cleanMatrix3x3(axisAngleToRotationMatrix(Vector3::right(), Scalar(10)));
	Matrix3x3 m_Roll = cleanMatrix3x3(axisAngleToRotationMatrix(Vector3::forward(), Scalar(10)));
	Matrix3x3 m_trueCumul = cleanMatrix3x3(multiplyMatrices(m_Yaw, m_Pitch));
	Matrix3x3 m_cumul = cleanMatrix3x3(multiplyMatrices(m_Pitch, m_Yaw));
	Matrix3x3 m_cumul2 = cleanMatrix3x3(multiplyMatrices(m_Yaw, m_Roll));
	Matrix3x3 m_cumul3 = cleanMatrix3x3(multiplyMatrices(m_Roll, m_Yaw));
	Matrix3x3 m_direct = cleanMatrix3x3(eulerToRotationMatrix(Scalar(10), Scalar(90), Scalar(0)));
	Matrix3x3 m_direct2 = cleanMatrix3x3(eulerToRotationMatrix(Scalar(0), Scalar(90), Scalar(10)));

	Quaternion q = cleanQuaternion(eulerToQuaternion(Scalar(10), Scalar(90), Scalar(0)));
	Matrix3x3 qToM = cleanMatrix3x3(quaternionToMatrix3x3(q));

	Vector3 v(Scalar(4), Scalar(5), Scalar(7));
	v.normalize();

	std::cout
		<< "matrix yaw:\n" << m_Yaw
		<< "\nmatrix pitch:\n" << m_Pitch << "\n"
		<< "\nmatrix roll:\n" << m_Roll << "\n"
		<< "matrix (m_yaw * m_pitch):\n" << m_trueCumul
		<< "\nmatrix (m_pitch * m_yaw):\n" << m_cumul
		<< "the second one matrix technically uses the wrong order\n\n"
		<< "matrix (m_yaw * m_roll):\n" << m_cumul2
		<< "\nmatrix (m_roll * m_yaw):\n" << m_cumul3
		<< "\nmatrix direct from (10,90,0):\n" << m_direct << "\n"
		<< "\nmatrix direct from (0,90,10):\n" << m_direct2 << "\n"
		<< "\nmatrix from from q (10,90,0):\n" << qToM << "\n"
		<< "\n"
		<< " m_yp = m_py?" << (areEqual(m_trueCumul, m_cumul) ? " Yes" : " No") << "\n"
		<< " m_yp = m_yr?" << (areEqual(m_trueCumul, m_cumul2) ? " Yes" : " No") << "\n"
		<< " m_py = m_yr?" << (areEqual(m_cumul, m_cumul2) ? " Yes" : " No") << "\n"
		<< " direct = direct2?" << (areEqual(m_direct, m_direct2) ? " Yes" : " No") << "\n\n"
		<< "v: " << v << "\n"
		<< "v rotated with m_yp: " << cleanVector3(applyMatrix3x3(m_trueCumul, v))
		<< "\nv rotated with m_py: " << cleanVector3(applyMatrix3x3(m_cumul, v))
		<< "\nv rotated with m_yr: " << cleanVector3(applyMatrix3x3(m_cumul2, v))
		<< "\nv rotated with direct: " << cleanVector3(applyMatrix3x3(m_direct, v))
		<< "\nv rotated with direct2: " << cleanVector3(applyMatrix3x3(m_direct2, v)) << "\n\n";
}

void adg::testTBN()
{
	std::cout << "random sample: ";
	Vector3 randomSample = Vector3(Scalar(1), Scalar(0), Scalar(1)); // tangent space
	randomSample.normalize();
	std::cout << randomSample;
	std::cout << std::endl;
	
	// normal in view space, in Unity view space is right-handed
	// forward vector points out, in our case Z
	Vector3 vNormalVS(Scalar(0), Scalar(0), Scalar(-1));  //Vector3 vNormalVS(Scalar(0), Scalar(0.5), Scalar(0.5));
	vNormalVS.normalize();
	std::cout << "normalVS: " << vNormalVS << std::endl;

	float xRand = RndU::uniformFloat(-1, 1);
	float yRand = RndU::uniformFloat(-1, 1);

	// Vector3(xRand, yRand, 0.0);  Vector3(-0.798987, 0.601348, 0.0);
	std::cout << "random tex vector: "; // it lies in xy plane
	Vector3 vRandomVectorTex(0.798987f, 0.601348f, 0.0f); 
	vRandomVectorTex.normalize();
	std::cout << vRandomVectorTex << "\n";

	Vector3 vTangent = vRandomVectorTex - vNormalVS * dot(vNormalVS, vRandomVectorTex);
	vTangent.normalize();
	std::cout 
		<< "vTangent: " 
		<< vTangent  // vTangent: ( -0.798987, 0.601348, 0 )
		<< "\nNormal & Tangent angle between: " 
		<< angleBetweenInDegrees(vNormalVS, vTangent)
		<< "\n";

	Vector3 vBiTangent = cross(vNormalVS, vTangent);
	std::cout 
		<< "vBiTangent: " 
		<< vBiTangent    // vBiTangent: ( 0.601348, -0.798987, 0 )
		<< "\nBiTangent Length: "
		<< vBiTangent.norm() << "\n"
		<< "Bi & tangent angle between: " << angleBetweenInDegrees(vBiTangent, vTangent) << "\n"
		<< "Bi & normal angle between: " << angleBetweenInDegrees(vBiTangent, vNormalVS) << "\n\n";

	Matrix3x3 tbn = Matrix3x3(vTangent, vBiTangent, vNormalVS);
	Vector3 rotatedSample = applyMatrix3x3(tbn, randomSample);
	std::cout 
		<< "rotated sample in VS: " 
		<< rotatedSample << "\n"
		<< "rotated sample length: " << rotatedSample.norm() << "\n"
		<< "dot with normal: " << dot(rotatedSample, vNormalVS) << "\n"
		<< "dot with the original sample: " << dot(randomSample, vNormalVS) << "\n";
	

	int count = 0;
	for (int i = 0; i < 128; ++i)
	{
		float xR = RndU::uniformFloat(-1, 1);
		float yR = RndU::uniformFloat(-1, 1);
		float zR = RndU::uniformFloat(0, 1);
		Vector3 randSample(xR, yR, zR);
		randSample.normalize();

		xR = RndU::uniformFloat(-1, 1);
		yR = RndU::uniformFloat(-1, 1);
		Vector3 randTexVector(xR, yR, 0.0f);
		randTexVector.normalize();

		Vector3 tan = randTexVector - dot(vNormalVS, randTexVector) * vNormalVS;
		tan.normalize();
		Vector3 biTan = cross(vNormalVS, tan);
		Matrix3x3 m(tan, biTan, vNormalVS);
		Vector3 rotSample = applyMatrix3x3(m, randSample);
		float dotR = dot(vNormalVS, rotSample);
		if (dotR > 0.0f)
		{
			++count;
		}
	}
	std::cout << "count: " << count << "\n";
}

void adg::testTemp()
{
	Vector2 v(Scalar(0), Scalar(3));
	// std::cout << cleanVector2(rotateVectorByAngle(v, Scalar(90))) << "\n";

	// test change of basis
	Vector2 vPerp(-v.y, v.x);
	Vector2 t(Scalar(3), Scalar(0));
	Vector2 r = cleanVector2(v) * t.x + cleanVector2(vPerp) * t.y;
	std::cout << cleanVector2(r) << "\n";
	
	// test reflect
	Vector2 dirC(1.0f, 1.0f);
	Vector2 normal = Vector2::down();
	Vector2 reflected = reflect(dirC, normal);

	std::cout << "reflected: " << cleanVector2(reflected);
}

void adg::testSoccer()
{
	
}

void adg::testLineInterception2D()
{
	// test line interception
	Vector2 position(Scalar(3), Scalar(0));
	Vector2 pointOntoLine(Scalar(7), Scalar(4));

	Vector2 t(Scalar(9), Scalar(4));
	Vector2 posDir = (t - position).normalized();
	Vector2 lineDir = Vector2::left();

	Vector2 result = computeInterceptionTestWithLine(position, posDir, pointOntoLine, lineDir);
	std::cout << "intercept: " << cleanVector2(result);
}
