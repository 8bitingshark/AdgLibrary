// ----------------------------------------------------------------
// TESTING
// ----------------------------------------------------------------

#include "Test.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Transform.h"
#include "AffineTransform.h"
#include "meshprocess\Mesh.h"
#include "meshprocess\ObjHandler.h"
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