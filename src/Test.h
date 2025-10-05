// ----------------------------------------------------------------
// TESTING
// ----------------------------------------------------------------

#ifndef ADG_TEST_H
#define ADG_TEST_H

namespace adg {

	void assignmentTests();
	void testQuaternionAssignment();
	void testRotateMesh();

	void adgTests();
	void testRotationMatrixAssignment();
	void testQuaternionConstruction();
	void testQuaternionAxisAngleConversions();
	void testQuaternionEulerAnglesConversions();
	void testQuaternionMatrixConversions();
	void testMatrixAxisAngleConversions();
	void testMatrixEulerAnglesConversions();
	void testSlerp();
	void testAffineTransform();

	// Extra
	void testColumnRowRepresentations();
	void testMultiplicationOrder();
	void testGimbal();
	void testTBN();
	void testTemp();

	void testSoccer();

	// Geometry Tests
	void testLineInterception2D();

	// Collision Exercises

}

#endif