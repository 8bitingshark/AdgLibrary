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
}

#endif