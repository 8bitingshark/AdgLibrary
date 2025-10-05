// ----------------------------------------------------------------
// SoccerPitch example
// ----------------------------------------------------------------

#ifndef SOCCER_PITCH_H
#define SOCCER_PITCH_H
#include "../Scalar.h"
#include "../Vector2.h"
#include <ostream>

namespace sc {

	class SoccerPitch
	{
	public:
		adg::Vector2 midfield = adg::Vector2();
		adg::Scalar halfHeight = adg::Scalar(0);
		adg::Scalar halfWidht = adg::Scalar(0);

		SoccerPitch();
		SoccerPitch(adg::Vector2 in_midField, adg::Scalar in_halfHeight, adg::Scalar in_halfWidth);

		adg::Vector2 GetFieldPosition(const adg::Vector2& in_Position) const;


	};
}

#endif