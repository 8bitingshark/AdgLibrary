// ----------------------------------------------------------------
// SoccerPitch example
// ----------------------------------------------------------------
#include "SoccerPitch.h"

sc::SoccerPitch::SoccerPitch() : 
	midfield(adg::Vector2::origin()),
	halfHeight(adg::Scalar(10)),
	halfWidht(adg::Scalar(20))
{
}

sc::SoccerPitch::SoccerPitch(adg::Vector2 in_midField, adg::Scalar in_halfHeight, adg::Scalar in_halfWidth) :
	midfield(in_midField),
	halfHeight(in_halfHeight),
	halfWidht(in_halfWidth)
{
}

adg::Vector2 sc::SoccerPitch::GetFieldPosition(const adg::Vector2& in_Position) const
{
	adg::Scalar topBoundary = midfield.y + halfHeight;
	adg::Scalar bottomBoundary = midfield.y - halfHeight;
	adg::Scalar leftBoundary = midfield.x + halfWidht;
	adg::Scalar rightBoundary = midfield.x - halfHeight;

	adg::Vector2 result = in_Position;

	if (result.y > topBoundary)
	{
		result.y = topBoundary;
	}
	else if (result.y < bottomBoundary)
	{
		result.y = bottomBoundary;
	}

	if (result.x > rightBoundary)
	{
		result.x = rightBoundary;
	}
	else if (result.x < leftBoundary)
	{
		result.x = leftBoundary;
	}

	return result;
}


