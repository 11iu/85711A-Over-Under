/*
 * Field locations for auto
 * Units are inches and degrees
 * Field reference is in the engineering notebook
 * Blue/Red colors refer to the side of the field
 * All positions are relative to the origin
 * All orientations are relative to facing the red offense side(0 deg);
 */

struct Pose2d
{
    float x;
    float y;

    Pose2d(float xVal, float yVal)
    {
        x = xVal;
        y = yVal;
    }
};

float tile = 24;
float fieldX = 144;
float fieldY = 144;
float centerPipeToTriballsY = 4; // TODO - measure this

// Field red defense left side defined as (0, 0)
Pose2d origin = Pose2d(0, 0);
Pose2d fieldCenter = Pose2d(fieldX / 2.0, fieldY / 2.0);

// elevation bars
Pose2d blueElevationHorizontalMid = Pose2d(12, tile * 3);
Pose2d redElevationHorizontalMid = Pose2d((fieldX - tile / 2.0), tile * 3);
Pose2d leftElevationVertical = Pose2d((tile), (fieldY / 2.0));
Pose2d rightElevationVertical = Pose2d((fieldX - tile), (fieldY / 2.0));

// goals
Pose2d blueGoalCenter = Pose2d((fieldX / 2.0), tile);
Pose2d redGoalCenter = Pose2d((fieldX / 2.0), (fieldY - tile));

// triballs in corners
Pose2d redLeftCornerTriball = Pose2d((tile / 4.0), (tile / 4.0));
Pose2d redRightCornerTriball = Pose2d((fieldX - tile / 4.0), (tile / 4.0));
Pose2d blueLeftCornerTriball = Pose2d((tile / 4.0), (fieldY - tile / 4.0));
Pose2d blueRightCornerTriball =
    Pose2d((fieldX - tile / 4.0), (fieldY - tile / 4.0));

// triballs to the side
Pose2d blueUnderElevationTriball = Pose2d((tile / 2.0), (fieldY / 2.0));
Pose2d redUnderElevationTriball = Pose2d((fieldX - tile / 2.0), (fieldY / 2.0));

// triballs near the center of the field
Pose2d redCenterLowerTriball = Pose2d((fieldX / 2.0), (tile * 2));
Pose2d redCenterLeftTriball =
    Pose2d((tile * 2), (tile * 3 - centerPipeToTriballsY));
Pose2d redCenterUpperTriball =
    Pose2d((fieldX / 2.0), (tile * 3 - centerPipeToTriballsY));
Pose2d blueCenterLowerTriball =
    Pose2d((fieldX / 2.0), (fieldY / 2.0 + centerPipeToTriballsY));
Pose2d blueCenterRightTriball =
    Pose2d((fieldX - tile * 2), (fieldY / 2.0 + centerPipeToTriballsY));
Pose2d blueCenterUpperTriball = Pose2d((fieldX / 2.0), (fieldY / 2.0 + tile));

// starting tiles and orientation
Pose2d redStartUpper = Pose2d((fieldX - tile / 2.0), (fieldY - tile * 1.5));
Pose2d redStartLower = Pose2d((fieldX - tile / 2.0), (tile * 1.5));
Pose2d blueStartUpper = Pose2d((tile / 2.0), (fieldY - tile * 1.5));
Pose2d blueStartLower = Pose2d((tile / 2.0), (tile * 1.5));
float redStartUpperHeading = -90;
float redStartLowerHeading = -120;
float blueStartUpperHeading = 30;
float blueStartLowerHeading = 90;

Pose2d redSkillsStart = Pose2d((fieldX - tile), (tile / 2.0));
float redSkillsStartHeading = -30; // this is a guess

// positions to push triball into the goal
Pose2d closeOppStart = Pose2d(tile / 2.0, tile + 5);
Pose2d closeOppEnd = Pose2d(tile, tile * 1.5);
float closeOppStartHeading = 135;
float closeOppEndHeading = -120;

Pose2d blueGoalLeftSide = Pose2d((tile * 2), (tile / 2.0));
Pose2d blueGoalRightSide = Pose2d((fieldX - tile * 2), (tile / 2.0));
Pose2d redGoalLeftSide = Pose2d((tile * 2), (fieldY - tile / 2.0));
Pose2d redGoalRightSide = Pose2d((fieldX - tile * 2), (fieldY - tile / 2.0));

Pose2d farAWPStart = Pose2d(fieldX - tile / 2.0, fieldY - tile - 5);
