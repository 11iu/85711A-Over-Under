/*
 * Field locations for auto
 * Units are inches and degrees
 * Field reference is in the engineering notebook
 * Blue/Red colors refer to the side of the field
 * All positions are relative to the origin
 * All orientations are relative to facing the red offense side(0 deg);
 * Starting positions defined for red only
 */

struct Pose2d
{
    float x;
    float y;
    float angle;

    Pose2d(float xVal, float yVal, float angleVal)
    {
        x = xVal;
        y = yVal;
        angle = angleVal;
    }
};

struct Translation2d
{
    float x;
    float y;

    Translation2d(float xVal, float yVal)
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
Translation2d origin = Translation2d(0, 0);
Translation2d fieldCenter = Translation2d(fieldX / 2.0, fieldY / 2.0);

// elevation bars
Translation2d blueElevationHorizontalMid = Translation2d(12, tile * 3);
Translation2d redElevationHorizontalMid = Translation2d((fieldX - tile / 2.0), tile * 3);
Translation2d leftElevationVertical = Translation2d((tile), (fieldY / 2.0));
Translation2d rightElevationVertical = Translation2d((fieldX - tile), (fieldY / 2.0));

// goals
Translation2d blueGoalCenter = Translation2d((fieldX / 2.0), tile);
Translation2d redGoalCenter = Translation2d((fieldX / 2.0), (fieldY - tile));
Translation2d blueGoalLeftSide = Translation2d((tile * 2), (tile / 2.0));
Translation2d blueGoalRightSide = Translation2d((fieldX - tile * 2), (tile / 2.0));
Translation2d redGoalLeftSide = Translation2d((tile * 2), (fieldY - tile / 2.0));
Translation2d redGoalRightSide = Translation2d((fieldX - tile * 2), (fieldY - tile / 2.0));

// triballs in corners
Translation2d redLeftCornerTriball = Translation2d((tile / 4.0), (tile / 4.0));
Translation2d redRightCornerTriball = Translation2d((fieldX - tile / 4.0), (tile / 4.0));
Translation2d blueLeftCornerTriball = Translation2d((tile / 4.0), (fieldY - tile / 4.0));
Translation2d blueRightCornerTriball =
    Translation2d((fieldX - tile / 4.0), (fieldY - tile / 4.0));

// triballs to the side
Translation2d blueUnderElevationTriball = Translation2d((tile / 2.0), (fieldY / 2.0));
Translation2d redUnderElevationTriball = Translation2d((fieldX - tile / 2.0), (fieldY / 2.0));

// triballs near the center of the field
Translation2d redCenterLowerTriball = Translation2d((fieldX / 2.0), (tile * 2));
Translation2d redCenterLeftTriball =
    Translation2d((tile * 2), (tile * 3 - centerPipeToTriballsY));
Translation2d redCenterUpperTriball =
    Translation2d((fieldX / 2.0), (tile * 3 - centerPipeToTriballsY));
Translation2d blueCenterLowerTriball =
    Translation2d((fieldX / 2.0), (fieldY / 2.0 + centerPipeToTriballsY));
Translation2d blueCenterRightTriball =
    Translation2d((fieldX - tile * 2), (fieldY / 2.0 + centerPipeToTriballsY));
Translation2d blueCenterUpperTriball = Translation2d((fieldX / 2.0), (fieldY / 2.0 + tile));

// starting and ending positions for auto
Pose2d closeOppStart = Pose2d(tile / 2.0, tile + 5, 135);
Pose2d closeOppEnd = Pose2d(tile - 6, tile - 16, -30);
Pose2d closeStart = Pose2d(fieldX - closeOppStart.x, closeOppStart.y, -closeOppStart.angle);
Pose2d closeEnd = Pose2d(fieldX - closeOppEnd.x, closeOppEnd.y, -closeOppEnd.angle);
Pose2d skillsStart = Pose2d((fieldX - tile), (tile / 2.0), -30); // right side
Pose2d farStart = Pose2d((fieldX - tile / 2.0), (fieldY - tile * 1.5), -90);
Pose2d farAWPStart = Pose2d(fieldX - tile / 2.0, fieldY - tile - 5, -45);
