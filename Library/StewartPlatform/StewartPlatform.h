#define LEGS_COUNT 6
#define AXES_COUNT 3

typedef struct Coords {
  float x;
  float y;
  float z;
} Coords;

typedef struct Quaternion {
  float w;
  float x;
  float y;
  float z;
} Quaternion;

typedef struct Legs {
  float  servoAngle;
  float  sinBeta;       // Sin of Pan angle of motors in base plate
  float  cosBeta;       // Cos of Pan angle of motors in base plate
  Coords baseJoint;     // base joints in base frame
  Coords platformJoint; // platform joints in platform frame
} Legs;

extern Legs legs[LEGS_COUNT];

void StewartPlatform_Init(float _rodLength, float _hornLength, float shaftDistance, float anchorDistance);

void StewartPlatform_Update(Coords translation, Quaternion orientation);
