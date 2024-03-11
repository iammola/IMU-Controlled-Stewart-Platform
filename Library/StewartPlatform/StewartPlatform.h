#define ARMS_COUNT 6
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

void StewartPlatform_Init(float _rodLength, float _hornLength, float shaftDistance, float anchorDistance);

void StewartPlatform_Update(Coords translation, Quaternion orientation);
