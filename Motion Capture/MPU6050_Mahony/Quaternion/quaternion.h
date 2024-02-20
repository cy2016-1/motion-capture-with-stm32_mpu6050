#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
    float w, x, y, z;
} Quaternion;

// º¯ÊýÉùÃ÷
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

void quaternion_inverseTransformation(Quaternion *My_quat);
#endif  // QUATERNION_H
