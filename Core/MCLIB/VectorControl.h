#include <stdint.h>
#include "math.h"
#include "GlogalVariables.h"

void VectorControlTasks(float *Idq_ref, float theta, float *Iuvw, float Vdc, float *Duty);
void OpenLoopTasks(float VamRef, float omega, float *Iuvw, float Vdc, float *Duty);
