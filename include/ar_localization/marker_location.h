#ifndef MARKER_LOCATION_H
#define MARKER_LOCATION_H

typedef struct{
	char name[256];
	int id;
	double marker_coord[3];
	double marker_quat[4];
} LocationData_T;
LocationData_T *read_LocData(char *name, int *objectnum);
#endif
