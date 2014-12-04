#include<stdio.h>
#include<stdlib.h>
#include<ros/ros.h>
#include "ar_localization/marker_location.h"

static char *get_buff(char *buf, int n, FILE *fp);

LocationData_T *read_LocData (char *name, int *objectnum){
	FILE *fp;
	LocationData_T *location;
	char buf[265], buf1[256];

	ROS_INFO("Openning Location File %s", name);

	if ((fp = fopen (name, "r")) == NULL){
		ROS_INFO ("Can't find the file - quitting");
		ROS_BREAK ();
	}

	get_buff(buf, 265, fp);
	if (sscanf(buf, "%d", objectnum) != 1){
		fclose(fp);
		ROS_BREAK ();
	}

	ROS_INFO ("about to load %d models", *objectnum);

	location = (LocationData_T *) malloc (sizeof (LocationData_T) * *objectnum);

	if (location == NULL){
		ROS_BREAK();
	}

	for (int i = 0; i < *objectnum; i++){
		location[i].id = i;

		get_buff(buf, 256, fp);

		if (sscanf (buf, "%s", location[i].name) != 1){
			fclose (fp);
			free (location);
			ROS_BREAK ();
		}

		ROS_INFO("reading %d marker location", i+1);

		get_buff(buf, 256, fp);

		if(sscanf (buf, "%lf %lf %lf", &location[i].marker_coord[0], &location[i].marker_coord[1], &location[i].marker_coord[2]) != 3){
			fclose (fp);
			free (location);
			ROS_BREAK ();
		}

		get_buff(buf, 256, fp);

		if(sscanf (buf, "%lf %lf %lf %lf", &location[i].marker_quat[0], &location[i].marker_quat[1], &location[i].marker_quat[2], &location[i].marker_quat[3]) != 4){
			fclose (fp);
			free (location);
			ROS_BREAK ();
		}
	}
	fclose (fp);
	return (location);
}

static char *get_buff(char *buff, int n, FILE *fp){
	char *ret;
	for(;;){
		ret = fgets (buff, n, fp);
		if(ret == NULL)
			return NULL;
		if(buff[0] != '\n' && buff[0] !='#')
			return (ret);
	}
}

