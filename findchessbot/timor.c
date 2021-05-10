#include <math.h>
#define pi 3.1412

// Global Variable
float C0x,C2x,C3x,C0y,C2y,C3y;
float T;

void Update_Coff(int x1,int y1,int x2,int y2,float Time)
{
    T = Time ;
    int delta_x = x2-x1;
    int delta_y = y2-y1;
    float Trajectory_Theta = atan2(delta_y, delta_x);
    float Trajectory_Magnitude = sqrt((delta_y * delta_y) + (delta_x * delta_x));
    float Time_2 = Time * Time ;
    float Time_3 = Time_2 * Time;
    C0x = delta_x;
    C2x = (3*delta_x)/Time_2;
    C3x = (2*delta_x)/Time_3;

    C0x = delta_y;
    C2x = (3*delta_y)/Time_2;
    C3x = (2*delta_y)/Time_3;
}

#define sample_time = 0.001; // 100 hz
float t = 0.;
void Control_Loop() // interrupt sample time = t
{
    float t_2 = t*t;
    float t_3 = t_2 * t;
    float Goal_position_x = C0x + (C2x*t_2) - (C3x*t_3);
    float Goal_position_y = C0y + (C2y*t_2) - (C3y*t_3);
    float Goal_velocity_x = (2*C2x*t) - (3 * C3x*t_2);
    float Goal_velocity_y = (2*C2y*t) - (3 * C3y*t_2);

    // เอาฟังก์ชั่น Control มาใส่ในนี้เลย ทำ Set point ไว้แล้ว
    
    if (t != T)
    {
        t = t + sample_time ;
    }
    else
    {
        // Stop Control Loop
    }
}