/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 调好默认PID参数的DM电机封装类，继承HXC_DMCtrl
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-03 17:32:27
 */


#ifndef HXC_DMMotors_HPP
#define HXC_DMMotors_HPP
#include "HXC_DMCtrl.hpp"

namespace HXC{
//达妙3510
class DMH3510:public HXC_DMCtrl{
    public:
    DMH3510(HXC_CAN*can,int MST_ID,int CAN_ID):HXC_DMCtrl(can,MST_ID,CAN_ID){
        this->set_Pmax(12.5);
        this->set_Vmax(280);
        this->set_Tmax(1); 
        set_speed_pid(pid_param(0.00102,22*1e-6,3.1*1e-6,5,1));
        set_location_pid(pid_param(0.04362,15.66*1e-6,4.4*1e-6,60,500));
    }
    DMH3510(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid)
    :HXC_DMCtrl(can, MST_ID, CAN_ID, speed_pid, location_pid) {
        this->set_Pmax(12.5);
        this->set_Vmax(280);
        this->set_Tmax(1);
    }
};
//达妙3507
class DM3507 : public HXC_DMCtrl{
    public:
    DM3507(HXC_CAN*can,int MST_ID,int CAN_ID):HXC_DMCtrl(can,MST_ID,CAN_ID){
        this->set_Pmax(12.566);
        this->set_Vmax(50);
        this->set_Tmax(5);
        set_speed_pid(pid_param(0.001,0.0006,0,1,1));
        set_location_pid(pid_param(0.0906,15.66*1e-6,42*1e-6,150,500));
    }; 
    DM3507(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid)
    :HXC_DMCtrl(can, MST_ID, CAN_ID, speed_pid, location_pid) {
        this->set_Pmax(12.566);
        this->set_Vmax(50);
        this->set_Tmax(5);
    }
};
//达妙3519
class DM3519 : public HXC_DMCtrl{
    public:
    DM3519(HXC_CAN*can,int MST_ID,int CAN_ID):HXC_DMCtrl(can,MST_ID,CAN_ID){
        this->set_Pmax(12.5);
        this->set_Vmax(200);
        this->set_Tmax(10);
        this->speed_reduction_ratio = 19.2;
        set_speed_pid(pid_param(0.000028, 0.00003, 0.000001, 100, 1));
        set_location_pid(pid_param(0.1,0.1,0,2000,500));
    };
    // 复用基类的构造函数
    DM3519(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid)
    :HXC_DMCtrl(can, MST_ID, CAN_ID, speed_pid, location_pid) {
        this->set_Pmax(12.5);
        this->set_Vmax(200);
        this->set_Tmax(10);
        this->speed_reduction_ratio = 19.2;
    }
};

class DM6220 : public HXC_DMCtrl{
    public:
    DM6220(HXC_CAN*can,int MST_ID,int CAN_ID):HXC_DMCtrl(can,MST_ID,CAN_ID){
        this->set_Pmax(12.5);
        this->set_Vmax(45);
        this->set_Tmax(10);
        set_speed_pid(pid_param(0.00067,0.00051,0.055*1e-6,5,1));
        set_location_pid(pid_param(0.068,0.0142,0,35,500));
    };
    // 复用基类的构造函数
    DM6220(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid)
    :HXC_DMCtrl(can, MST_ID, CAN_ID, speed_pid, location_pid) {
        this->set_Pmax(12.5);
        this->set_Vmax(45);
        this->set_Tmax(10);
    }

};
}
#endif
