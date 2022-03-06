#include "HC.h"

void HC::send_to_rasp(float error,int mode,int isarm)
{

    temp.value = error;

    _buffertx[0] = Frame_Header1;    //head1
    _buffertx[1] = Frame_Header2;    //head2


    _buffertx[protocol_len] = 4; //len is 4
    _buffertx[3] = mode;
    _buffertx[4] = isarm;

    int i = 0;
    for(i = protocol_payload; i < (protocol_payload + 4); i++){
        _buffertx[i] = temp.value_char[i - protocol_payload];
    }

    _buffertx[9] = Frame_Tail1;
    _buffertx[10] = Frame_Tail2;  

    for(i = 0;i < 11;i++){
        hal.uartD->write(_buffertx[i]);
    }

    // if (IS_ARM)
    // {
    //     _buffertx[2] = tran_angle.angle_char[0];
    //     _buffertx[3] = tran_angle.angle_char[1];
    //     _buffertx[4] = tran_angle.angle_char[2];
    //     _buffertx[5] = tran_angle.angle_char[3];
    // }
    // else
    // {
    //     _buffertx[2] = 0x00;
    //     _buffertx[3] = 0x00;
    //     _buffertx[4] = 0x00;
    //     _buffertx[5] = 0x00;
    // }


    //	hal.uartC->printf("error_angle:%f\n",error_angle);

    //	int i = 0;
    //	while(i < 8){
    //		hal.uartD->write(_buffertx[i]);
    //		hal.scheduler->delay(10);
    //		i += 1;
    //	}
    // hal.uartD->write(_buffertx[0]);
    // hal.uartD->write(_buffertx[1]);
    // hal.uartD->write(_buffertx[2]);
    // hal.uartD->write(_buffertx[3]);
    // hal.uartD->write(_buffertx[4]);
    // hal.uartD->write(_buffertx[5]);
    // hal.uartD->write(_buffertx[6]);
    // hal.uartD->write(_buffertx[7]);
    // hal.uartD->write(_buffertx[8]);
    //	for(int j = 0; j < 8; j++){
    //		hal.uartD->printf("%c\n",_buffertx[i]);
    //		hal.scheduler->delay(10);
    //	}
    //	hal.uartD->write(const uint8_t * buffer, size_t size)

    _buffertx[0] = 0x00;
    _buffertx[1] = 0x00;
    _buffertx[2] = 0x00;
    _buffertx[3] = 0x00;
    _buffertx[4] = 0x00;
    _buffertx[5] = 0x00;
    _buffertx[6] = 0x00;
    _buffertx[7] = 0x00;
    _buffertx[8] = 0x00;

    //	hal.uartC->printf("real_angle:%f\n",real_angle);
}

void HC::receive_from_rasp()
{
    // hal.uartC->printf("receive_from_rasp\n");
    f_h1_flag = 0;
    f_h_flag = 0;
    f_t1_flag = 0;
    bool pass_flag = false;
    uint32_t numc = 0;
    int16_t len = 0;
    int16_t mode = 3;
    // int16_t ARM = 4;
    int16_t payload = 5;
    // int16_t bit_max = 11;

    // union hc_sensor
    // {
    // 	float value;
    //     unsigned char value_char[4];
    // }dvl_vx,dvl_vy,dvl_vz,dvl_distance,dvl_roll,dvl_pitch,dvl_yaw,ms5837_pressure;
    // ,ms5837_depth

    // union hc_control
    // {
    // 	float value;
    // 	unsigned char value_char[4];
    // }yaw_force,depth_force,att_force;
    
    numc = hal.uartD->available(); //length of data
    // hal.uartC->printf("numc:%d\n", numc);
    // hal.uartC->printf("flag:%d\n", hc_ms5837_flag);

    uint32_t tnum = 0;
    // if ((numc + temp_pass_flag) == 11 || (numc + temp_pass_flag) == 19 || (numc + temp_pass_flag) == 23){
    //     pass_flag = true;
    //     numc = numc + temp_pass_flag;
    // }
    if ( (numc == 11) || (numc == 19) || (numc == 23)){
        pass_flag = true;
    }
    else{
        pass_flag = false;
        temp_pass_flag = numc;
    }

    uint32_t run_time;
    run_time = AP_HAL::micros() - sensor_start_time;//now - start
    if(run_time > 2000000){
        hc_ms5837_flag = false;
    }
    
                        

    if (pass_flag)
    {
        while (numc --)
        {
            _bufferrx[tnum] = hal.uartD->read();
            // hal.uartC->printf("_bufferrx[tnum]:%c\n",_bufferrx[tnum]);
            // hal.uartC->printf("while\n");
            if (f_h_flag == 1)
            { //有帧头，判断帧尾，接收消息
                if (f_t1_flag == 1)
                { //有帧头，有帧尾1
                    if (_bufferrx[tnum] == Frame_Tail2)
                    {   
                        temp_pass_flag = 0;
                        // for(i = 2; i < (tnum - 1); i++ ) // tnum < numc  so tnum - 1 is ok
                        len = _bufferrx[2];
                        if((_bufferrx[4] & 0x10) == 0x10){
                            hc_ms5837_flag = true;
                            sensor_start_time = AP_HAL::micros();
                        }
                        if((_bufferrx[4] & 0x20) == 0x20){
                            hc_dvl_vel_flag = true;
                        }
                        if((_bufferrx[4] & 0x40) == 0x40){
                            hc_dvl_pos_flag = true;
                        } 
                        
                        switch (_bufferrx[mode]){
                            case 0x50:
                                {   //depth sensor
                                    int i;
                                    for(i = payload; i < (payload + len); i++){
                                        ms5837_pressure.value_char[i-payload] = _bufferrx[i];
                                    }
                                    hc_ms5837_pressure = ms5837_pressure.value;
                                    hal.uartC->printf("pressure:%f\n",hc_ms5837_pressure); 
                                break;
                                }
                            
                            case 0x56:{
                                // int num = len / 4;
                                int i;
                                for(i = payload; i < (payload + len/4*1); i++){
                                        dvl_vx.value_char[i-payload] = _bufferrx[i];
                                    }
                                hc_dvl50_vel_x = dvl_vx.value;
                                for(i = (payload + len/4); i < (payload + len/4*2); i++){
                                        dvl_vy.value_char[i-(payload + len/4)] = _bufferrx[i];
                                    }
                                hc_dvl50_vel_y = dvl_vy.value;
                                for(i = (payload + len/4*2); i < (payload + len/4*3); i++){
                                        dvl_vz.value_char[i-(payload + len/4*2)] = _bufferrx[i];
                                    }
                                hc_dvl50_vel_z = dvl_vz.value;
                                for(i = (payload + len/4*3); i < (payload + len/4*4); i++){
                                        dvl_distance.value_char[i-(payload + len/4*3)] = _bufferrx[i];
                                    }
                                hc_dvl50_distacne = dvl_distance.value;
                                // hal.uartC->printf("distacne:%f\n",hc_dvl50_distacne); 
                                hal.uartC->printf("vx:%f vy:%f vz:%f vdistacne:%f\n",hc_dvl50_vel_x,hc_dvl50_vel_y,hc_dvl50_vel_z,hc_dvl50_distacne);
                                //dvl_v
                                break;

                            }
                            case 0x41:
                            {   //att
                                int i;
                                for(i = payload; i < (payload + len/3*1); i++){
                                        dvl_roll.value_char[i-payload] = _bufferrx[i];
                                    }
                                hc_dvl50_roll = dvl_roll.value;
                                for(i = (payload + len/3); i < (payload + len/3*2); i++){
                                        dvl_pitch.value_char[i-(payload + len/3)] = _bufferrx[i];
                                    }
                                hc_dvl50_pitch = dvl_pitch.value;
                                for(i = (payload + len/3*2); i < (payload + len/3*3); i++){
                                        dvl_yaw.value_char[i-(payload + len/3*2)] = _bufferrx[i];
                                    }
                                hc_dvl50_yaw = dvl_yaw.value;
                                // hal.uartC->printf("yaw:%f\n",hc_dvl50_yaw); 
                                hal.uartC->printf("roll:%f pitch:%f yaw:%f\n",hc_dvl50_roll,hc_dvl50_pitch,hc_dvl50_yaw); 
                                break;
                            }
                            case 0x79:
                            {   //control_yaw_robust
                                int i;
                                for(i = payload; i < (payload + len); i++){
                                        yaw_force.value_char[i-payload] = _bufferrx[i];
                                    }
                                hc_yaw_robust_force = yaw_force.value;
                                hal.uartC->printf("hc_yaw_robust_force:%f\n",hc_yaw_robust_force);
                                break;
                            }
                            case 0x64:
                            {   //control_depth_hold_robust
                                int i;
                                for(i = payload; i < (payload + len); i++){
                                        depth_force.value_char[i-payload] = _bufferrx[i];
                                    }
                                hc_depth_hold_robust_force = depth_force.value;
                                // hal.uartC->printf("hc_depth_hold_robust:%f\n",hc_depth_hold_robust_force);
                                break;
                            }
                            case 0x61:
                            {   //control_att_robust
                                int i;
                                for(i = payload; i < (payload + len); i++){
                                        att_force.value_char[i-payload] = _bufferrx[i];
                                    }
                                hc_att_robust_force = att_force.value;
                                // hal.uartC->printf("hc_att_robust:%f\n",hc_att_robust_force);
                                break;
                            }
                        }
                        

                        tnum = 0;
                    }
                    else
                    {
                        f_t1_flag = 0;
                        tnum++;
                    }
                }
                else
                { // 有帧头，无帧尾1
                    if (_bufferrx[tnum] == Frame_Tail1)
                    {
                        f_t1_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        tnum++;
                    }
                }
            }
            else
            {
                if (f_h1_flag == 1)
                {
                    if (_bufferrx[tnum] == Frame_Header2)
                    {
                        f_h_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        f_h1_flag = 0;
                        tnum = 0;
                    }
                }
                else
                {
                    if (_bufferrx[tnum] == Frame_Header1)
                    {
                        f_h1_flag = 1;
                        tnum++;
                    }
                    else
                    {
                        tnum = 0;
                    }
                }
            }

        }

    }
    else{
        while(numc--){
            hal.uartD->read();
        }
    }
    

    //	if(hc_decode(numc)){}
    //	hal.uartD->printf("receive_real_angle:%f\n",real																																																																																																																																																						_angle);

    //处理程序

    //	hc_decode(numc);
    //	if(_bufferrx[0] == Frame_Header1 && _bufferrx[1] == Frame_Header2 && _bufferrx[6] == Frame_Tail1 && _bufferrx[7] == Frame_Tail2){
    //
    //
    //		hal.uartD->write(_bufferrx[0]);
    //		hal.uartD->write(_bufferrx[1]);
    //
    //
    //		hal.uartD->write(_bufferrx[2]);
    //		hal.uartD->write(_bufferrx[3]);
    //		hal.uartD->write(_bufferrx[4]);
    //		hal.uartD->write(_bufferrx[5]);
    //
    //		hal.uartD->write(_bufferrx[6]);
    //		hal.uartD->write(_bufferrx[7]);
    //	}

    //	while(i < numc){
    //		_bufferrx[i] = 0;
    //	}
    _bufferrx[0] = 0x00;
    _bufferrx[1] = 0x00;
    _bufferrx[2] = 0x00;
    _bufferrx[3] = 0x00;
    _bufferrx[4] = 0x00;
    _bufferrx[5] = 0x00;
    _bufferrx[6] = 0x00;
    _bufferrx[7] = 0x00;
    _bufferrx[8] = 0x00;
    _bufferrx[9] = 0x00;
    _bufferrx[10] = 0x00;
    // hal.uartD->clear();
    //	hal.uartD->printf("_bufferrx is 0 ?:%d",int(_bufferrx[3]));
}

// void HC::receive_from_rasp()
// {
//     int16_t numc;
//     numc = hal.uartD->available(); //length of data
//     //	hal.uartD->printf("numc:%d\n",numc);
//     int tnum = 0;
//     if (numc)
//     {
//         while (tnum < numc)
//         {
//             _bufferrx[tnum] = hal.uartD->read();
//             //			hal.uartD->printf("receive:%c\n",_buffer[i]);
//             if (f_h_flag == 1)
//             { //有帧头，判断帧尾，接收消息
//                 if (f_t1_flag == 1)
//                 { //有帧头，有帧尾1
//                     if (_bufferrx[tnum] == Frame_Tail2)
//                     {

//                         int i = 0;
//                         for (i = 2; i < (tnum - 2); i++)
//                         {
//                             tran_force.force_char[i - 2] = _bufferrx[i];
//                         }
//                         torque = tran_force.forceX;
//                         //                        for (i = 2; i < (tnum - 1); i++)
//                         //                        {
//                         //                            hal.uartC->write(_bufferrx[i]);	// 通过串口发送字节
//                         //                        }

//                         tnum = 0;
//                     }
//                     else
//                     {
//                         f_t1_flag = 0;
//                         tnum++;
//                     }
//                 }
//                 else
//                 { // 有帧头，无帧尾1
//                     if (_bufferrx[tnum] == Frame_Tail1)
//                     {
//                         f_t1_flag = 1;
//                         tnum++;
//                     }
//                     else
//                     {
//                         tnum++;
//                     }
//                 }
//             }
//             else
//             {
//                 if (f_h1_flag == 1)
//                 {
//                     if (_bufferrx[tnum] == Frame_Header2)
//                     {
//                         f_h_flag = 1;
//                         tnum++;
//                     }
//                     else
//                     {
//                         f_h1_flag = 0;
//                         tnum = 0;
//                     }
//                 }
//                 else
//                 {
//                     if (_bufferrx[tnum] == Frame_Header1)
//                     {
//                         f_h1_flag = 1;
//                         tnum++;
//                     }
//                     else
//                     {
//                         tnum = 0;
//                     }
//                 }
//             }
//         }
//     }
//     //	if(hc_decode(numc)){}
//     //	hal.uartD->printf("receive_real_angle:%f\n",real																																																																																																																																																						_angle);

//     //处理程序

//     //	hc_decode(numc);
//     //	if(_bufferrx[0] == Frame_Header1 && _bufferrx[1] == Frame_Header2 && _bufferrx[6] == Frame_Tail1 && _bufferrx[7] == Frame_Tail2){
//     //
//     //
//     //		hal.uartD->write(_bufferrx[0]);
//     //		hal.uartD->write(_bufferrx[1]);
//     //
//     //
//     //		hal.uartD->write(_bufferrx[2]);
//     //		hal.uartD->write(_bufferrx[3]);
//     //		hal.uartD->write(_bufferrx[4]);
//     //		hal.uartD->write(_bufferrx[5]);
//     //
//     //		hal.uartD->write(_bufferrx[6]);
//     //		hal.uartD->write(_bufferrx[7]);
//     //	}

//     //	while(i < numc){
//     //		_bufferrx[i] = 0;
//     //	}
//     _bufferrx[0] = 0x00;
//     _bufferrx[1] = 0x00;
//     _bufferrx[2] = 0x00;
//     _bufferrx[3] = 0x00;
//     _bufferrx[4] = 0x00;
//     _bufferrx[5] = 0x00;
//     _bufferrx[6] = 0x00;
//     _bufferrx[7] = 0x00;

//     //	hal.uartD->printf("_bufferrx is 0 ?:%d",int(_bufferrx[3]));
// }
